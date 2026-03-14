"""
Minimal FastAPI backend for the host webapp: list serial ports and send UART protocol commands.
Run from repo root: uvicorn host.webapp.backend.main:app --reload --app-dir .
"""
from __future__ import annotations

import io
import json
import os
import sys

# Repo root and generated protocol for protocol_tool imports
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
if os.path.join(REPO_ROOT, "generated", "protocol", "python") not in sys.path:
    sys.path.insert(0, os.path.join(REPO_ROOT, "generated", "protocol", "python"))

import queue
import threading
import time
from contextlib import redirect_stdout
from typing import List

from fastapi import FastAPI, HTTPException

# In-memory event log (interleaved messages from serial, e.g. log events). Max entries to avoid unbounded growth.
EVENT_LOG_MAX = 1000
_event_log: List[str] = []
_event_log_lock = threading.Lock()
# Background monitor: one serial port open, reader thread pushes all frames to _event_log; commands use same port.
_monitor_serial = None
_monitor_port: str | None = None
_monitor_thread: threading.Thread | None = None
_monitor_stop_event = threading.Event()
_response_queue: queue.Queue = queue.Queue()
_pending_expected_id: int | None = None
_serial_lock = threading.Lock()
# SSE: list of queues to push new event lines to (one per connected client). Use stdlib queue for thread-safe put from reader thread.
_sse_queues: list[queue.Queue] = []
_sse_queues_lock = threading.Lock()

from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

try:
    import serial
    import serial.tools.list_ports as list_ports_mod
except ImportError:
    serial = None
    list_ports_mod = None

# Import protocol_ui first so host_options_pb2 is loaded before protocol_pb2 (extension registration)
try:
    from host.webapp.backend import protocol_ui as _protocol_ui
except Exception:
    _protocol_ui = None

# Optional: import protocol tool for get_commands() and send_command()
_protocol_tool = None
try:
    from host.serial import protocol_tool as _protocol_tool
except Exception:
    pass

app = FastAPI(title="Hylo host webapp", version="0.1.0")

# Serve static frontend
STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")
if os.path.isdir(STATIC_DIR):
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


class CommandRequest(BaseModel):
    port: str
    command: str
    args: List[str] = []


class MonitorStartRequest(BaseModel):
    port: str


def _monitor_reader_loop(ser, stop_event: threading.Event) -> None:
    """Background thread: read COBS frames from serial, append to _event_log; if frame matches _pending_expected_id, put in _response_queue."""
    global _event_log
    buf = bytearray()
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                buf.extend(ser.read(ser.in_waiting))
            while 0x00 in buf:
                idx = buf.index(0x00)
                block = bytes(buf[: idx + 1])
                del buf[: idx + 1]
                result = _protocol_tool.parse_frame(block) if _protocol_tool else None
                if result:
                    msg_id, payload = result
                    line = "[rx] %s" % _protocol_tool.format_response(msg_id, payload)
                    with _event_log_lock:
                        _event_log = (_event_log + [line])[-EVENT_LOG_MAX:]
                    with _sse_queues_lock:
                        for q in _sse_queues:
                            try:
                                q.put(line, block=False)
                            except queue.Full:
                                pass
                    with _serial_lock:
                        if _pending_expected_id is not None and msg_id == _pending_expected_id:
                            _response_queue.put((msg_id, payload))
            time.sleep(0.01)
        except Exception:
            if not stop_event.is_set():
                try:
                    _response_queue.put(None)  # signal error
                except Exception:
                    pass
            break


@app.get("/api/ports")
def list_ports() -> dict:
    """List available serial ports for the UI dropdown."""
    if serial is None or list_ports_mod is None:
        raise HTTPException(status_code=503, detail="pyserial not installed")
    ports = []
    for p in list_ports_mod.comports():
        ports.append({"device": p.device, "description": p.description or p.device})
    return {"ports": ports}


def _run_send_command(port: str, command: str, args: List[str]) -> tuple[bool, str, List[str]]:
    """Send a command. If background monitoring is active on this port, use shared serial; else open/close per call."""
    global _event_log, _pending_expected_id
    if _protocol_tool is None:
        return False, "Protocol module not available. Run from repo root and ensure host.serial.protocol_tool is importable and protocol codegen has been run.", []

    # Use shared serial when monitoring is active on this port
    if _monitor_serial is not None and _monitor_port == port:
        with _event_log_lock:
            prev_len = len(_event_log)
        try:
            req = _protocol_tool.build_request(command, args)
            payload = req.SerializeToString()
            if len(payload) > _protocol_tool.PROTOCOL_MAX_PAYLOAD:
                return False, "Payload too large", []
            msg_id_req = _protocol_tool.get_msg_id_for_type(command)
            if msg_id_req is None:
                return False, "Unknown command", []
            resp_msg_id = _protocol_tool.get_expected_response_msg_id(command)
            resp_type = _protocol_tool.get_response_type_for_request(command)
            if resp_msg_id is None:
                return False, "Unknown response type", []
            frame = _protocol_tool.build_frame(msg_id_req, payload)
            with _serial_lock:
                while True:
                    try:
                        _response_queue.get_nowait()
                    except queue.Empty:
                        break
                _pending_expected_id = resp_msg_id
                _monitor_serial.write(frame)
            try:
                result = _response_queue.get(timeout=5.0)
            except queue.Empty:
                return False, "Timeout waiting for response", []
            finally:
                with _serial_lock:
                    _pending_expected_id = None
            if result is None:
                return False, "Serial read error", []
            msg_id, resp_payload = result
            resp_cls = _protocol_tool.get_message_class(resp_type)
            if resp_cls is None:
                return True, "OK", []
            resp = resp_cls()
            resp.ParseFromString(resp_payload)
            response_text = _protocol_tool.format_message(resp_type, resp)
            with _event_log_lock:
                new_events = list(_event_log[prev_len:])
            return True, response_text, new_events
        except Exception as e:
            with _serial_lock:
                _pending_expected_id = None
            return False, str(e), []

    # No monitoring: open port, send, read response, close
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
    except Exception as e:
        return False, f"Serial open failed: {e}", []
    events_this_call: List[str] = []
    try:
        out = io.StringIO()
        with redirect_stdout(out):
            ok = _protocol_tool.send_command(ser, command, args, timeout_s=5.0, event_list=events_this_call)
        response_text = out.getvalue().strip() or ("OK" if ok else "Timeout or error")
        with _event_log_lock:
            _event_log = (_event_log + events_this_call)[-EVENT_LOG_MAX:]
        return ok, response_text, events_this_call
    except Exception as e:
        return False, str(e), []
    finally:
        try:
            ser.close()
        except Exception:
            pass


def _commands_list():
    """List of commands from descriptor, each with command, description, args, module (for UI grouping)."""
    if _protocol_tool is None:
        return []
    raw = _protocol_tool.get_commands()
    msg_types = None
    if _protocol_tool and hasattr(_protocol_tool, "pb2") and _protocol_tool.pb2 is not None:
        msg_types = getattr(_protocol_tool.pb2.DESCRIPTOR, "message_types_by_name", None)
    out = []
    for cmd in raw:
        rec = {
            "command": cmd["command"],
            "description": cmd["command"],
            "args": cmd["args"],
        }
        if _protocol_ui is not None and msg_types is not None:
            module_label, _ = _protocol_ui.get_module_for_message(cmd["command"], msg_types)
            rec["module"] = module_label
        else:
            rec["module"] = "Other"
        out.append(rec)
    out.sort(key=lambda c: c["command"].lower())
    return out


@app.get("/api/commands")
def api_get_commands() -> dict:
    """Return commands and module_order for UI grouping (sections ordered by module_order)."""
    commands = _commands_list()
    module_order = _protocol_ui.get_module_order() if _protocol_ui is not None else ["Other"]
    return {"commands": commands, "module_order": module_order}


@app.get("/api/events")
def api_get_events() -> dict:
    """Return the event log (interleaved messages from serial, e.g. log events) for the event log panel."""
    with _event_log_lock:
        return {"events": list(_event_log)}


def _event_stream_generator():
    """Yield SSE events for each new log line. Reader thread pushes to our queue (thread-safe stdlib queue)."""
    event_queue: queue.Queue = queue.Queue()
    with _sse_queues_lock:
        _sse_queues.append(event_queue)
    try:
        while True:
            try:
                line = event_queue.get(timeout=30)
            except queue.Empty:
                yield ": keepalive\n\n"
                continue
            yield f"data: {json.dumps({'line': line})}\n\n"
    finally:
        with _sse_queues_lock:
            if event_queue in _sse_queues:
                _sse_queues.remove(event_queue)


@app.get("/api/events/stream")
async def api_events_stream():
    """Server-Sent Events stream: new event log lines are pushed as they arrive (no polling)."""
    return StreamingResponse(
        _event_stream_generator(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@app.post("/api/events/clear")
def api_clear_events() -> dict:
    """Clear the event log."""
    global _event_log
    with _event_log_lock:
        _event_log = []
    return {"ok": True}


@app.get("/api/monitor/status")
def api_monitor_status() -> dict:
    """Return whether background serial monitoring is active and on which port."""
    return {"active": _monitor_serial is not None, "port": _monitor_port}


@app.post("/api/monitor/start")
def api_monitor_start(req: MonitorStartRequest) -> dict:
    """Start background serial monitoring on the given port. All received frames are appended to the event log."""
    global _monitor_serial, _monitor_port, _monitor_thread, _monitor_stop_event
    if serial is None:
        raise HTTPException(status_code=503, detail="pyserial not installed")
    if _protocol_tool is None:
        raise HTTPException(status_code=503, detail="Protocol module not available")
    with _serial_lock:
        if _monitor_serial is not None:
            if _monitor_port == req.port:
                return {"ok": True, "message": "Already monitoring this port"}
            _monitor_stop_event.set()
            if _monitor_thread is not None:
                _monitor_thread.join(timeout=2.0)
            try:
                _monitor_serial.close()
            except Exception:
                pass
            _monitor_serial = None
            _monitor_port = None
            _monitor_thread = None
    try:
        ser = serial.Serial(req.port, 115200, timeout=0.01)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Serial open failed: {e}")
    _monitor_stop_event = threading.Event()
    _monitor_serial = ser
    _monitor_port = req.port
    _monitor_thread = threading.Thread(target=_monitor_reader_loop, args=(ser, _monitor_stop_event), daemon=True)
    _monitor_thread.start()
    return {"ok": True, "port": req.port}


@app.post("/api/monitor/stop")
def api_monitor_stop() -> dict:
    """Stop background serial monitoring and close the port."""
    global _monitor_serial, _monitor_port, _monitor_thread
    with _serial_lock:
        if _monitor_serial is None:
            return {"ok": True, "message": "Not monitoring"}
        _monitor_stop_event.set()
    if _monitor_thread is not None:
        _monitor_thread.join(timeout=2.0)
        _monitor_thread = None
    with _serial_lock:
        try:
            if _monitor_serial is not None:
                _monitor_serial.close()
        except Exception:
            pass
        _monitor_serial = None
        _monitor_port = None
    return {"ok": True}


@app.post("/api/command")
def run_command(req: CommandRequest) -> dict:
    """Send one protocol command and return the response and any new events from this call."""
    if serial is None:
        raise HTTPException(status_code=503, detail="pyserial not installed")
    success, response, new_events = _run_send_command(req.port, req.command, req.args)
    return {"success": success, "response": response, "events": new_events}


@app.get("/")
def index():
    """Serve the single-page frontend."""
    index_path = os.path.join(STATIC_DIR, "index.html")
    if os.path.isfile(index_path):
        response = FileResponse(index_path)
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        return response
    return {"message": "Host webapp API. Put index.html in backend/static/ or mount a frontend."}
