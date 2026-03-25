"""
Minimal FastAPI backend for the host webapp: list serial ports and send UART protocol commands.
Run from repo root: uvicorn host.webapp.backend.main:app --reload --app-dir .
"""
from __future__ import annotations

import io
import json
import os
import socket
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
from collections import deque
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
_monitor_enabled = False
_response_queue: queue.Queue = queue.Queue()
_pending_expected_id: int | None = None
_pending_transport: str | None = None
_serial_lock = threading.Lock()
_command_lock = threading.Lock()
_active_route = "uart"
_recent_rx_frames = deque(maxlen=32)   # (seq, source, msg_id, payload_len)
_response_mailbox = deque(maxlen=256)  # (seq, t_monotonic, msg_id, payload, source)
_rx_seq = 0
_WAIT_DIAG_REV = "wait-v2"
_wifi_silent_resp_counts: dict[int, int] = {}

_wifi_server_host = os.getenv("HYLO_WIFI_SERVER_HOST", "0.0.0.0")
_wifi_server_port = int(os.getenv("HYLO_WIFI_SERVER_PORT", "5000"))
_wifi_server_thread: threading.Thread | None = None
_wifi_server_stop_event = threading.Event()
_wifi_server_socket: socket.socket | None = None
_wifi_client_socket: socket.socket | None = None
_wifi_client_addr: str | None = None
_wifi_lock = threading.Lock()
_wifi_last_loop_error_log_t = 0.0
# SSE: list of queues to push new event lines to (one per connected client). Use stdlib queue for thread-safe put from reader thread.
_sse_queues: list[queue.Queue] = []
_sse_queues_lock = threading.Lock()

_HIDDEN_UI_COMMANDS = {
    "TransportSetRequest",
    "TransportGetRequest",
}

# --- Indoor visualization state ---
_viz_lock = threading.Lock()
_viz_position = {"x": 0.0, "y": 0.0, "z": 0.0, "timestamp_ms": 0}
_viz_ranges: dict = {}  # addr (int) -> {"distance_m": float, "addr": int}
_viz_anchors: dict = {}  # addr (str) -> {"x": float, "y": float, "z": float, "label": str}
_VIZ_TRAIL_MAX = 200
_viz_trail: list = []  # recent positions for trail rendering

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


@app.on_event("startup")
def _startup_wifi_server() -> None:
    global _wifi_server_thread
    if _wifi_server_thread is not None and _wifi_server_thread.is_alive():
        return
    _wifi_server_stop_event.clear()
    _wifi_server_thread = threading.Thread(target=_wifi_server_loop, args=(_wifi_server_stop_event,), daemon=True)
    _wifi_server_thread.start()


@app.on_event("shutdown")
def _shutdown_wifi_server() -> None:
    global _wifi_server_thread
    _wifi_server_stop_event.set()
    if _wifi_server_thread is not None:
        _wifi_server_thread.join(timeout=2.0)
        _wifi_server_thread = None


class CommandRequest(BaseModel):
    port: str | None = None
    command: str
    args: List[str] = []
    route: str | None = None


class MonitorStartRequest(BaseModel):
    port: str | None = None


class RouteSelectRequest(BaseModel):
    route: str


def _try_extract_viz_data(msg_id, payload) -> dict | None:
    """If this message is a TwrMgrRangeEvent, extract position/ranging data for visualization and return a viz event dict."""
    global _viz_position, _viz_trail
    if _protocol_tool is None:
        return None
    try:
        from generated.protocol.python import protocol_ids
        name = protocol_ids.MSG_NAMES[msg_id] if msg_id < len(protocol_ids.MSG_NAMES) else None
        if name != "TwrMgrRangeEvent":
            return None
        cls = _protocol_tool.get_message_class(name)
        if cls is None:
            return None
        msg = cls()
        msg.ParseFromString(payload)
        addr = getattr(msg, "addr", 0)
        distance_m = getattr(msg, "distance_m", 0.0)
        x = getattr(msg, "x", 0.0)
        y = getattr(msg, "y", 0.0)
        z = getattr(msg, "z", 0.0)
        pos_unknown = getattr(msg, "position_unknown", True)
        with _viz_lock:
            _viz_ranges[addr] = {"distance_m": float(distance_m), "addr": addr}
            if not pos_unknown:
                _viz_position = {"x": float(x), "y": float(y), "z": float(z), "timestamp_ms": int(time.time() * 1000)}
                _viz_trail = (_viz_trail + [{"x": float(x), "y": float(y)}])[-_VIZ_TRAIL_MAX:]
        return {
            "type": "viz",
            "x": float(x), "y": float(y), "z": float(z),
            "position_unknown": bool(pos_unknown),
            "addr": addr, "distance_m": float(distance_m),
        }
    except Exception:
        return None


def _publish_rx_message(msg_id: int, payload: bytes, source: str) -> None:
    global _event_log
    if _protocol_tool is None:
        return

    with _serial_lock:
        matched_pending_response = (
            _pending_expected_id is not None
            and msg_id == _pending_expected_id
            and (_pending_transport is None or _pending_transport == source)
        )
        matched_silent_wifi_response = False
        if source == "wifi":
            pending_silent = _wifi_silent_resp_counts.get(msg_id, 0)
            if pending_silent > 0:
                matched_silent_wifi_response = True
                if pending_silent == 1:
                    _wifi_silent_resp_counts.pop(msg_id, None)
                else:
                    _wifi_silent_resp_counts[msg_id] = pending_silent - 1

        global _rx_seq
        _rx_seq += 1
        seq = _rx_seq
        _response_mailbox.append((seq, time.monotonic(), msg_id, bytes(payload), source))
        _recent_rx_frames.append((seq, source, msg_id, len(payload)))
        if matched_pending_response:
            _response_queue.put((msg_id, payload))

    # Match UART behavior for command/response UX: don't show WiFi command responses in Event Log.
    if source == "wifi" and (matched_pending_response or matched_silent_wifi_response):
        return

    if not _monitor_enabled:
        return

    line = "[%s] %s" % (source, _protocol_tool.format_response(msg_id, payload))
    with _event_log_lock:
        _event_log = (_event_log + [line])[-EVENT_LOG_MAX:]

    viz_event = _try_extract_viz_data(msg_id, payload)
    with _sse_queues_lock:
        for q in _sse_queues:
            try:
                q.put({"line": line, "viz": viz_event}, block=False)
            except queue.Full:
                pass


def _take_response_from_mailbox(expected_msg_id: int, min_seq: int):
    with _serial_lock:
        for idx, (seq, t_seen, msg_id, payload, source) in enumerate(_response_mailbox):
            if seq > min_seq and msg_id == expected_msg_id:
                del _response_mailbox[idx]
                return (msg_id, payload, source, seq)
    return None


def _discard_mailbox_msg_id(expected_msg_id: int):
    with _serial_lock:
        keep = [entry for entry in _response_mailbox if entry[2] != expected_msg_id]
        _response_mailbox.clear()
        _response_mailbox.extend(keep)


def _delayed_discard_mailbox_msg_id(expected_msg_id: int, delay_s: float) -> None:
    try:
        time.sleep(delay_s)
        _discard_mailbox_msg_id(expected_msg_id)
        with _serial_lock:
            _wifi_silent_resp_counts.pop(expected_msg_id, None)
    except Exception:
        pass


def _send_wifi_warmup_request(conn: socket.socket) -> None:
    """Send a lightweight request once on WiFi connect to prime the link path."""
    if _protocol_tool is None:
        return
    try:
        warmup_command = "TransportGetRequest"
        req = _protocol_tool.build_request(warmup_command, [])
        payload = req.SerializeToString()
        if len(payload) > _protocol_tool.PROTOCOL_MAX_PAYLOAD:
            return
        msg_id_req = _protocol_tool.get_msg_id_for_type(warmup_command)
        if msg_id_req is None:
            return
        resp_msg_id = _protocol_tool.get_expected_response_msg_id(warmup_command)
        if resp_msg_id is not None:
            _discard_mailbox_msg_id(resp_msg_id)
            with _serial_lock:
                _wifi_silent_resp_counts[resp_msg_id] = _wifi_silent_resp_counts.get(resp_msg_id, 0) + 1

        frame = _protocol_tool.build_frame(msg_id_req, payload)
        conn.sendall(frame)

        if resp_msg_id is not None:
            threading.Thread(
                target=_delayed_discard_mailbox_msg_id,
                args=(resp_msg_id, 1.0),
                daemon=True,
            ).start()
    except Exception:
        pass


def _wait_for_response(expected_msg_id: int, min_seq: int, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        hit = _take_response_from_mailbox(expected_msg_id, min_seq)
        if hit is not None:
            return hit
        time.sleep(0.005)
    return None


def _mailbox_diag(expected_msg_id: int) -> tuple[int, str]:
    with _serial_lock:
        count_expected = sum(1 for (_, _, mid, _, _) in _response_mailbox if mid == expected_msg_id)
        tail = list(_response_mailbox)[-8:]
    tail_text = ", ".join([f"{src}:{mid}@{seq}" for (seq, _, mid, _, src) in tail])
    return count_expected, tail_text


def _publish_tx_event(command: str, source: str) -> None:
    global _event_log
    if not _monitor_enabled:
        return
    line = "[tx:%s] %s" % (source, command)
    with _event_log_lock:
        _event_log = (_event_log + [line])[-EVENT_LOG_MAX:]
    with _sse_queues_lock:
        for q in _sse_queues:
            try:
                q.put({"line": line, "viz": None}, block=False)
            except queue.Full:
                pass


def _publish_debug_event(line: str) -> None:
    global _event_log
    if not _monitor_enabled:
        return
    with _event_log_lock:
        _event_log = (_event_log + [line])[-EVENT_LOG_MAX:]
    with _sse_queues_lock:
        for q in _sse_queues:
            try:
                q.put({"line": line, "viz": None}, block=False)
            except queue.Full:
                pass


def _publish_status_update_event() -> None:
    with _sse_queues_lock:
        for q in _sse_queues:
            try:
                q.put({"event": "status_update"}, block=False)
            except queue.Full:
                pass


def _wifi_close_client_locked() -> None:
    global _wifi_client_socket, _wifi_client_addr
    if _wifi_client_socket is not None:
        try:
            _wifi_client_socket.close()
        except Exception:
            pass
    _wifi_client_socket = None
    _wifi_client_addr = None


# main.py - Update _wifi_server_loop
def _wifi_server_loop(stop_event: threading.Event) -> None:
    global _wifi_server_socket, _wifi_client_socket, _wifi_client_addr, _wifi_last_loop_error_log_t
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((_wifi_server_host, _wifi_server_port))
    server.listen(1)
    server.settimeout(0.5)
    
    rx_buf = bytearray()

    while not stop_event.is_set():
        try:
            with _wifi_lock:
                client = _wifi_client_socket

            if client is None:
                try:
                    conn, addr = server.accept()
                    conn.settimeout(0.1)
                    with _wifi_lock:
                        _wifi_client_socket = conn
                        _wifi_client_addr = f"{addr[0]}:{addr[1]}"
                    _publish_debug_event(f"[wifi-host] New connection from {_wifi_client_addr}")
                    _publish_status_update_event()
                    _send_wifi_warmup_request(conn)
                except socket.timeout:
                    continue
            else:
                # Check if the client is still alive by attempting a tiny heartbeat
                try:
                    # MSG_PEEK | MSG_DONTWAIT checks if the connection was closed by peer
                    data = client.recv(2048)
                    if not data: # Connection closed
                        raise socket.error("Remote disconnected")
                    
                    # Process valid data
                    rx_buf.extend(data)
                    while 0x00 in rx_buf:
                        idx = rx_buf.index(0x00)
                        block = bytes(rx_buf[:idx + 1])
                        del rx_buf[:idx + 1]
                        result = _protocol_tool.parse_frame(block) if _protocol_tool else None
                        if result:
                            msg_id, payload = result
                            _publish_rx_message(msg_id, payload, "wifi")
                except socket.timeout:
                    continue
                except (socket.error, ConnectionResetError):
                    with _wifi_lock:
                        _wifi_close_client_locked()
                    rx_buf.clear()
                    _publish_status_update_event()
        except Exception as e:
            now = time.monotonic()
            if now - _wifi_last_loop_error_log_t >= 2.0:
                _wifi_last_loop_error_log_t = now
                _publish_debug_event(f"[wifi-host] loop error: {e}")
            time.sleep(0.1)


def _monitor_reader_loop(ser, stop_event: threading.Event) -> None:
    """Background thread: read COBS frames from serial, append to _event_log; if frame matches _pending_expected_id, put in _response_queue."""
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
                    _publish_rx_message(msg_id, payload, "uart")
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


def _run_send_command(port: str | None, command: str, args: List[str], route_override: str | None = None) -> tuple[bool, str, List[str]]:
    """Send a command over selected host route (uart/wifi)."""
    global _event_log, _pending_expected_id, _pending_transport
    if _protocol_tool is None:
        return False, "Protocol module not available. Run from repo root and ensure host.serial.protocol_tool is importable and protocol codegen has been run.", []

    active_route = route_override or _active_route
    if active_route == "wifi":
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
            _discard_mailbox_msg_id(resp_msg_id)
            with _serial_lock:
                start_seq = _rx_seq

            with _serial_lock:
                while True:
                    try:
                        _response_queue.get_nowait()
                    except queue.Empty:
                        break
                _pending_expected_id = resp_msg_id
                _pending_transport = "wifi"

            with _wifi_lock:
                if _wifi_client_socket is None:
                    with _serial_lock:
                        _pending_expected_id = None
                        _pending_transport = None
                    return False, "WiFi client not connected", []
                _wifi_client_socket.sendall(frame)
                _publish_tx_event(command, "wifi")

            result = _wait_for_response(resp_msg_id, start_seq, 5.0)
            if result is None:
                with _serial_lock:
                    recent = list(_recent_rx_frames)
                with _wifi_lock:
                    client = _wifi_client_addr
                recent_text = ", ".join([f"{src}:{mid}@{seq}" for (seq, src, mid, _) in recent[-8:]])
                mailbox_count, mailbox_tail = _mailbox_diag(resp_msg_id)
                return False, (
                    f"Timeout waiting for WiFi response ({_WAIT_DIAG_REV}, expected={resp_msg_id}, "
                    f"client={client}, start_seq={start_seq}, recent=[{recent_text}], mailbox_expected={mailbox_count}, "
                    f"mailbox_tail=[{mailbox_tail}])"
                ), []

            with _serial_lock:
                _pending_expected_id = None
                _pending_transport = None

            _, resp_payload, resp_source, _ = result
            resp_cls = _protocol_tool.get_message_class(resp_type)
            if resp_cls is None:
                return True, f"OK [src:{str(resp_source).upper()}]", []
            resp = resp_cls()
            resp.ParseFromString(resp_payload)
            response_text = _protocol_tool.format_message(resp_type, resp) + f" [src:{str(resp_source).upper()}]"
            # Don't return per-command events: SSE already handles event log updates.
            return True, response_text, []
        except Exception as e:
            with _serial_lock:
                _pending_expected_id = None
                _pending_transport = None
            return False, str(e), []

    if not port:
        return False, "Serial port is required for UART transport", []

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
            _discard_mailbox_msg_id(resp_msg_id)
            with _serial_lock:
                start_seq = _rx_seq
            with _serial_lock:
                while True:
                    try:
                        _response_queue.get_nowait()
                    except queue.Empty:
                        break
                _pending_expected_id = resp_msg_id
                _pending_transport = "uart"
                _monitor_serial.write(frame)
                _publish_tx_event(command, "uart")
            result = _wait_for_response(resp_msg_id, start_seq, 5.0)
            if result is None:
                with _serial_lock:
                    recent = list(_recent_rx_frames)
                recent_text = ", ".join([f"{src}:{mid}@{seq}" for (seq, src, mid, _) in recent[-8:]])
                mailbox_count, mailbox_tail = _mailbox_diag(resp_msg_id)
                return False, (
                    f"Timeout waiting for response ({_WAIT_DIAG_REV}, expected={resp_msg_id}, "
                    f"start_seq={start_seq}, recent=[{recent_text}], mailbox_expected={mailbox_count}, mailbox_tail=[{mailbox_tail}])"
                ), []

            with _serial_lock:
                _pending_expected_id = None
                _pending_transport = None
            _, resp_payload, resp_source, _ = result
            resp_cls = _protocol_tool.get_message_class(resp_type)
            if resp_cls is None:
                return True, f"OK [src:{str(resp_source).upper()}]", []
            resp = resp_cls()
            resp.ParseFromString(resp_payload)
            response_text = _protocol_tool.format_message(resp_type, resp)
            # Don't return new_events here: the SSE stream already delivered them to the frontend.
            return True, response_text, []
        except Exception as e:
            with _serial_lock:
                _pending_expected_id = None
                _pending_transport = None
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
        _publish_tx_event(command, "uart")
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
        if cmd.get("command") in _HIDDEN_UI_COMMANDS:
            continue
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
    """Yield SSE events for each new log line and viz position updates."""
    event_queue: queue.Queue = queue.Queue()
    with _sse_queues_lock:
        _sse_queues.append(event_queue)
    try:
        while True:
            try:
                item = event_queue.get(timeout=30)
            except queue.Empty:
                yield ": keepalive\n\n"
                continue
            if isinstance(item, dict):
                event_name = item.get("event")
                if event_name == "status_update":
                    yield "event: status_update\ndata: {}\n\n"
                    continue
                yield f"data: {json.dumps({'line': item.get('line', '')})}\n\n"
                viz = item.get("viz")
                if viz:
                    yield f"event: viz\ndata: {json.dumps(viz)}\n\n"
            else:
                yield f"data: {json.dumps({'line': item})}\n\n"
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
    with _wifi_lock:
        wifi_connected = _wifi_client_socket is not None
        wifi_client = _wifi_client_addr
    return {
        "active": _monitor_enabled,
        "port": _monitor_port,
        "route": _active_route,
        "wifi_server": {
            "host": _wifi_server_host,
            "port": _wifi_server_port,
            "running": _wifi_server_thread is not None and _wifi_server_thread.is_alive(),
            "connected": wifi_connected,
            "client": wifi_client,
        },
    }


@app.post("/api/route/select")
def api_route_select(req: RouteSelectRequest) -> dict:
    """Select active host command route (UART serial or WiFi TCP)."""
    global _active_route
    route = (req.route or "").strip().lower()
    if route not in ("uart", "wifi"):
        raise HTTPException(status_code=400, detail="route must be 'uart' or 'wifi'")

    _active_route = route
    _publish_status_update_event()
    return {
        "ok": True,
        "active_route": _active_route,
        "route_used": _active_route,
        "response": "Route selected",
        "events": [],
    }


@app.post("/api/monitor/start")
def api_monitor_start(req: MonitorStartRequest) -> dict:
    """Start background serial monitoring on the given port. All received frames are appended to the event log."""
    global _monitor_serial, _monitor_port, _monitor_thread, _monitor_stop_event, _monitor_enabled
    route = (_active_route or "uart").lower()

    if route == "uart":
        if serial is None:
            raise HTTPException(status_code=503, detail="pyserial not installed")
        if _protocol_tool is None:
            raise HTTPException(status_code=503, detail="Protocol module not available")
        if not req.port:
            raise HTTPException(status_code=400, detail="Select a serial port for UART monitoring")
        with _serial_lock:
            if _monitor_serial is not None:
                if _monitor_port == req.port:
                    _monitor_enabled = True
                    _publish_status_update_event()
                    return {"ok": True, "port": req.port, "route": route}
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
        _monitor_enabled = True
        _publish_status_update_event()
        return {"ok": True, "port": req.port, "route": route}

    _monitor_enabled = True
    _publish_status_update_event()
    return {"ok": True, "port": None, "route": route}


@app.post("/api/monitor/stop")
def api_monitor_stop() -> dict:
    """Stop background serial monitoring and close the port."""
    global _monitor_serial, _monitor_port, _monitor_thread, _monitor_enabled
    _monitor_enabled = False
    with _serial_lock:
        has_serial_monitor = _monitor_serial is not None
        if has_serial_monitor:
            _monitor_stop_event.set()
    if has_serial_monitor and _monitor_thread is not None:
        _monitor_thread.join(timeout=2.0)
        _monitor_thread = None
    if has_serial_monitor:
        with _serial_lock:
            try:
                if _monitor_serial is not None:
                    _monitor_serial.close()
            except Exception:
                pass
            _monitor_serial = None
            _monitor_port = None
    _publish_status_update_event()
    return {"ok": True}


@app.post("/api/command")
def run_command(req: CommandRequest) -> dict:
    """Send one protocol command and return the response and any new events from this call."""
    global _active_route
    route_used = (req.route or _active_route or "uart").strip().lower()
    if route_used not in ("uart", "wifi"):
        raise HTTPException(status_code=400, detail="route must be 'uart' or 'wifi'")

    _active_route = route_used

    if route_used == "uart" and serial is None:
        raise HTTPException(status_code=503, detail="pyserial not installed")
    if route_used == "uart" and not req.port:
        raise HTTPException(status_code=400, detail="Select a serial port for UART transport")
    with _command_lock:
        success, response, new_events = _run_send_command(req.port, req.command, req.args, route_override=route_used)
    return {
        "success": success,
        "response": response,
        "events": new_events,
        "route_used": route_used,
    }


@app.get("/api/viz/state")
def api_viz_state() -> dict:
    """Return current visualization state: tag position, ranges, anchors, and trail."""
    with _viz_lock:
        return {
            "position": dict(_viz_position),
            "ranges": {str(k): v for k, v in _viz_ranges.items()},
            "anchors": dict(_viz_anchors),
            "trail": list(_viz_trail),
        }


class AnchorConfig(BaseModel):
    anchors: dict  # addr (str) -> {"x": float, "y": float, "z": float, "label": str}


@app.post("/api/viz/anchors")
def api_viz_set_anchors(req: AnchorConfig) -> dict:
    """Set anchor positions for visualization overlay."""
    global _viz_anchors
    with _viz_lock:
        _viz_anchors = {str(k): v for k, v in req.anchors.items()}
    return {"ok": True, "count": len(_viz_anchors)}


@app.post("/api/viz/clear")
def api_viz_clear() -> dict:
    """Clear visualization trail and ranges."""
    global _viz_trail, _viz_ranges, _viz_position
    with _viz_lock:
        _viz_trail = []
        _viz_ranges = {}
        _viz_position = {"x": 0.0, "y": 0.0, "z": 0.0, "timestamp_ms": 0}
    return {"ok": True}


@app.get("/")
def index():
    """Serve the single-page frontend."""
    index_path = os.path.join(STATIC_DIR, "index.html")
    if os.path.isfile(index_path):
        response = FileResponse(index_path)
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        return response
    return {"message": "Host webapp API. Put index.html in backend/static/ or mount a frontend."}
