"""
Minimal FastAPI backend for the host webapp: list serial ports and send UART protocol commands.
Run from repo root: uvicorn host.webapp.backend.main:app --reload --app-dir .
"""
from __future__ import annotations

import io
import os
import sys

# Repo root and generated protocol for protocol_tool imports
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
if os.path.join(REPO_ROOT, "generated", "protocol", "python") not in sys.path:
    sys.path.insert(0, os.path.join(REPO_ROOT, "generated", "protocol", "python"))

from contextlib import redirect_stdout
from typing import List

from fastapi import FastAPI, HTTPException

# In-memory event log (interleaved messages from serial, e.g. log events). Max entries to avoid unbounded growth.
EVENT_LOG_MAX = 1000
_event_log: List[str] = []
from fastapi.responses import FileResponse
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
    """Call protocol_tool.send_command and capture printed response and any interleaved events."""
    if _protocol_tool is None:
        return False, "Protocol module not available. Run from repo root and ensure host.serial.protocol_tool is importable and protocol codegen has been run.", []
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
        global _event_log
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
    return {"events": list(_event_log)}


@app.post("/api/events/clear")
def api_clear_events() -> dict:
    """Clear the event log."""
    global _event_log
    _event_log = []
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
