from __future__ import annotations

import asyncio
from typing import Any, List, Optional

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from serial.tools import list_ports

from host.webapp.backend.commands_catalog import get_commands_catalog
from host.webapp.backend.session import get_session

router = APIRouter(prefix="/api")


class RouteSelectBody(BaseModel):
    route: str


class MonitorBody(BaseModel):
    port: Optional[str] = None


class CommandBody(BaseModel):
    port: Optional[str] = None
    command: str
    args: List[str] = []
    route: str = "uart"


class VizAnchorsBody(BaseModel):
    anchors: dict[str, Any]


@router.get("/ports")
async def api_ports() -> dict:
    ports = []
    for p in list_ports.comports():
        ports.append({"device": p.device, "description": p.description or ""})
    return {"ports": ports}


@router.post("/route/select")
async def api_route_select(body: RouteSelectBody, request: Request) -> dict:
    sess = get_session(asyncio.get_running_loop())
    try:
        events = await sess.set_route(body.route)
        return {"ok": True, "events": events}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


@router.get("/monitor/status")
async def api_monitor_status(request: Request) -> dict:
    sess = get_session(asyncio.get_running_loop())
    return sess.monitor_status()


@router.post("/monitor/start")
async def api_monitor_start(body: MonitorBody, request: Request) -> dict:
    sess = get_session(asyncio.get_running_loop())
    try:
        await sess.set_monitoring(True, body.port)
        return {"ok": True}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


@router.post("/monitor/stop")
async def api_monitor_stop(request: Request) -> dict:
    sess = get_session(asyncio.get_running_loop())
    await sess.set_monitoring(False, None)
    return {"ok": True}


@router.get("/events")
async def api_events(request: Request) -> dict:
    sess = get_session(asyncio.get_running_loop())
    return {"events": sess.get_events()}


@router.post("/events/clear")
async def api_events_clear(request: Request) -> dict:
    sess = get_session(asyncio.get_running_loop())
    sess.clear_events()
    return {"ok": True}


@router.get("/events/stream")
async def api_events_stream(request: Request) -> StreamingResponse:
    sess = get_session(asyncio.get_running_loop())
    q = sess.subscribe_sse()

    async def gen():
        try:
            while True:
                if await request.is_disconnected():
                    break
                try:
                    ev, data = await asyncio.wait_for(q.get(), timeout=30.0)
                except asyncio.TimeoutError:
                    yield ": keepalive\n\n"
                    continue
                if ev == "message":
                    yield f"data: {data}\n\n"
                else:
                    yield f"event: {ev}\ndata: {data}\n\n"
        finally:
            sess.unsubscribe_sse(q)

    return StreamingResponse(
        gen(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )


@router.get("/commands")
async def api_commands() -> dict:
    commands, module_order = get_commands_catalog()
    return {"commands": commands, "module_order": module_order}


@router.post("/command")
async def api_command(body: CommandBody, request: Request) -> dict:
    sess = get_session(asyncio.get_running_loop())
    try:
        ok, text, events = await sess.command(
            body.command, body.args, body.route
        )
        return {
            "success": ok,
            "response": text,
            "events": events,
        }
    except KeyError as e:
        raise HTTPException(status_code=400, detail=f"unknown command: {e}") from e


@router.post("/viz/anchors")
async def api_viz_anchors(body: VizAnchorsBody, request: Request) -> dict:
    sess = get_session(asyncio.get_running_loop())
    sess.set_viz_anchors(body.anchors)
    return {"ok": True}


@router.post("/viz/clear")
async def api_viz_clear(request: Request) -> dict:
    sess = get_session(asyncio.get_running_loop())
    sess.set_viz_anchors({})
    return {"ok": True}
