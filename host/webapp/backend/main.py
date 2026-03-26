from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from host.webapp.backend.routers import api
from host.webapp.backend.session import get_session

STATIC_DIR = Path(__file__).resolve().parent / "static"


@asynccontextmanager
async def lifespan(app: FastAPI):
    loop = asyncio.get_running_loop()
    sess = get_session(loop)
    await sess.start_pump()
    yield


app = FastAPI(title="Hylo host", lifespan=lifespan)
app.include_router(api.router)


@app.websocket("/ws/stream")
async def websocket_stream(ws: WebSocket):
    sess = get_session(asyncio.get_running_loop())
    q = await sess.fanout.register(ws)
    try:
        while True:
            data = await q.get()
            await ws.send_bytes(data)
    except WebSocketDisconnect:
        pass
    finally:
        await sess.fanout.unregister(ws)


@app.get("/")
async def root_index():
    return FileResponse(STATIC_DIR / "index.html")


static_has_files = STATIC_DIR.is_dir() and any(STATIC_DIR.iterdir())
if static_has_files:
    app.mount(
        "/static",
        StaticFiles(directory=str(STATIC_DIR)),
        name="static_assets",
    )
