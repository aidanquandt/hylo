"""
High-rate stream path: ring buffer + WebSocket fan-out with drop-old backpressure.
"""
from __future__ import annotations

import asyncio
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Set

from fastapi import WebSocket

STREAM_RING_MAX = 512


@dataclass
class StreamRecord:
    t_mono: float
    msg_id: int
    payload: bytes


class StreamFanout:
    def __init__(self, ring_max: int = STREAM_RING_MAX) -> None:
        self._ring: Deque[StreamRecord] = deque(maxlen=ring_max)
        self._lock = asyncio.Lock()
        self._subs: Set[WebSocket] = set()
        self._sub_queues: dict[WebSocket, asyncio.Queue] = {}

    async def push(self, msg_id: int, payload: bytes) -> None:
        rec = StreamRecord(time.monotonic(), msg_id, payload)
        async with self._lock:
            self._ring.append(rec)
            subs_snapshot = list(self._sub_queues.items())
        dead: list[WebSocket] = []
        data = self._encode_ws_frame(msg_id, payload)
        for ws, q in subs_snapshot:
            try:
                while q.full():
                    try:
                        q.get_nowait()
                    except asyncio.QueueEmpty:
                        break
                q.put_nowait(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            await self.unregister(ws)

    def snapshot_ring(self) -> list[StreamRecord]:
        return list(self._ring)

    @staticmethod
    def _encode_ws_frame(msg_id: int, payload: bytes) -> bytes:
        return bytes([msg_id & 0xFF, (msg_id >> 8) & 0xFF]) + payload

    @staticmethod
    def decode_ws_frame(data: bytes) -> tuple[int, bytes]:
        if len(data) < 2:
            raise ValueError("short frame")
        msg_id = data[0] | (data[1] << 8)
        return msg_id, data[2:]

    async def register(self, ws: WebSocket) -> asyncio.Queue:
        await ws.accept()
        q: asyncio.Queue = asyncio.Queue(maxsize=64)
        async with self._lock:
            self._subs.add(ws)
            self._sub_queues[ws] = q
        return q

    async def unregister(self, ws: WebSocket) -> None:
        async with self._lock:
            self._subs.discard(ws)
            self._sub_queues.pop(ws, None)
