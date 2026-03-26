"""
Single host session: UART + TCP, framing, protobuf, monitors, SSE fan-out.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import serial as pyserial

from host.webapp.backend.commands_catalog import build_request
from host.webapp.backend.framing import FrameDecoder, build_framed_message
from host.webapp.backend.protobuf_codec import ParsedFrame, codec
from host.webapp.backend.response_format import format_command_response
from host.webapp.backend.stream_fanout import StreamFanout

EVENTS_MAX = 5000
SSE_QUEUE_SIZE = 80
COMMAND_TIMEOUT_S = 8.0

_log = logging.getLogger(__name__)


def _wifi_bind() -> str:
    return os.environ.get("HYLO_WIFI_BIND", "0.0.0.0")


def _wifi_port() -> int:
    return int(os.environ.get("HYLO_WIFI_PORT", "5000"))


SubscriberQueue = asyncio.Queue


class HostSession:
    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self._queue: asyncio.Queue[Tuple[int, bytes]] = asyncio.Queue(maxsize=4096)
        self.fanout = StreamFanout()
        self.codec = codec

        self.route: str = "uart"
        self._serial: Optional[pyserial.Serial] = None
        self._serial_thread: Optional[threading.Thread] = None
        self._serial_stop = threading.Event()
        self._serial_decoder = FrameDecoder()

        self._tcp_server: Any = None
        self._tcp_reader_task: Optional[asyncio.Task] = None
        self._tcp_writer: Optional[asyncio.StreamWriter] = None
        self._tcp_decoder = FrameDecoder()

        self._write_lock = asyncio.Lock()
        self._monitoring = False
        self._events: Deque[str] = deque(maxlen=EVENTS_MAX)

        self._pump_task: Optional[asyncio.Task] = None
        self._pending_cmd: Optional[Tuple[str, asyncio.Future]] = None

        self._sse_subs: List[SubscriberQueue] = []
        self._viz_anchors: Dict[str, Any] = {}

        self.wifi_client_addr: Optional[str] = None

    def _enqueue(self, msg_id: int, payload: bytes) -> None:
        def put() -> None:
            try:
                self._queue.put_nowait((msg_id, payload))
            except asyncio.QueueFull:
                try:
                    self._queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
                try:
                    self._queue.put_nowait((msg_id, payload))
                except asyncio.QueueFull:
                    pass

        self._loop.call_soon_threadsafe(put)

    async def start_pump(self) -> None:
        if self._pump_task and not self._pump_task.done():
            return
        self._pump_task = asyncio.create_task(self._pump_loop(), name="frame_pump")

    async def _pump_loop(self) -> None:
        while True:
            try:
                msg_id, payload = await self._queue.get()
                await self._handle_frame(msg_id, payload)
            except Exception:
                _log.exception("frame pump")
                await asyncio.sleep(0.01)

    async def _handle_frame(self, msg_id: int, payload: bytes) -> None:
        try:
            pf = self.codec.parse(msg_id, payload)
        except Exception as e:
            _log.warning("frame parse error id=%s: %s", msg_id, e)
            return

        if self.codec.is_stream(msg_id):
            await self.fanout.push(msg_id, payload)
            return

        if self._pending_cmd and not self._pending_cmd[1].done():
            req_name, fut = self._pending_cmd
            if self.codec.matches_command_response(req_name, pf.msg_id):
                fut.set_result(pf)

        if self.codec.is_event(msg_id):
            line = self.codec.format_log_line(pf)
            if line:
                await self._maybe_log(line)

        viz = self.codec.to_viz_payload(pf)
        if viz:
            await self._broadcast_sse_event("viz", viz)

        fusion = self.codec.to_fusion_payload(pf)
        if fusion:
            await self._broadcast_sse_event("fusion", fusion)

    async def _maybe_log(self, line: str) -> None:
        if not self._monitoring:
            return
        self._events.append(line)
        await self._broadcast_sse_message(line)

    async def _broadcast_sse_message(self, line: str) -> None:
        payload = json.dumps({"line": line})
        dead: List[SubscriberQueue] = []
        for q in self._sse_subs:
            try:
                while q.full():
                    try:
                        q.get_nowait()
                    except asyncio.QueueEmpty:
                        break
                q.put_nowait(("message", payload))
            except Exception:
                dead.append(q)
        for q in dead:
            if q in self._sse_subs:
                self._sse_subs.remove(q)

    async def _broadcast_sse_event(self, event: str, data: dict) -> None:
        if not self._sse_subs:
            return
        payload = json.dumps(data)
        dead: List[SubscriberQueue] = []
        for q in self._sse_subs:
            try:
                while q.full():
                    try:
                        q.get_nowait()
                    except asyncio.QueueEmpty:
                        break
                q.put_nowait((event, payload))
            except Exception:
                dead.append(q)
        for q in dead:
            if q in self._sse_subs:
                self._sse_subs.remove(q)

    async def _broadcast_status(self) -> None:
        dead: List[SubscriberQueue] = []
        for q in self._sse_subs:
            try:
                while q.full():
                    try:
                        q.get_nowait()
                    except asyncio.QueueEmpty:
                        break
                q.put_nowait(("status_update", "{}"))
            except Exception:
                dead.append(q)
        for q in dead:
            if q in self._sse_subs:
                self._sse_subs.remove(q)

    def subscribe_sse(self) -> SubscriberQueue:
        q: SubscriberQueue = asyncio.Queue(maxsize=SSE_QUEUE_SIZE)
        self._sse_subs.append(q)
        return q

    def unsubscribe_sse(self, q: SubscriberQueue) -> None:
        if q in self._sse_subs:
            self._sse_subs.remove(q)

    async def set_route(self, route: str) -> List[str]:
        route = route.lower()
        if route not in ("uart", "wifi"):
            raise ValueError("route must be uart or wifi")
        events: List[str] = []
        if route == self.route:
            return events
        await self._stop_wifi_server()
        await self._stop_serial()
        self.route = route
        events.append(f"[host] Route set to {route.upper()}")
        if route == "wifi":
            await self._start_wifi_server()
            events.append(
                f"[host] WiFi TCP server {_wifi_bind()}:{_wifi_port()} "
                "(device connects outbound)"
            )
        await self._broadcast_status()
        return events

    async def _start_wifi_server(self) -> None:
        if self._tcp_server:
            return

        async def _client(
            reader: asyncio.StreamReader, writer: asyncio.StreamWriter
        ) -> None:
            addr = writer.get_extra_info("peername")
            self.wifi_client_addr = str(addr) if addr else None
            self._tcp_decoder.reset()
            if self._tcp_reader_task and not self._tcp_reader_task.done():
                self._tcp_reader_task.cancel()
                try:
                    await self._tcp_reader_task
                except asyncio.CancelledError:
                    pass
            self._tcp_writer = writer
            self._tcp_reader_task = asyncio.create_task(
                self._tcp_read_loop(reader), name="tcp_read"
            )
            await self._broadcast_status()

        self._tcp_server = await asyncio.start_server(
            _client, _wifi_bind(), _wifi_port()
        )
        await self.start_pump()

    async def _tcp_read_loop(self, reader: asyncio.StreamReader) -> None:
        try:
            while True:
                chunk = await reader.read(4096)
                if not chunk:
                    break
                for msg_id, payload in self._tcp_decoder.feed(chunk):
                    self._enqueue(msg_id, payload)
        except asyncio.CancelledError:
            raise
        except Exception:
            pass
        finally:
            self._tcp_writer = None
            self.wifi_client_addr = None
            self._tcp_decoder.reset()
            await self._broadcast_status()

    async def _stop_wifi_server(self) -> None:
        if self._tcp_reader_task and not self._tcp_reader_task.done():
            self._tcp_reader_task.cancel()
            try:
                await self._tcp_reader_task
            except asyncio.CancelledError:
                pass
        self._tcp_reader_task = None
        if self._tcp_writer:
            try:
                self._tcp_writer.close()
                await self._tcp_writer.wait_closed()
            except Exception:
                pass
        self._tcp_writer = None
        if self._tcp_server:
            self._tcp_server.close()
            await self._tcp_server.wait_closed()
            self._tcp_server = None
        self.wifi_client_addr = None
        self._tcp_decoder.reset()

    def _serial_worker(self, port: str) -> None:
        del port
        try:
            while not self._serial_stop.is_set():
                ser = self._serial
                if ser is None or not ser.is_open:
                    break
                try:
                    chunk = ser.read(4096)
                    if not chunk:
                        continue
                    for msg_id, payload in self._serial_decoder.feed(chunk):
                        self._enqueue(msg_id, payload)
                except Exception:
                    break
        except Exception:
            pass

    async def start_serial(self, port: str) -> None:
        await self._stop_serial()
        try:
            self._serial = pyserial.Serial(port, baudrate=115200, timeout=0.05)
        except Exception as e:
            raise RuntimeError(f"open serial failed: {e}") from e
        self._serial_decoder.reset()
        self._serial_stop.clear()
        self._serial_thread = threading.Thread(
            target=self._serial_worker, args=(port,), daemon=True
        )
        self._serial_thread.start()
        await self.start_pump()

    async def _stop_serial(self) -> None:
        self._serial_stop.set()
        if self._serial_thread and self._serial_thread.is_alive():
            self._serial_thread.join(timeout=2.0)
        self._serial_thread = None
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self._serial_decoder.reset()

    async def set_monitoring(self, active: bool, port: Optional[str]) -> None:
        self._monitoring = active
        if not active:
            if self.route == "uart":
                await self._stop_serial()
            await self._broadcast_status()
            return
        if self.route == "uart":
            if not port:
                raise ValueError("port required for UART")
            await self.start_serial(port)
        await self._broadcast_status()

    def clear_events(self) -> None:
        self._events.clear()

    def get_events(self) -> List[str]:
        return list(self._events)

    def set_viz_anchors(self, anchors: Dict[str, Any]) -> None:
        self._viz_anchors = dict(anchors)

    def get_viz_anchors(self) -> Dict[str, Any]:
        return self._viz_anchors

    async def _write_bytes(self, data: bytes) -> None:
        if self.route == "uart":
            if not self._serial or not self._serial.is_open:
                raise RuntimeError("serial not open")
            self._serial.write(data)
            self._serial.flush()
        else:
            if not self._tcp_writer:
                raise RuntimeError("no WiFi/TCP client connected")
            self._tcp_writer.write(data)
            await self._tcp_writer.drain()

    async def send_frame(self, msg_id: int, payload: bytes) -> None:
        data = build_framed_message(msg_id, payload)
        async with self._write_lock:
            await self._write_bytes(data)

    async def command(
        self, request_name: str, arg_strings: List[str], route: str
    ) -> Tuple[bool, str, List[str]]:
        route = route.lower()
        if route != self.route:
            return False, "route mismatch; call /api/route/select first", []
        if route == "uart" and (not self._serial or not self._serial.is_open):
            return (
                False,
                "Serial not open — use 'Open serial & show events' in the toolbar after choosing a port.",
                [],
            )
        if route == "wifi" and not self._tcp_writer:
            return (
                False,
                "no TCP peer (device must connect to host WiFi server)",
                [],
            )

        msg = build_request(request_name, arg_strings)
        mid, payload = self.codec.serialize(msg)
        fut: asyncio.Future = self._loop.create_future()
        self._pending_cmd = (request_name, fut)
        try:
            await self.send_frame(mid, payload)
            pf: ParsedFrame = await asyncio.wait_for(fut, timeout=COMMAND_TIMEOUT_S)
            if pf.message is None:
                return True, pf.name, []
            return True, format_command_response(pf.message), []
        except asyncio.TimeoutError:
            return False, "timeout waiting for response", []
        except Exception as e:
            return False, str(e), []
        finally:
            self._pending_cmd = None

    def monitor_status(self) -> dict:
        port_name = None
        if self._serial and self._serial.is_open:
            port_name = getattr(self._serial, "port", None)
        return {
            "active": self._monitoring,
            "route": self.route,
            "port": port_name,
            "wifi_server": {
                "host": _wifi_bind(),
                "port": _wifi_port(),
                "connected": self._tcp_writer is not None,
                "client": self.wifi_client_addr or "",
            },
        }


_session: Optional[HostSession] = None


def get_session(loop: asyncio.AbstractEventLoop) -> HostSession:
    global _session
    if _session is None:
        _session = HostSession(loop)
    return _session
