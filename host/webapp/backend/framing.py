"""
UART / TCP framing: COBS + CRC16-CCITT — mirror firmware uart_framing.c
"""
from __future__ import annotations

PROTOCOL_VERSION = 1
PROTOCOL_HEADER_LEN = 7  # version + msg_id + payload_len + crc (crc covers version..payload)
PROTOCOL_MAX_PAYLOAD = 110
UART_TX_MAX_LEN = 128


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(src: bytes) -> bytes:
    if not src:
        return b"\x00"
    out = bytearray()
    run_start = 0
    src_len = len(src)
    while run_start <= src_len:
        if run_start == src_len:
            out.append(0x00)
            return bytes(out)
        block_len = 0
        while (
            run_start + block_len < src_len
            and src[run_start + block_len] != 0
            and block_len < 254
        ):
            block_len += 1
        out.append(block_len + 1)
        out.extend(src[run_start : run_start + block_len])
        run_start += block_len
        if run_start < src_len and src[run_start] == 0:
            run_start += 1
    out.append(0x00)
    return bytes(out)


def cobs_decode(src: bytes) -> bytes:
    """Decode COBS block without trailing 0x00 delimiter (firmware feeds bytes between delimiters)."""
    out = bytearray()
    i = 0
    n = len(src)
    while i < n:
        code = src[i]
        i += 1
        if code == 0:
            break
        if code > 1:
            copy = code - 1
            if i + copy > n:
                return b""
            out.extend(src[i : i + copy])
            i += copy
        if code != 255:
            if code == 1:
                out.append(0)
            elif i < n and src[i] != 0:
                out.append(0)
    return bytes(out)


def build_framed_message(msg_id: int, payload: bytes) -> bytes:
    if len(payload) > PROTOCOL_MAX_PAYLOAD:
        raise ValueError("payload exceeds PROTOCOL_MAX_PAYLOAD")
    raw = bytearray()
    raw.append(PROTOCOL_VERSION)
    raw.append(msg_id & 0xFF)
    raw.append((msg_id >> 8) & 0xFF)
    raw.append(len(payload) & 0xFF)
    raw.append((len(payload) >> 8) & 0xFF)
    raw.extend(payload)
    crc = crc16_ccitt(raw)
    raw.append(crc & 0xFF)
    raw.append((crc >> 8) & 0xFF)
    framed = cobs_encode(bytes(raw))
    return framed


class FrameDecoder:
    """Stream decoder: buffer bytes until COBS frames (0x00) complete."""

    def __init__(self) -> None:
        self._cobs_buf = bytearray()
        self._raw_cap = PROTOCOL_HEADER_LEN + PROTOCOL_MAX_PAYLOAD

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []
        for b in data:
            if b == 0:
                if self._cobs_buf:
                    raw = cobs_decode(bytes(self._cobs_buf))
                    self._cobs_buf.clear()
                    if len(raw) >= PROTOCOL_HEADER_LEN:
                        payload_len = raw[3] | (raw[4] << 8)
                        if payload_len <= PROTOCOL_MAX_PAYLOAD and len(raw) == 7 + payload_len:
                            crc = raw[5 + payload_len] | (raw[6 + payload_len] << 8)
                            if crc == crc16_ccitt(raw[: 5 + payload_len]):
                                msg_id = raw[1] | (raw[2] << 8)
                                payload = bytes(raw[5 : 5 + payload_len])
                                frames.append((msg_id, payload))
                continue
            if len(self._cobs_buf) < UART_TX_MAX_LEN:
                self._cobs_buf.append(b)
            else:
                self._cobs_buf.clear()
        return frames

    def reset(self) -> None:
        self._cobs_buf.clear()
