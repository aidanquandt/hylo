"""
Unit tests for UART protocol framing: COBS encode/decode, CRC16, frame build/parse.
Run from repo root: python -m pytest tools/host/serial/tests/test_uart_framing.py -v
Or: python tools/host/serial/tests/test_uart_framing.py
"""
from __future__ import print_function

import os
import sys

# Add tool and generated/protocol to path
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "generated", "protocol"))
sys.path.insert(0, os.path.join(ROOT, "tools", "host", "serial"))

import protocol_ids  # noqa: E402
from uart_protocol_tool import (  # noqa: E402
    build_frame,
    cobs_decode,
    cobs_encode,
    crc16_ccitt,
    decode_get_config_response,
    decode_ping_response,
    encode_get_config_request,
    encode_get_config_response,
    encode_ping_request,
    encode_ping_response,
    encode_set_address_request,
    encode_set_address_response,
    parse_frame,
)


def test_crc16_ccitt():
    assert crc16_ccitt(b"") == 0xFFFF
    assert crc16_ccitt(b"\x00") != 0xFFFF
    # Deterministic: same input -> same output
    assert crc16_ccitt(b"hello") == crc16_ccitt(b"hello")


def test_cobs_encode_decode_roundtrip():
    for orig in [b"", b"x", b"hello", b"\x00", b"a\x00b", b"\x00\x00", b"x" * 253]:
        encoded = cobs_encode(orig)
        assert 0x00 not in encoded[:-1], "COBS encoded block must not contain 0x00 except at end"
        assert encoded[-1] == 0x00
        decoded = cobs_decode(encoded)
        assert decoded == orig, (orig, encoded, decoded)


def test_cobs_encode_no_zero_in_body():
    encoded = cobs_encode(b"hello world")
    assert all(b != 0 for b in encoded[:-1])
    assert encoded[-1] == 0


def test_build_and_parse_frame():
    msg_id = protocol_ids.MSG_ID_PingRequest
    payload = encode_ping_request()
    frame = build_frame(msg_id, payload)
    assert isinstance(frame, bytes)
    assert 0x00 in frame
    result = parse_frame(frame)
    assert result is not None
    mid, p = result
    assert mid == msg_id
    assert p == payload


def test_ping_roundtrip():
    payload = encode_ping_request()
    frame = build_frame(protocol_ids.MSG_ID_PingRequest, payload)
    result = parse_frame(frame)
    assert result is not None
    msg_id, p = result
    assert msg_id == protocol_ids.MSG_ID_PingRequest
    assert p == b""


def test_ping_response_encode_decode():
    for seq in [0, 1, 255, 256]:
        enc = encode_ping_response(seq)
        dec = decode_ping_response(enc)
        assert dec["seq"] == seq


def test_get_config_response_encode_decode():
    enc = encode_get_config_response(0x1234, 0x5678)
    dec = decode_get_config_response(enc)
    assert dec["pan_id"] == 0x1234
    assert dec["short_addr"] == 0x5678


def test_parse_frame_invalid_crc():
    msg_id = protocol_ids.MSG_ID_PingRequest
    payload = b""
    frame = build_frame(msg_id, payload)
    # Corrupt last two bytes (CRC)
    bad = bytearray(frame)
    bad[-1] ^= 0xFF
    result = parse_frame(bytes(bad))
    assert result is None


def test_parse_frame_truncated():
    result = parse_frame(b"\x02\x01\x00")  # valid COBS but too short for header
    assert result is None


if __name__ == "__main__":
    # Run without pytest
    test_crc16_ccitt()
    test_cobs_encode_decode_roundtrip()
    test_cobs_encode_no_zero_in_body()
    test_build_and_parse_frame()
    test_ping_roundtrip()
    test_ping_response_encode_decode()
    test_get_config_response_encode_decode()
    test_parse_frame_invalid_crc()
    test_parse_frame_truncated()
    print("All tests passed.")
