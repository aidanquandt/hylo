#!/usr/bin/env python3
"""
UART protocol host tool: send framed messages and/or listen for responses.
Usage:
  python uart_protocol_tool.py --port COM10 send ping
  python uart_protocol_tool.py --port COM10 listen
  python uart_protocol_tool.py --port COM10 send getconfig
  python uart_protocol_tool.py --port COM10 send setaddress <address> <pan_id>
"""
from __future__ import print_function

import argparse
import os
import struct
import sys

# Add protocol/generated to path for protocol_ids
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "protocol", "generated"))
import protocol_ids  # noqa: E402

try:
    import serial
except ImportError:
    print(
        "pyserial not found. Install with your package manager (e.g. msys2: "
        "pacman -S mingw-w64-ucrt-x86_64-python-pyserial) or pip install pyserial",
        file=sys.stderr,
    )
    sys.exit(1)

PROTOCOL_VERSION = 1
PROTOCOL_HEADER_LEN = 7
PROTOCOL_MAX_PAYLOAD = 110


def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_varint(value):
    buf = []
    while value > 0x7F:
        buf.append((value & 0x7F) | 0x80)
        value >>= 7
    buf.append(value & 0x7F)
    return bytes(buf)


def decode_varint(buf, offset):
    result = 0
    shift = 0
    while offset < len(buf):
        b = buf[offset]
        offset += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, offset
        shift += 7
        if shift >= 35:
            raise ValueError("varint too long")
    raise ValueError("varint truncated")


def cobs_encode(src):
    dst = []
    run_start = 0
    while run_start <= len(src):
        if run_start == len(src):
            dst.append(0x00)
            return bytes(dst)
        block_len = 0
        while run_start + block_len < len(src) and src[run_start + block_len] != 0 and block_len < 254:
            block_len += 1
        dst.append(block_len + 1)
        dst.extend(src[run_start : run_start + block_len])
        run_start += block_len
        if run_start < len(src) and src[run_start] == 0:
            run_start += 1
    dst.append(0x00)
    return bytes(dst)


def cobs_decode(src):
    dst = []
    i = 0
    while i < len(src):
        code = src[i]
        i += 1
        if code == 0:
            return bytes(dst)
        if code > 1:
            copy = code - 1
            if i + copy > len(src):
                return None
            dst.extend(src[i : i + copy])
            i += copy
        # Stuffed zero: after each block output one 0, except after last block (next byte is delimiter)
        if code == 1:
            dst.append(0)
        elif code != 255 and i < len(src) and src[i] != 0:
            dst.append(0)
    return None


def build_frame(msg_id, payload):
    raw = bytearray(PROTOCOL_HEADER_LEN + len(payload))
    raw[0] = PROTOCOL_VERSION
    raw[1] = msg_id & 0xFF
    raw[2] = (msg_id >> 8) & 0xFF
    raw[3] = len(payload) & 0xFF
    raw[4] = (len(payload) >> 8) & 0xFF
    raw[5 : 5 + len(payload)] = payload
    crc = crc16_ccitt(bytes(raw[: 5 + len(payload)]))
    raw[5 + len(payload)] = crc & 0xFF
    raw[6 + len(payload)] = (crc >> 8) & 0xFF
    return cobs_encode(bytes(raw[: 7 + len(payload)]))


def parse_frame(cobs_block):
    raw = cobs_decode(cobs_block)
    if raw is None or len(raw) < PROTOCOL_HEADER_LEN:
        return None
    payload_len = raw[3] | (raw[4] << 8)
    if payload_len > PROTOCOL_MAX_PAYLOAD or len(raw) != 7 + payload_len:
        return None
    crc = raw[5 + payload_len] | (raw[6 + payload_len] << 8)
    if crc != crc16_ccitt(raw[: 5 + payload_len]):
        return None
    msg_id = raw[1] | (raw[2] << 8)
    payload = bytes(raw[5 : 5 + payload_len])
    return msg_id, payload


# Minimal protobuf encode for our message types (wire-compatible with nanopb)
def encode_ping_request():
    return b""


def encode_ping_response(seq):
    return b"\x08" + encode_varint(seq)


def decode_ping_response(payload):
    if not payload or payload[0] != 0x08:
        return {"seq": 0}
    seq, _ = decode_varint(payload, 1)
    return {"seq": seq}


def encode_get_config_request():
    return b""


def encode_get_config_response(pan_id, short_addr):
    return b"\x08" + encode_varint(pan_id) + b"\x10" + encode_varint(short_addr)


def decode_get_config_response(payload):
    d = {"pan_id": 0, "short_addr": 0}
    off = 0
    while off < len(payload):
        if off >= len(payload):
            break
        tag = payload[off]
        off += 1
        if tag == 0x08:
            d["pan_id"], off = decode_varint(payload, off)
        elif tag == 0x10:
            d["short_addr"], off = decode_varint(payload, off)
        else:
            break
    return d


def encode_set_address_request(address, pan_id):
    return b"\x08" + encode_varint(address) + b"\x10" + encode_varint(pan_id)


def encode_set_address_response(success):
    return b"\x08" + (1 if success else 0)


def send_ping(ser):
    frame = build_frame(protocol_ids.MSG_ID_PingRequest, encode_ping_request())
    ser.write(frame)
    print("Sent PingRequest, waiting for PingResponse...")
    buf = bytearray()
    while True:
        if ser.in_waiting > 0:
            buf.extend(ser.read(ser.in_waiting))
            while 0x00 in buf:
                idx = buf.index(0x00)
                block = bytes(buf[: idx + 1])
                del buf[: idx + 1]
                result = parse_frame(block)
                if result:
                    msg_id, payload = result
                    if msg_id == protocol_ids.MSG_ID_PingResponse:
                        r = decode_ping_response(payload)
                        print("PingResponse seq=%u" % r["seq"])
                        return
        else:
            import time
            time.sleep(0.01)


def send_getconfig(ser):
    frame = build_frame(protocol_ids.MSG_ID_GetConfigRequest, encode_get_config_request())
    ser.write(frame)
    print("Sent GetConfigRequest, waiting for GetConfigResponse...")
    buf = bytearray()
    while True:
        if ser.in_waiting > 0:
            buf.extend(ser.read(ser.in_waiting))
            while 0x00 in buf:
                idx = buf.index(0x00)
                block = bytes(buf[: idx + 1])
                del buf[: idx + 1]
                result = parse_frame(block)
                if result:
                    msg_id, payload = result
                    if msg_id == protocol_ids.MSG_ID_GetConfigResponse:
                        r = decode_get_config_response(payload)
                        print("GetConfigResponse pan_id=0x%04x short_addr=0x%04x" % (r["pan_id"], r["short_addr"]))
                        return
        else:
            import time
            time.sleep(0.01)


def send_setaddress(ser, address, pan_id):
    payload = encode_set_address_request(address, pan_id)
    frame = build_frame(protocol_ids.MSG_ID_SetAddressRequest, payload)
    ser.write(frame)
    print("Sent SetAddressRequest %u %u, waiting for SetAddressResponse..." % (address, pan_id))
    buf = bytearray()
    while True:
        if ser.in_waiting > 0:
            buf.extend(ser.read(ser.in_waiting))
            while 0x00 in buf:
                idx = buf.index(0x00)
                block = bytes(buf[: idx + 1])
                del buf[: idx + 1]
                result = parse_frame(block)
                if result:
                    msg_id, payload = result
                    if msg_id == protocol_ids.MSG_ID_SetAddressResponse:
                        success = payload and payload[0] == 1
                        print("SetAddressResponse success=%s" % success)
                        return
        else:
            import time
            time.sleep(0.01)


def listen(ser):
    print("Listening (Ctrl+C to stop)...")
    buf = bytearray()
    names = protocol_ids.MSG_NAMES
    try:
        while True:
            if ser.in_waiting > 0:
                buf.extend(ser.read(ser.in_waiting))
                while 0x00 in buf:
                    idx = buf.index(0x00)
                    block = bytes(buf[: idx + 1])
                    del buf[: idx + 1]
                    result = parse_frame(block)
                    if result:
                        msg_id, payload = result
                        name = names[msg_id] if msg_id < len(names) else "Unknown"
                        print("RX %s (%u bytes)" % (name, len(payload)))
                        if msg_id == protocol_ids.MSG_ID_LogLine and len(payload) >= 1:
                            level, off = decode_varint(payload, 0)
                            text = payload[off:].decode("utf-8", errors="replace") if off < len(payload) else ""
                            print("  LogLine level=%u text=%r" % (level, text))
            else:
                import time
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped.")


def main():
    ap = argparse.ArgumentParser(description="UART protocol tool (COBS+CRC16)")
    ap.add_argument("--port", required=True, help="Serial port (e.g. COM10, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("listen", help="Listen and print received frames")
    send_parser = sub.add_parser("send", help="Send a message")
    send_parser.add_argument("msg", choices=["ping", "getconfig", "setaddress"], help="Message to send")
    send_parser.add_argument("args", nargs="*", help="Extra args (e.g. address pan_id for setaddress)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    try:
        if args.cmd == "listen":
            listen(ser)
        elif args.cmd == "send":
            if args.msg == "ping":
                send_ping(ser)
            elif args.msg == "getconfig":
                send_getconfig(ser)
            elif args.msg == "setaddress":
                if len(args.args) < 2:
                    print("setaddress requires <address> <pan_id>", file=sys.stderr)
                    sys.exit(1)
                send_setaddress(ser, int(args.args[0], 0), int(args.args[1], 0))
    finally:
        ser.close()


if __name__ == "__main__":
    main()
