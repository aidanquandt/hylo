#!/usr/bin/env python3
"""
Protocol host tool: send framed Request messages and listen for Response/log events.
Transport-agnostic protocol (e.g. over UART, USB, etc.).

Uses generated protocol_pb2.py for encoding requests and decoding responses.
Run tools/protocol_codegen/verify_codegen.py (and ensure nanopb_pb2.py is in generated/protocol/python) before use.

Usage:
  python protocol_tool.py --port COM10 interactive   # listen + type commands anytime
  python protocol_tool.py --port COM10 send ping
  python protocol_tool.py --port COM10 listen
  python protocol_tool.py --port COM10 send setaddress <address> <pan_id>
  python protocol_tool.py --port COM10 commands   # list all commands

See: python protocol_tool.py send --help
"""
from __future__ import print_function

import argparse
import os
import queue
import sys
import threading
import time

# Add generated/protocol/python to path for protocol_ids and protocol_pb2
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "generated", "protocol", "python"))

import protocol_ids  # noqa: E402

try:
    import protocol_pb2 as pb2  # noqa: E402
    from google.protobuf import descriptor as _descriptor  # noqa: E402
except ImportError as e:
    print(
        "protocol_pb2 not available (missing or missing nanopb_pb2). "
        "Run: python tools/protocol_codegen/verify_codegen.py\n"
        "If verify_codegen passes, generate nanopb_pb2.py into generated/protocol/python, e.g.:\n"
        "  protoc --python_out=generated/protocol/python -I third_party/nanopb/generator/proto "
        "third_party/nanopb/generator/proto/nanopb.proto",
        file=sys.stderr,
    )
    raise

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

# NodeType enum for CLI (tag=0, anchor=1, hybrid=2)
NODE_TYPE_NAMES = {"tag": 0, "anchor": 1, "hybrid": 2}


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


def get_message_class(name):
    """Resolve proto message class by name (e.g. 'PingRequest', 'AckResponse')."""
    cls = getattr(pb2, name, None)
    if cls is None and name.endswith("Request") and hasattr(pb2, "ErrorClearRequest"):
        # Some names might differ; try without suffix for enums
        pass
    return cls


def _get_response_type_for_request(request_message_name):
    """Infer expected response: XxxResponse if exists, else AckResponse."""
    if not request_message_name.endswith("Request"):
        return "AckResponse"
    base = request_message_name[:-7]  # strip "Request"
    response_name = base + "Response"
    if pb2.DESCRIPTOR.message_types_by_name.get(response_name) is not None:
        return response_name
    if response_name in protocol_ids.MSG_NAMES:
        return response_name
    return "AckResponse"


def _field_descriptor_to_arg_spec(fd):
    """Convert a FieldDescriptor to UI arg spec: {name, type, choices?}."""
    out = {"name": fd.name, "type": "str"}
    if fd.type == _descriptor.FieldDescriptor.TYPE_ENUM:
        out["type"] = "choice"
        if fd.enum_type.full_name.endswith("NodeType"):
            out["choices"] = list(NODE_TYPE_NAMES.keys())
        else:
            out["choices"] = [v.name.split("_")[-1].lower() for v in fd.enum_type.values]
        return out
    if fd.type in (_descriptor.FieldDescriptor.TYPE_INT32, _descriptor.FieldDescriptor.TYPE_INT64,
                   _descriptor.FieldDescriptor.TYPE_UINT32, _descriptor.FieldDescriptor.TYPE_UINT64,
                   _descriptor.FieldDescriptor.TYPE_FIXED32, _descriptor.FieldDescriptor.TYPE_FIXED64,
                   _descriptor.FieldDescriptor.TYPE_SFIXED32, _descriptor.FieldDescriptor.TYPE_SFIXED64):
        out["type"] = "int"
        return out
    if fd.type in (_descriptor.FieldDescriptor.TYPE_FLOAT, _descriptor.FieldDescriptor.TYPE_DOUBLE):
        out["type"] = "float"
        return out
    if fd.type == _descriptor.FieldDescriptor.TYPE_BOOL:
        out["type"] = "bool"
        return out
    if fd.type == _descriptor.FieldDescriptor.TYPE_BYTES:
        out["type"] = "str"  # hex input for bytes
        return out
    if fd.label == _descriptor.FieldDescriptor.LABEL_REPEATED and fd.name == "addresses":
        out["type"] = "str"  # comma-separated
        return out
    out["type"] = "str"
    return out


def get_commands():
    """Build command list from proto descriptor: all *Request messages with inferred response and args."""
    msg_names = pb2.DESCRIPTOR.message_types_by_name
    commands = []
    for name in sorted(msg_names.keys()):
        if not name.endswith("Request"):
            continue
        desc = msg_names[name]
        response_type = _get_response_type_for_request(name)
        args = []
        for fd in desc.fields:
            args.append(_field_descriptor_to_arg_spec(fd))
        commands.append({
            "command": name,
            "response_type": response_type,
            "args": args,
        })
    return commands


def build_request(message_name, args_list):
    """Build a proto Request message from message type name and list of string args (in field order)."""
    req_cls = get_message_class(message_name)
    if req_cls is None:
        raise ValueError("Unknown request type %s" % message_name)
    req = req_cls()
    desc = pb2.DESCRIPTOR.message_types_by_name.get(message_name)
    if desc is None:
        raise ValueError("No descriptor for %s" % message_name)
    for i, fd in enumerate(desc.fields):
        if i >= len(args_list):
            raise ValueError("Missing argument: %s" % fd.name)
        raw = args_list[i]
        if fd.label == _descriptor.FieldDescriptor.LABEL_REPEATED and fd.name == "addresses":
            req.addresses.extend([int(x, 0) for x in raw.split(",") if x.strip()])
            continue
        if fd.type == _descriptor.FieldDescriptor.TYPE_BYTES or fd.name == "config":
            req.config = bytes.fromhex(raw.replace(" ", ""))
            continue
        if fd.type == _descriptor.FieldDescriptor.TYPE_ENUM:
            raw_lower = raw.lower()
            if fd.enum_type.full_name.endswith("NodeType") and raw_lower in NODE_TYPE_NAMES:
                setattr(req, fd.name, NODE_TYPE_NAMES[raw_lower])
            else:
                setattr(req, fd.name, int(raw, 0))
            continue
        if fd.type in (_descriptor.FieldDescriptor.TYPE_INT32, _descriptor.FieldDescriptor.TYPE_INT64,
                      _descriptor.FieldDescriptor.TYPE_UINT32, _descriptor.FieldDescriptor.TYPE_UINT64,
                      _descriptor.FieldDescriptor.TYPE_FIXED32, _descriptor.FieldDescriptor.TYPE_FIXED64,
                      _descriptor.FieldDescriptor.TYPE_SFIXED32, _descriptor.FieldDescriptor.TYPE_SFIXED64):
            setattr(req, fd.name, int(raw, 0))
            continue
        if fd.type in (_descriptor.FieldDescriptor.TYPE_FLOAT, _descriptor.FieldDescriptor.TYPE_DOUBLE):
            setattr(req, fd.name, float(raw))
            continue
        if fd.type == _descriptor.FieldDescriptor.TYPE_BOOL:
            setattr(req, fd.name, raw.lower() in ("1", "true", "yes"))
            continue
        setattr(req, fd.name, raw)
    return req


def get_msg_id_for_type(type_name):
    """Return protocol msg_id for a message type name (e.g. PingRequest)."""
    for i, name in enumerate(protocol_ids.MSG_NAMES):
        if name == type_name:
            return i
    return None


def format_response(msg_id, payload):
    """Decode payload with pb2 and return a human-readable string. Uses LOG_EVENT_FORMAT_TABLE for event types."""
    if msg_id >= len(protocol_ids.MSG_NAMES):
        return "Unknown msg_id=%d (%d bytes)" % (msg_id, len(payload))
    name = protocol_ids.MSG_NAMES[msg_id]
    cls = get_message_class(name)
    if cls is None:
        return "%s (%d bytes)" % (name, len(payload))
    try:
        msg = cls()
        msg.ParseFromString(payload)
    except Exception:
        return "%s (parse error, %d bytes)" % (name, len(payload))
    if name in LOG_EVENT_FORMAT_TABLE:
        return format_log_event(name, msg)
    return format_message(name, msg)


def format_message(name, msg):
    """Optional format table: human-readable one-liner for a decoded message."""
    parts = []
    for fd in msg.DESCRIPTOR.fields:
        try:
            if fd.type == fd.TYPE_MESSAGE and msg.HasField(fd.name):
                parts.append("%s={...}" % fd.name)
            elif fd.type != fd.TYPE_MESSAGE:
                val = getattr(msg, fd.name)
                if fd.type == fd.TYPE_ENUM:
                    parts.append("%s=%s" % (fd.name, val))
                else:
                    parts.append("%s=%s" % (fd.name, val))
        except Exception:
            pass
    if not parts:
        return name
    return "%s %s" % (name, " ".join(parts))


# Optional format table for log events: message type name -> format string or callable(msg) -> str
# Format string can use %(field_name)s. Callable allows custom formatting.
LOG_EVENT_FORMAT_TABLE = {
    "ImuInitReturnedNullEvent": "IMU %(imu_index)u: init returned NULL",
    "ImuInvalidChipIdEvent": "IMU %(imu_index)u: invalid chip ID 0x%(chip_id)02X",
    "ImuFaultEvent": "IMU %(imu_index)u fault: code=%(fault_code)u %(description)s",
    "OtaConfigUnknownMessageTypeEvent": "OTA unknown message type: 0x%(message_type)02X",
    "UwbFrameTooSmallEvent": "UWB frame too small: %(length)u bytes",
    "UwbNodeInvalidTypeEvent": "Invalid UWB node type: %(node_type)d",
    "ResponderFailedToSendMessageEvent": "Responder failed to send message type %(message_type)d",
    "TwrSchedulerTargetListFullEvent": "TWR target list full",
    "SystemFatalEvent": "FATAL [%(module)s] %(message)s",
}


def format_log_event(name, msg):
    """Format a log event message for display; fallback to format_message."""
    if name in LOG_EVENT_FORMAT_TABLE:
        fmt = LOG_EVENT_FORMAT_TABLE[name]
        if callable(fmt):
            return fmt(msg)
        try:
            kwargs = {}
            for fd in msg.DESCRIPTOR.fields:
                if fd.type != fd.TYPE_MESSAGE:
                    kwargs[fd.name] = getattr(msg, fd.name)
                elif msg.HasField(fd.name):
                    kwargs[fd.name] = getattr(msg, fd.name)
            return fmt % kwargs
        except (KeyError, TypeError):
            pass
    return format_message(name, msg)


def send_command(ser, message_name, args_list, timeout_s=5.0, event_list=None):
    """Send a request and wait for the expected response. Returns True on success.
    If event_list is provided, any interleaved messages (e.g. log events) are appended as formatted strings instead of printed to stderr."""
    req = build_request(message_name, args_list)
    resp_type = _get_response_type_for_request(message_name)
    msg_id_req = get_msg_id_for_type(message_name)
    if msg_id_req is None:
        raise ValueError("No msg_id for %s" % message_name)
    payload = req.SerializeToString()
    if len(payload) > PROTOCOL_MAX_PAYLOAD:
        raise ValueError("Payload too large: %d > %d" % (len(payload), PROTOCOL_MAX_PAYLOAD))
    frame = build_frame(msg_id_req, payload)
    ser.write(frame)
    expected_ids = set()
    resp_name = resp_type
    resp_cls = get_message_class(resp_name)
    if resp_cls is None:
        raise ValueError("No class for response %s" % resp_name)
    for i, name in enumerate(protocol_ids.MSG_NAMES):
        if name == resp_name:
            expected_ids.add(i)
            break
    if not expected_ids:
        raise ValueError("No msg_id for response %s" % resp_name)
    deadline = time.time() + timeout_s
    buf = bytearray()
    while time.time() < deadline:
        if ser.in_waiting > 0:
            buf.extend(ser.read(ser.in_waiting))
        while 0x00 in buf:
            idx = buf.index(0x00)
            block = bytes(buf[: idx + 1])
            del buf[: idx + 1]
            result = parse_frame(block)
            if result:
                msg_id, payload = result
                if msg_id in expected_ids:
                    resp = resp_cls()
                    resp.ParseFromString(payload)
                    print(format_message(resp_name, resp))
                    return True
                # Log unexpected response (e.g. log event interleaved)
                line = "[rx] %s" % format_response(msg_id, payload)
                if event_list is not None:
                    event_list.append(line)
                else:
                    print(line, file=sys.stderr)
        time.sleep(0.01)
    print("Timeout waiting for %s" % resp_name, file=sys.stderr)
    return False


def _serial_reader_thread(ser, out_queue, stop_event):
    """Background thread: read serial, parse frames, put (msg_id, payload) in out_queue."""
    buf = bytearray()
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                buf.extend(ser.read(ser.in_waiting))
            while 0x00 in buf:
                idx = buf.index(0x00)
                block = bytes(buf[: idx + 1])
                del buf[: idx + 1]
                result = parse_frame(block)
                if result:
                    out_queue.put(result)
            time.sleep(0.01)
        except Exception:
            if not stop_event.is_set():
                out_queue.put(None)  # signal error
            break


def _format_queued_message(msg_id, payload, use_format_table=True):
    """Decode (msg_id, payload) and return a string for printing."""
    names = protocol_ids.MSG_NAMES
    name = names[msg_id] if msg_id < len(names) else "Unknown"
    cls = get_message_class(name)
    if cls is None:
        return "[rx] %s (%d bytes)" % (name, len(payload))
    try:
        msg = cls()
        msg.ParseFromString(payload)
        if use_format_table and name in LOG_EVENT_FORMAT_TABLE:
            line = format_log_event(name, msg)
        else:
            line = format_message(name, msg)
        return "[rx] %s" % line
    except Exception:
        return "[rx] %s (%d bytes)" % (name, len(payload))


def send_command_interactive(ser, message_name, args_list, serial_queue, timeout_s=5.0):
    """Send a request and wait for the expected response by reading from serial_queue. Prints any other messages (e.g. log events) as they arrive. Returns True on success."""
    req = build_request(message_name, args_list)
    resp_type = _get_response_type_for_request(message_name)
    msg_id_req = get_msg_id_for_type(message_name)
    if msg_id_req is None:
        print("No msg_id for %s" % message_name, file=sys.stderr)
        return False
    payload = req.SerializeToString()
    if len(payload) > PROTOCOL_MAX_PAYLOAD:
        print("Payload too large", file=sys.stderr)
        return False
    frame = build_frame(msg_id_req, payload)
    ser.write(frame)
    resp_cls = get_message_class(resp_type)
    if resp_cls is None:
        print("No class for response %s" % resp_type, file=sys.stderr)
        return False
    expected_ids = set()
    for i, name in enumerate(protocol_ids.MSG_NAMES):
        if name == resp_type:
            expected_ids.add(i)
            break
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            item = serial_queue.get(timeout=0.25)
        except queue.Empty:
            continue
        if item is None:
            continue
        msg_id, payload = item
        line = _format_queued_message(msg_id, payload)
        if msg_id in expected_ids:
            print(line)
            return True
        print(line)
    print("Timeout waiting for %s" % resp_type, file=sys.stderr)
    return False


def interactive(ser, use_format_table=True):
    """Interactive mode: background thread listens and enqueues frames; main loop prints them and accepts commands from stdin. Type 'help' or 'commands', or 'quit'/'exit' to leave."""
    serial_queue = queue.Queue()
    command_queue = queue.Queue()
    stop_event = threading.Event()

    def input_thread_fn():
        while not stop_event.is_set():
            try:
                line = input("> ")
                command_queue.put((line,))
            except EOFError:
                command_queue.put(("quit",))
                break
            except Exception:
                break

    reader = threading.Thread(target=_serial_reader_thread, args=(ser, serial_queue, stop_event), daemon=True)
    reader.start()
    input_thread = threading.Thread(target=input_thread_fn, daemon=True)
    input_thread.start()

    print("Interactive mode. Type a command (e.g. PingRequest, SetAddressRequest) or: help | commands | quit")
    commands = get_commands()
    cmd_names = [c["command"] for c in commands]
    try:
        while True:
            # Drain serial queue and print (non-blocking)
            while True:
                try:
                    item = serial_queue.get_nowait()
                except queue.Empty:
                    break
                if item is None:
                    continue
                msg_id, payload = item
                print(_format_queued_message(msg_id, payload, use_format_table))
            # Check for user command (short timeout so we keep draining serial)
            try:
                line_tuple = command_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            line = (line_tuple[0] or "").strip()
            if not line:
                continue
            if line.lower() in ("quit", "exit", "q"):
                print("Bye.")
                break
            if line.lower() in ("help", "commands"):
                print_commands()
                continue
            parts = line.split()
            message_name = parts[0]
            args_list = parts[1:] if len(parts) > 1 else []
            if message_name not in cmd_names:
                print("Unknown command: %s (type 'commands' for list)" % message_name, file=sys.stderr)
                continue
            cmd_spec = next((c for c in commands if c["command"] == message_name), None)
            if cmd_spec is None:
                continue
            arg_count = len(cmd_spec["args"])
            if len(args_list) < arg_count:
                print("Command %s requires %d argument(s): %s" % (message_name, arg_count, [a["name"] for a in cmd_spec["args"]]), file=sys.stderr)
                continue
            try:
                send_command_interactive(ser, message_name, args_list, serial_queue, timeout_s=5.0)
            except ValueError as e:
                print(str(e), file=sys.stderr)
    finally:
        stop_event.set()


def listen(ser, use_format_table=True):
    """Listen and print received frames; decode with pb2 and optional format table for log events."""
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
                    cls = get_message_class(name)
                    if cls is not None:
                        try:
                            msg = cls()
                            msg.ParseFromString(payload)
                            if use_format_table and name in LOG_EVENT_FORMAT_TABLE:
                                line = format_log_event(name, msg)
                            else:
                                line = format_message(name, msg)
                            print("[rx] %s" % line)
                        except Exception:
                            print("[rx] %s (%d bytes)" % (name, len(payload)))
                    else:
                        print("[rx] %s (%d bytes)" % (name, len(payload)))
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped.")


def print_commands():
    """Print command set from descriptor (help)."""
    print("Commands (use: send <MessageName> [args...]):")
    print()
    for cmd in get_commands():
        name = cmd["command"]
        args_str = " ".join("<%s>" % a["name"] for a in cmd["args"])
        if args_str:
            print("  %s %s" % (name, args_str))
        else:
            print("  %s" % name)
    print()
    print("Examples:")
    print("  send PingRequest")
    print("  send GetConfigRequest")
    print("  send SetAddressRequest 0x1234 0x5678")
    print("  send UwbNodeSetTypeRequest tag")
    print("  send TwrMgrSetTargetsRequest 1,2,3")
    print("  send OtaConfigSendAddressRequest 0 0x200 0x1234")


def main():
    ap = argparse.ArgumentParser(
        description="Protocol tool: send Request messages, listen for Response/log events (COBS+CRC16)."
    )
    ap.add_argument("--port", required=True, help="Serial port (e.g. COM10, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("interactive", help="Listen for messages and type commands in the same terminal (recommended)")
    sub.add_parser("listen", help="Listen and print received frames (responses and log events)")
    sub.add_parser("commands", help="List all send commands and their arguments")
    send_parser = sub.add_parser("send", help="Send a message (request); wait for response")
    command_choices = [c["command"] for c in get_commands()]
    send_parser.add_argument(
        "msg",
        choices=command_choices,
        metavar="MessageName",
        help="Request message name (e.g. PingRequest, SetAddressRequest)",
    )
    send_parser.add_argument("args", nargs="*", help="Arguments for the command (see: commands)")
    args = ap.parse_args()

    if args.cmd == "commands":
        print_commands()
        return

    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    try:
        if args.cmd == "interactive":
            interactive(ser)
        elif args.cmd == "listen":
            listen(ser)
        elif args.cmd == "send":
            cmd_spec = next((c for c in get_commands() if c["command"] == args.msg), None)
            if cmd_spec is None:
                print("Unknown command: %s" % args.msg, file=sys.stderr)
                sys.exit(1)
            arg_count = len(cmd_spec["args"])
            if len(args.args) < arg_count:
                print("Command %s requires %d argument(s): %s" % (args.msg, arg_count, [a["name"] for a in cmd_spec["args"]]), file=sys.stderr)
                print("Run: protocol_tool.py --port <PORT> commands", file=sys.stderr)
                sys.exit(1)
            try:
                ok = send_command(ser, args.msg, args.args)
                sys.exit(0 if ok else 1)
            except ValueError as e:
                print(str(e), file=sys.stderr)
                sys.exit(1)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
