#!/usr/bin/env python3
"""
UART protocol host tool: send framed Request messages and listen for Response/log events.

Uses generated uart_protocol_pb2.py for encoding requests and decoding responses.
Run protocol/verify_codegen.py (and ensure nanopb_pb2.py is in generated/protocol) before use.

Usage:
  python uart_protocol_tool.py --port COM10 interactive   # listen + type commands anytime
  python uart_protocol_tool.py --port COM10 send ping
  python uart_protocol_tool.py --port COM10 listen
  python uart_protocol_tool.py --port COM10 send setaddress <address> <pan_id>
  python uart_protocol_tool.py --port COM10 commands   # list all commands

See: python uart_protocol_tool.py send --help
"""
from __future__ import print_function

import argparse
import os
import queue
import sys
import threading
import time

# Add generated/protocol to path for protocol_ids and uart_protocol_pb2
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "generated", "protocol"))

import protocol_ids  # noqa: E402

try:
    import uart_protocol_pb2 as pb2  # noqa: E402
except ImportError as e:
    print(
        "uart_protocol_pb2 not available (missing or missing nanopb_pb2). "
        "Run: python protocol/verify_codegen.py\n"
        "If verify_codegen passes, generate nanopb_pb2.py into generated/protocol, e.g.:\n"
        "  protoc --python_out=generated/protocol -I third_party/nanopb/generator/proto "
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


# ---------------------------------------------------------------------------
# CLI command definitions: (cli_name, request_type, response_type, arg_spec, description)
# response_type is the expected response message name, or "AckResponse" for fire-and-forget.
# arg_spec: list of (name, type_or_choices) e.g. ("address", int), ("pan_id", int), ("node_type", ["tag","anchor","hybrid"])
# ---------------------------------------------------------------------------
COMMANDS = [
    # System
    ("get-uuid", "SystemGetUuidRequest", "SystemGetUuidResponse", [], "Get device UUID words"),
    ("get-info", "SystemGetInfoRequest", "SystemGetInfoResponse", [], "Get device info string"),
    # Beacon
    ("ping", "PingRequest", "PingResponse", [], "Ping (sequence)"),
    ("beacon-ping", "BeaconPingRequest", "BeaconPingResponse", [], "Send UWB DATA beacon ping"),
    # UWB node config
    ("getconfig", "GetConfigRequest", "GetConfigResponse", [], "Get PAN ID and short address"),
    ("setaddress", "SetAddressRequest", "SetAddressResponse", [("address", int), ("pan_id", int)], "Set address and PAN ID"),
    ("uwb-node-get-type", "UwbNodeGetTypeRequest", "UwbNodeGetTypeResponse", [], "Get UWB node type"),
    ("uwb-node-set-type", "UwbNodeSetTypeRequest", "AckResponse", [("node_type", list(NODE_TYPE_NAMES.keys()))], "Set UWB node type (tag|anchor|hybrid)"),
    ("uwb-node-get-position", "UwbNodeGetPositionRequest", "UwbNodeGetPositionResponse", [], "Get UWB node position"),
    ("uwb-node-set-position", "UwbNodeSetPositionRequest", "AckResponse", [("x", float), ("y", float), ("z", float)], "Set UWB node position"),
    ("uwb-node-get-status", "UwbNodeGetStatusRequest", "UwbNodeGetStatusResponse", [], "Get UWB node status"),
    # UWB
    ("uwb-get-status", "UwbGetStatusRequest", "UwbGetStatusResponse", [], "Get UWB radio status"),
    ("uwb-get-stats", "UwbGetStatsRequest", "UwbGetStatsResponse", [], "Get UWB TX/RX stats"),
    ("uwb-start", "UwbStartRequest", "AckResponse", [], "Start UWB"),
    ("uwb-stop", "UwbStopRequest", "AckResponse", [], "Stop UWB"),
    ("uwb-reset-stats", "UwbResetStatsRequest", "AckResponse", [], "Reset UWB stats"),
    # Error
    ("error-clear", "ErrorClearRequest", "AckResponse", [], "Clear errors"),
    # IMU
    ("imu-get-status", "ImuGetStatusRequest", "ImuGetStatusResponse", [], "Get IMU status"),
    ("imu-get-data", "ImuGetDataRequest", "ImuGetDataResponse", [], "Get IMU data sample"),
    ("imu-stream-start", "ImuStreamStartRequest", "AckResponse", [("mode", int)], "Start IMU stream (mode 0=array 1=avg)"),
    ("imu-stream-stop", "ImuStreamStopRequest", "AckResponse", [], "Stop IMU stream"),
    # Datalogger
    ("datalogger-get-tasks", "DataloggerGetTasksRequest", "DataloggerGetTasksResponse", [], "Get datalogger task names"),
    ("datalogger-get-stats", "DataloggerGetStatsRequest", "DataloggerGetStatsResponse", [], "Get datalogger stats"),
    # TWR
    ("twr-get-status", "TwrGetStatusRequest", "TwrGetStatusResponse", [], "Get TWR status"),
    ("twr-get-result", "TwrGetResultRequest", "TwrGetResultResponse", [], "Get last TWR result"),
    ("twr-range", "TwrRangeRequest", "AckResponse", [("target_addr", int)], "Request range to target address"),
    # TWR manager
    ("twr-mgr-start", "TwrMgrStartRequest", "AckResponse", [], "Start TWR manager"),
    ("twr-mgr-stop", "TwrMgrStopRequest", "AckResponse", [], "Stop TWR manager"),
    ("twr-mgr-get-status", "TwrMgrGetStatusRequest", "TwrMgrGetStatusResponse", [], "Get TWR manager status"),
    ("twr-mgr-add-target", "TwrMgrAddTargetRequest", "AckResponse", [("address", int)], "Add TWR target"),
    ("twr-mgr-remove-target", "TwrMgrRemoveTargetRequest", "AckResponse", [("address", int)], "Remove TWR target"),
    ("twr-mgr-set-targets", "TwrMgrSetTargetsRequest", "AckResponse", [("addresses", str)], "Set TWR targets (comma-separated addresses)"),
    ("twr-mgr-clear-targets", "TwrMgrClearTargetsRequest", "AckResponse", [], "Clear TWR targets"),
    ("twr-mgr-set-ranging-rate", "TwrMgrSetRangingRateRequest", "AckResponse", [("rate_hz", int)], "Set TWR ranging rate (Hz)"),
    ("twr-mgr-get-ranging-rate", "TwrMgrGetRangingRateRequest", "TwrMgrGetRangingRateResponse", [], "Get TWR ranging rate"),
    # Stopwatch
    ("stopwatch-get", "StopwatchGetRequest", "StopwatchGetResponse", [], "Get stopwatch value"),
    ("stopwatch-start", "StopwatchStartRequest", "AckResponse", [], "Start stopwatch"),
    ("stopwatch-stop", "StopwatchStopRequest", "AckResponse", [], "Stop stopwatch"),
    # OTA config
    ("ota-send-address", "OtaConfigSendAddressRequest", "AckResponse", [("target_addr", int), ("new_address", int), ("pan_id", int)], "OTA set target address"),
    ("ota-send-position", "OtaConfigSendPositionRequest", "AckResponse", [("target_addr", int), ("x", float), ("y", float), ("z", float)], "OTA set target position"),
    ("ota-send-type", "OtaConfigSendTypeRequest", "AckResponse", [("target_addr", int), ("node_type", list(NODE_TYPE_NAMES.keys()))], "OTA set target type"),
    ("ota-send-gpio", "OtaConfigSendGpioRequest", "AckResponse", [("target_addr", int), ("pin", int), ("state", int)], "OTA set GPIO"),
    ("ota-set-token", "OtaConfigSetTokenRequest", "AckResponse", [("token", int)], "Set OTA auth token"),
    ("ota-get-token", "OtaConfigGetTokenRequest", "OtaConfigGetTokenResponse", [], "Get OTA auth token"),
    ("ota-get-stats", "OtaConfigGetStatsRequest", "OtaConfigGetStatsResponse", [], "Get OTA config stats"),
    # Sensor fusion
    ("sf-get-debug", "SensorFusionGetDebugRequest", "AckResponse", [], "Get sensor fusion debug (fire-and-forget ack)"),
    ("sf-set-debug", "SensorFusionSetDebugRequest", "AckResponse", [("debug_flags", int)], "Set sensor fusion debug flags"),
    ("sf-get-status", "SensorFusionGetStatusRequest", "SensorFusionGetStatusResponse", [], "Get sensor fusion status"),
    ("sf-set-active", "SensorFusionSetActiveRequest", "AckResponse", [("active", int)], "Set sensor fusion active (0/1)"),
    ("sf-get-imu-enabled", "SensorFusionGetImuEnabledRequest", "SensorFusionGetImuEnabledResponse", [], "Get IMU enabled in sensor fusion"),
    ("sf-set-imu-enabled", "SensorFusionSetImuEnabledRequest", "AckResponse", [("imu_enabled", int)], "Set IMU enabled (0/1)"),
    ("sf-set-noise", "SensorFusionSetNoiseRequest", "AckResponse", [("pos", float), ("vel", float), ("att", float)], "Set sensor fusion noise"),
    ("sf-get-noise", "SensorFusionGetNoiseRequest", "SensorFusionGetNoiseResponse", [], "Get sensor fusion noise"),
    ("sf-get-config", "SensorFusionGetConfigRequest", "AckResponse", [], "Get sensor fusion config (ack only)"),
    ("sf-set-config", "SensorFusionSetConfigRequest", "AckResponse", [("config_hex", str)], "Set sensor fusion config (hex string)"),
]


def build_request(cli_name, args_list):
    """Build a proto Request message from CLI command name and list of string args."""
    for cmd in COMMANDS:
        if cmd[0] != cli_name:
            continue
        _, req_type, _resp_type, arg_spec, _ = cmd
        req_cls = get_message_class(req_type)
        if req_cls is None:
            raise ValueError("Unknown request type %s" % req_type)
        req = req_cls()
        for i, (arg_name, type_or_choices) in enumerate(arg_spec):
            if i >= len(args_list):
                raise ValueError("Missing argument: %s" % arg_name)
            raw = args_list[i]
            if isinstance(type_or_choices, list):
                # enum-like: choices
                raw_lower = raw.lower()
                if raw_lower not in type_or_choices:
                    raise ValueError("%s must be one of: %s" % (arg_name, ", ".join(type_or_choices)))
                if arg_name == "node_type":
                    req.node_type = NODE_TYPE_NAMES[raw_lower]
                else:
                    setattr(req, arg_name, raw)
            elif type_or_choices == int:
                setattr(req, arg_name, int(raw, 0))
            elif type_or_choices == float:
                setattr(req, arg_name, float(raw))
            elif type_or_choices == str and arg_name == "addresses":
                # Comma-separated list of integers
                req.addresses.extend([int(x, 0) for x in raw.split(",") if x.strip()])
            elif type_or_choices == str and arg_name == "config_hex":
                req.config = bytes.fromhex(raw.replace(" ", ""))
            else:
                setattr(req, arg_name, raw)
        return req
    raise ValueError("Unknown command: %s" % cli_name)


def get_msg_id_for_type(type_name):
    """Return protocol msg_id for a message type name (e.g. PingRequest)."""
    for i, name in enumerate(protocol_ids.MSG_NAMES):
        if name == type_name:
            return i
    return None


def format_response(msg_id, payload):
    """Decode payload with pb2 and return a human-readable string."""
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


def send_command(ser, cli_name, args_list, timeout_s=5.0):
    """Send a request and wait for the expected response. Returns True on success."""
    req = build_request(cli_name, args_list)
    req_type = None
    for cmd in COMMANDS:
        if cmd[0] == cli_name:
            req_type = cmd[1]
            resp_type = cmd[2]
            break
    if req_type is None:
        raise ValueError("Unknown command: %s" % cli_name)
    msg_id_req = get_msg_id_for_type(req_type)
    if msg_id_req is None:
        raise ValueError("No msg_id for %s" % req_type)
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
                print("[rx] %s" % format_response(msg_id, payload), file=sys.stderr)
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


def send_command_interactive(ser, cli_name, args_list, serial_queue, timeout_s=5.0):
    """Send a request and wait for the expected response by reading from serial_queue. Prints any other messages (e.g. log events) as they arrive. Returns True on success."""
    req = build_request(cli_name, args_list)
    req_type = resp_type = None
    for cmd in COMMANDS:
        if cmd[0] == cli_name:
            req_type = cmd[1]
            resp_type = cmd[2]
            break
    if req_type is None:
        print("Unknown command: %s" % cli_name, file=sys.stderr)
        return False
    msg_id_req = get_msg_id_for_type(req_type)
    if msg_id_req is None:
        print("No msg_id for %s" % req_type, file=sys.stderr)
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

    print("Interactive mode. Type a command (e.g. ping, getconfig) or: help | commands | quit")
    cmd_names = [c[0] for c in COMMANDS]
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
            cmd = parts[0]
            args_list = parts[1:] if len(parts) > 1 else []
            if cmd not in cmd_names:
                print("Unknown command: %s (type 'commands' for list)" % cmd, file=sys.stderr)
                continue
            cmd_spec = None
            for c in COMMANDS:
                if c[0] == cmd:
                    cmd_spec = c
                    break
            if cmd_spec is None:
                continue
            _cli_name, _req, _resp, arg_spec, _ = cmd_spec
            if len(args_list) < len(arg_spec):
                print("Command %s requires %d argument(s): %s" % (cmd, len(arg_spec), [a[0] for a in arg_spec]), file=sys.stderr)
                continue
            try:
                send_command_interactive(ser, cmd, args_list, serial_queue, timeout_s=5.0)
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
    """Print documented command set (help)."""
    print("Commands (use: send <command> [args...]):")
    print()
    for cli_name, _req, _resp, arg_spec, description in COMMANDS:
        args_str = " ".join("<%s>" % a[0] for a in arg_spec)
        if args_str:
            print("  %s %s  - %s" % (cli_name, args_str, description))
        else:
            print("  %s  - %s" % (cli_name, description))
    print()
    print("Examples:")
    print("  send ping")
    print("  send getconfig")
    print("  send setaddress 0x1234 0x5678")
    print("  send uwb-node-set-type tag")
    print("  send twr-mgr-set-targets 1,2,3")
    print("  send ota-send-address 0 0x200 0x1234")


def main():
    ap = argparse.ArgumentParser(
        description="UART protocol tool: send Request messages, listen for Response/log events (COBS+CRC16)."
    )
    ap.add_argument("--port", required=True, help="Serial port (e.g. COM10, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("interactive", help="Listen for messages and type commands in the same terminal (recommended)")
    sub.add_parser("listen", help="Listen and print received frames (responses and log events)")
    sub.add_parser("commands", help="List all send commands and their arguments")
    send_parser = sub.add_parser("send", help="Send a message (request); wait for response")
    send_parser.add_argument(
        "msg",
        choices=[c[0] for c in COMMANDS],
        metavar="command",
        help="Command name (e.g. ping, getconfig, setaddress)",
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
            cmd_spec = None
            for c in COMMANDS:
                if c[0] == args.msg:
                    cmd_spec = c
                    break
            if cmd_spec is None:
                print("Unknown command: %s" % args.msg, file=sys.stderr)
                sys.exit(1)
            _cli_name, _req, _resp, arg_spec, _ = cmd_spec
            if len(args.args) < len(arg_spec):
                print("Command %s requires %d argument(s): %s" % (args.msg, len(arg_spec), [a[0] for a in arg_spec]), file=sys.stderr)
                print("Run: uart_protocol_tool.py --port <PORT> commands", file=sys.stderr)
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
