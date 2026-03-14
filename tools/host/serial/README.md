# Host serial tools

Tools for talking to the device over UART using the framed protobuf protocol (COBS + CRC16).

## UART protocol tool (`uart_protocol_tool.py`)

Send Request messages and receive Response / log-event messages. Uses generated `generated/protocol/uart_protocol_pb2.py` and `protocol_ids.py`.

### Prerequisites

1. Run protocol codegen so that `generated/protocol/uart_protocol_pb2.py` and `generated/protocol/nanopb_pb2.py` exist:
   - `python protocol/verify_codegen.py`
   - If `nanopb_pb2` is missing, generate it:
     `protoc --python_out=generated/protocol -I third_party/nanopb/generator/proto third_party/nanopb/generator/proto/nanopb.proto`
2. Install pyserial: `pip install pyserial` (or use your system package manager).

### Usage

```text
python uart_protocol_tool.py --port <PORT> interactive
python uart_protocol_tool.py --port <PORT> listen
python uart_protocol_tool.py --port <PORT> send <command> [args...]
python uart_protocol_tool.py --port <PORT> commands
```

- **`interactive`** — **Recommended.** One terminal: continuously prints incoming messages and lets you type commands whenever. Type `help` or `commands` for the command list, `quit` or `exit` to leave.
- **`listen`** — Listen and print all received frames (responses and log events). Decodes using `*_pb2.py` and an optional format table for human-readable log events.
- **`send <command> [args...]`** — Send a Request and wait for the matching Response (or `AckResponse` for fire-and-forget commands).
- **`commands`** — List all supported commands and their arguments.

### Command set

Run `python uart_protocol_tool.py --port COM1 commands` for the full list. Summary by domain:

| Domain        | Examples |
|---------------|----------|
| System        | `get-uuid`, `get-info` |
| Beacon        | `ping`, `beacon-ping` |
| UWB node      | `getconfig`, `setaddress`, `uwb-node-get-type`, `uwb-node-set-type`, `uwb-node-get-position`, `uwb-node-set-position`, `uwb-node-get-status` |
| UWB           | `uwb-get-status`, `uwb-get-stats`, `uwb-start`, `uwb-stop`, `uwb-reset-stats` |
| Error         | `error-clear` |
| IMU           | `imu-get-status`, `imu-get-data`, `imu-stream-start`, `imu-stream-stop` |
| Datalogger    | `datalogger-get-tasks`, `datalogger-get-stats` |
| TWR           | `twr-get-status`, `twr-get-result`, `twr-range` |
| TWR manager   | `twr-mgr-start`, `twr-mgr-stop`, `twr-mgr-get-status`, `twr-mgr-add-target`, `twr-mgr-remove-target`, `twr-mgr-set-targets`, `twr-mgr-clear-targets`, `twr-mgr-set-ranging-rate`, `twr-mgr-get-ranging-rate` |
| Stopwatch     | `stopwatch-get`, `stopwatch-start`, `stopwatch-stop` |
| OTA config    | `ota-send-address`, `ota-send-position`, `ota-send-type`, `ota-send-gpio`, `ota-set-token`, `ota-get-token`, `ota-get-stats` |
| Sensor fusion | `sf-get-debug`, `sf-set-debug`, `sf-get-status`, `sf-set-active`, `sf-get-imu-enabled`, `sf-set-imu-enabled`, `sf-set-noise`, `sf-get-noise`, `sf-get-config`, `sf-set-config` |

### Examples

```text
send ping
send getconfig
send setaddress 0x1234 0x5678
send uwb-node-set-type tag
send twr-mgr-set-targets 1,2,3
send ota-send-address 0 0x200 0x1234
```

Arguments that accept hex: use `0x` prefix (e.g. `0x1234`). Node type: `tag`, `anchor`, or `hybrid`.
