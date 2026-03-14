#!/usr/bin/env python3
"""
Codegen: read protocol.proto and emit protocol_ids.h, protocol_dispatch.c/h,
protocol_tx.c/h (C), and protocol_ids.py (Python). Message IDs follow .proto definition order.
"""
from __future__ import print_function
import os
import re
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))
PROTO_PATH = os.path.join(REPO_ROOT, "protocol", "protocol.proto")
OUTPUT_DIR_C = os.path.join(REPO_ROOT, "generated", "protocol", "c")
OUTPUT_DIR_PY = os.path.join(REPO_ROOT, "generated", "protocol", "python")

CODEGEN_ATTRIBUTION = "/* Auto-generated from protocol.proto by tools/protocol_codegen/codegen_protocol.py */"
CODEGEN_ATTRIBUTION_PY = "# Auto-generated from protocol.proto by tools/protocol_codegen/codegen_protocol.py"


def parse_message_names(proto_path):
    """Extract message names in definition order from a .proto file."""
    with open(proto_path, "r", encoding="utf-8") as f:
        content = f.read()
    # Match "message Name {" (allow optional space before {)
    pattern = re.compile(r"^\s*message\s+(\w+)\s*\{", re.MULTILINE)
    return pattern.findall(content)


def emit_protocol_ids_h(messages, out_path):
    lines = [
        CODEGEN_ATTRIBUTION,
        "#ifndef PROTOCOL_IDS_H",
        "#define PROTOCOL_IDS_H",
        "#include <stdint.h>",
        "",
        "typedef enum {",
    ]
    for i, name in enumerate(messages):
        lines.append("  MSG_ID_{} = {},".format(name, i))
    lines.append("  MSG_ID_COUNT")
    lines.append("} protocol_msg_id_t;")
    lines.append("")
    lines.append("#endif /* PROTOCOL_IDS_H */")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def emit_protocol_dispatch_h(messages, out_path):
    lines = [
        CODEGEN_ATTRIBUTION,
        "#ifndef PROTOCOL_DISPATCH_H",
        "#define PROTOCOL_DISPATCH_H",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "#include \"protocol_ids.h\"",
        "#include \"protocol.pb.h\"",
        "",
        "void protocol_dispatch(uint16_t msg_id, const uint8_t *payload, size_t len);",
        "",
    ]
    for name in messages:
        lines.append("void protocol_rx_{}(const {} *msg);".format(name, name))
    lines.append("")
    lines.append("#endif /* PROTOCOL_DISPATCH_H */")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def emit_protocol_dispatch_c(messages, out_path):
    lines = [
        CODEGEN_ATTRIBUTION,
        "#include \"protocol_dispatch.h\"",
        "#include \"protocol.pb.h\"",
        "#include \"protocol_ids.h\"",
        "#include <pb_decode.h>",
        "#include <stddef.h>",
        "",
    ]
    for name in messages:
        lines.append("__attribute__((weak)) void protocol_rx_{}(const {} *msg) {{ (void)msg; }}".format(name, name))
    lines.append("")
    lines.append("void protocol_dispatch(uint16_t msg_id, const uint8_t *payload, size_t len)")
    lines.append("{")
    lines.append("  pb_istream_t stream = pb_istream_from_buffer(payload, len);")
    lines.append("  switch (msg_id) {")
    for name in messages:
        lines.append("    case MSG_ID_{}: {{".format(name))
        lines.append("      {} decoded = {}_init_zero;".format(name, name))
        lines.append("      if (pb_decode(&stream, {}_fields, &decoded))".format(name))
        lines.append("        protocol_rx_{}(&decoded);".format(name))
        lines.append("      break;")
        lines.append("    }")
    lines.append("    default:")
    lines.append("      break;")
    lines.append("  }")
    lines.append("}")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def emit_protocol_tx_h(messages, out_path):
    lines = [
        CODEGEN_ATTRIBUTION,
        "#ifndef PROTOCOL_TX_H",
        "#define PROTOCOL_TX_H",
        "#include \"protocol_ids.h\"",
        "#include \"protocol.pb.h\"",
        "",
    ]
    for name in messages:
        lines.append("void protocol_tx_{}(const {} *msg);".format(name, name))
    lines.append("")
    lines.append("#endif /* PROTOCOL_TX_H */")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def emit_protocol_tx_c(messages, out_path):
    lines = [
        CODEGEN_ATTRIBUTION,
        "#include \"protocol_tx.h\"",
        "#include \"protocol.pb.h\"",
        "#include \"uart_framing.h\"",
        "#include \"protocol_ids.h\"",
        "#include <pb_encode.h>",
        "#include <stddef.h>",
        "",
        "/* Single encode buffer; size must fit largest message (see PROTOCOL_PB_H_MAX_SIZE). */",
        "#define PROTOCOL_TX_BUF_SIZE 110",
        "static uint8_t s_tx_buf[PROTOCOL_TX_BUF_SIZE];",
        "",
    ]
    for name in messages:
        lines.append("void protocol_tx_{}(const {} *msg)".format(name, name))
        lines.append("{")
        lines.append("  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));")
        lines.append("  if (!pb_encode(&stream, {}_fields, msg))".format(name))
        lines.append("    return;")
        lines.append("  protocol_send_frame(MSG_ID_{}, s_tx_buf, stream.bytes_written);".format(name))
        lines.append("}")
        lines.append("")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def emit_protocol_ids_py(messages, out_path):
    lines = [
        CODEGEN_ATTRIBUTION_PY,
        "",
    ]
    for i, name in enumerate(messages):
        lines.append("MSG_ID_{} = {}".format(name, i))
    lines.append("MSG_ID_COUNT = {}".format(len(messages)))
    lines.append("")
    lines.append("# Name list for dispatch")
    lines.append("MSG_NAMES = (")
    for name in messages:
        lines.append("    \"{}\",".format(name))
    lines.append(")")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    if not os.path.isfile(PROTO_PATH):
        sys.stderr.write("Proto file not found: %s\n" % PROTO_PATH)
        sys.exit(1)
    os.makedirs(OUTPUT_DIR_C, exist_ok=True)
    os.makedirs(OUTPUT_DIR_PY, exist_ok=True)
    messages = parse_message_names(PROTO_PATH)
    if not messages:
        sys.stderr.write("No messages found in %s\n" % PROTO_PATH)
        sys.exit(1)
    emit_protocol_ids_h(messages, os.path.join(OUTPUT_DIR_C, "protocol_ids.h"))
    emit_protocol_dispatch_h(messages, os.path.join(OUTPUT_DIR_C, "protocol_dispatch.h"))
    emit_protocol_dispatch_c(messages, os.path.join(OUTPUT_DIR_C, "protocol_dispatch.c"))
    emit_protocol_tx_h(messages, os.path.join(OUTPUT_DIR_C, "protocol_tx.h"))
    emit_protocol_tx_c(messages, os.path.join(OUTPUT_DIR_C, "protocol_tx.c"))
    emit_protocol_ids_py(messages, os.path.join(OUTPUT_DIR_PY, "protocol_ids.py"))
    print("Generated C in", OUTPUT_DIR_C, "and Python in", OUTPUT_DIR_PY)


if __name__ == "__main__":
    main()
