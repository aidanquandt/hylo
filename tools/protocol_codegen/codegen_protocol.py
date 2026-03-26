#!/usr/bin/env python3
"""
Codegen: read protocol/wire_catalog.proto (host.WireMessageId) plus barrel imports in
protocol/protocol.proto and emit protocol_ids.h, protocol_dispatch.c/h, protocol_tx.c/h (C), and
protocol_ids.py (Python).

Wire IDs: add an enumerator in wire_catalog.proto whose name matches the top-level message type,
then define the message in a file imported by protocol.proto (after wire_catalog). Import order in
protocol.proto is the single source for codegen, nanopb, CMake, and verify_codegen.

Each wire message must set option (host.host_message_type) (see host_options.proto); UNSPECIFIED
fails validation.

Host UI module: set option (host.default_host_ui_module) once per messages/*.proto file, or
(host.host_ui_module) on an individual message to override. Effective value must not be UNSPECIFIED.
"""
from __future__ import print_function

import os
import subprocess
import sys
import tempfile

from google.protobuf import descriptor_pb2

from protocol_barrel import load_message_proto_paths_relative_to_protocol_dir

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))
PROTO_DIR = os.path.join(REPO_ROOT, "protocol")
OUTPUT_DIR_C = os.path.join(REPO_ROOT, "generated", "protocol", "c")
OUTPUT_DIR_PY = os.path.join(REPO_ROOT, "generated", "protocol", "python")
NANOPB_GEN_PROTO = os.path.join(REPO_ROOT, "third_party", "nanopb", "generator", "proto")

CODEGEN_ATTRIBUTION = "/* Auto-generated from protocol/protocol.proto barrel by tools/protocol_codegen/codegen_protocol.py */"
CODEGEN_ATTRIBUTION_PY = "# Auto-generated from protocol/protocol.proto barrel by tools/protocol_codegen/codegen_protocol.py"

# Canonical ID list (names must match top-level message types).
WIRE_CATALOG_PROTO = "wire_catalog.proto"

def _message_proto_files():
    return load_message_proto_paths_relative_to_protocol_dir()


def _descriptor_proto_inputs():
    paths = [os.path.join(PROTO_DIR, WIRE_CATALOG_PROTO)]
    paths.extend(os.path.join(PROTO_DIR, p) for p in _message_proto_files())
    return paths


def _run_protoc_descriptor_set(out_path):
    if not os.path.isdir(NANOPB_GEN_PROTO):
        sys.stderr.write(
            "Nanopb proto dir missing: %s\nRun: git submodule update --init third_party/nanopb\n"
            % NANOPB_GEN_PROTO
        )
        sys.exit(1)
    proto_inputs = _descriptor_proto_inputs()
    for p in proto_inputs:
        if not os.path.isfile(p):
            sys.stderr.write("Proto file not found: %s\n" % p)
            sys.exit(1)
    cmd = [
        "protoc",
        "--descriptor_set_out=" + out_path,
        "--include_imports",
        "-I",
        PROTO_DIR,
        "-I",
        NANOPB_GEN_PROTO,
    ] + proto_inputs
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(result.stderr or result.stdout or "protoc failed\n")
        sys.exit(1)


def _load_wire_id_map_from_fds(fds):
    for fd in fds.file:
        if fd.name != WIRE_CATALOG_PROTO:
            continue
        for et in fd.enum_type:
            if et.name == "WireMessageId":
                mp = {}
                for v in et.value:
                    if v.name in mp:
                        sys.stderr.write("duplicate enum value name in WireMessageId: %s\n" % v.name)
                        sys.exit(1)
                    mp[v.name] = v.number
                return mp
    sys.stderr.write("Missing %s with enum WireMessageId\n" % WIRE_CATALOG_PROTO)
    sys.exit(1)


def load_file_descriptor_set():
    """Build FileDescriptorSet via protoc. Caller must import host_options_pb2 first so MessageOptions keep custom options."""
    with tempfile.NamedTemporaryFile(suffix=".pb", delete=False) as tmp:
        tmp_path = tmp.name
    try:
        _run_protoc_descriptor_set(tmp_path)
        with open(tmp_path, "rb") as f:
            return descriptor_pb2.FileDescriptorSet.FromString(f.read())
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass


def collect_wire_messages_from_fds(fds):
    """
    Returns list of (message_name, wire_id) in protocol.proto import order (per-file declaration order).
    """
    id_by_msg_name = _load_wire_id_map_from_fds(fds)
    by_rel = {fd.name: fd for fd in fds.file}
    ordered = []
    for rel in _message_proto_files():
        fd = by_rel.get(rel)
        if fd is None:
            sys.stderr.write("descriptor set missing file %r (protoc include path?)\n" % rel)
            sys.exit(1)
        for msg in fd.message_type:
            wid = id_by_msg_name.get(msg.name)
            if wid is None:
                sys.stderr.write(
                    "%s: message %s has no matching host.WireMessageId entry in %s\n"
                    % (rel, msg.name, WIRE_CATALOG_PROTO)
                )
                sys.exit(1)
            ordered.append((msg.name, wid))

    catalog_names = set(id_by_msg_name.keys())
    message_names = {name for name, _ in ordered}
    orphan = catalog_names - message_names
    if orphan:
        sys.stderr.write(
            "%s: enumerators with no top-level message: %s\n"
            % (WIRE_CATALOG_PROTO, ", ".join(sorted(orphan)))
        )
        sys.exit(1)

    ids = [w for _, w in ordered]
    if len(ids) != len(set(ids)):
        seen = {}
        for name, w in ordered:
            if w in seen:
                sys.stderr.write("Duplicate wire id %d: %s and %s\n" % (w, seen[w], name))
                sys.exit(1)
            seen[w] = name

    return ordered


def _ensure_host_options_pb2_py():
    """host_options_pb2 registers MessageOptions extensions needed to read custom options from FileDescriptorProto."""
    pb2_path = os.path.join(OUTPUT_DIR_PY, "host_options_pb2.py")
    if os.path.isfile(pb2_path):
        return
    ho = os.path.join(PROTO_DIR, "host_options.proto")
    r = subprocess.run(
        ["protoc", "--python_out=" + OUTPUT_DIR_PY, "-I", PROTO_DIR, ho],
        capture_output=True,
        text=True,
    )
    if r.returncode != 0:
        sys.stderr.write(r.stderr or r.stdout or "protoc host_options.proto failed\n")
        sys.exit(1)


def _descriptor_proto_by_message_name(fds):
    by_name = {}
    by_rel = {fd.name: fd for fd in fds.file}
    for rel in _message_proto_files():
        fd = by_rel.get(rel)
        if fd is None:
            continue
        for msg in fd.message_type:
            by_name[msg.name] = msg
    return by_name


def _descriptor_proto_and_file_by_message_name(fds):
    """Map top-level message name -> (DescriptorProto, FileDescriptorProto)."""
    by_name = {}
    by_rel = {fd.name: fd for fd in fds.file}
    for rel in _message_proto_files():
        fd = by_rel.get(rel)
        if fd is None:
            continue
        for msg in fd.message_type:
            by_name[msg.name] = (msg, fd)
    return by_name


def _validate_host_message_types(fds, ordered, host_options_pb2_mod):
    """Every wire message must set (host.host_message_type) to a value other than UNSPECIFIED (0)."""
    ext = host_options_pb2_mod.host_message_type
    by_msg = _descriptor_proto_by_message_name(fds)
    unspecified = host_options_pb2_mod.HOST_MESSAGE_TYPE_UNSPECIFIED
    for msg_name, _ in ordered:
        dp = by_msg.get(msg_name)
        if dp is None:
            sys.stderr.write("host_message_type check: missing DescriptorProto for %s\n" % msg_name)
            sys.exit(1)
        val = dp.options.Extensions[ext]
        if val == unspecified:
            sys.stderr.write(
                "message %s: set option (host.host_message_type) to a non-UNSPECIFIED HostMessageType\n"
                % msg_name
            )
            sys.exit(1)


def _validate_host_ui_modules(fds, ordered, host_options_pb2_mod):
    """Resolve HostUiModule from message option, else file (default_host_ui_module); must not be UNSPECIFIED."""
    msg_ext = host_options_pb2_mod.host_ui_module
    file_ext = host_options_pb2_mod.default_host_ui_module
    unspecified = host_options_pb2_mod.HOST_UI_MODULE_UNSPECIFIED
    by_msg = _descriptor_proto_and_file_by_message_name(fds)
    for msg_name, _ in ordered:
        pair = by_msg.get(msg_name)
        if pair is None:
            sys.stderr.write("host_ui_module check: missing DescriptorProto for %s\n" % msg_name)
            sys.exit(1)
        dp, fd = pair
        if dp.options.HasExtension(msg_ext):
            val = dp.options.Extensions[msg_ext]
        elif fd.options.HasExtension(file_ext):
            val = fd.options.Extensions[file_ext]
        else:
            val = unspecified
        if val == unspecified:
            sys.stderr.write(
                "message %s: set option (host.default_host_ui_module) on its .proto file "
                "or (host.host_ui_module) on the message\n" % msg_name
            )
            sys.exit(1)


def _validate_dense_ids(ordered_pairs):
    """Guardrail: IDs are dense 0..N-1 (UART code often assumes a contiguous range)."""
    ids = sorted({w for _, w in ordered_pairs})
    expected = list(range(len(ids)))
    if ids != expected:
        sys.stderr.write(
            "WireMessageId values are not dense 0..N-1; got %r\n"
            "Fix wire_catalog.proto or relax this check in codegen_protocol.py\n" % ids
        )
        sys.exit(1)


def emit_protocol_ids_h(ordered_by_id, out_path):
    lines = [
        CODEGEN_ATTRIBUTION,
        "#ifndef PROTOCOL_IDS_H",
        "#define PROTOCOL_IDS_H",
        "#include <stdint.h>",
        "",
        "typedef enum {",
    ]
    for name, wid in ordered_by_id:
        lines.append("  MSG_ID_{} = {},".format(name, wid))
    count = ordered_by_id[-1][1] + 1 if ordered_by_id else 0
    lines.append("  MSG_ID_COUNT = {}".format(count))
    lines.append("} protocol_msg_id_t;")
    lines.append("")
    lines.append("#endif /* PROTOCOL_IDS_H */")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def emit_protocol_dispatch_h(ordered_by_id, out_path):
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
    for name, _ in ordered_by_id:
        lines.append("void protocol_rx_{}(const {} *msg);".format(name, name))
    lines.append("")
    lines.append("#endif /* PROTOCOL_DISPATCH_H */")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def emit_protocol_dispatch_c(ordered_by_id, out_path):
    lines = [
        CODEGEN_ATTRIBUTION,
        "#include \"protocol_dispatch.h\"",
        "#include \"protocol.pb.h\"",
        "#include \"protocol_ids.h\"",
        "#include <pb_decode.h>",
        "#include <stddef.h>",
        "",
    ]
    for name, _ in ordered_by_id:
        lines.append("__attribute__((weak)) void protocol_rx_{}(const {} *msg) {{ (void)msg; }}".format(name, name))
    lines.append("")
    lines.append("void protocol_dispatch(uint16_t msg_id, const uint8_t *payload, size_t len)")
    lines.append("{")
    lines.append("  pb_istream_t stream = pb_istream_from_buffer(payload, len);")
    lines.append("  switch (msg_id) {")
    for name, wid in ordered_by_id:
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


def emit_protocol_tx_h(ordered_by_id, out_path):
    lines = [
        CODEGEN_ATTRIBUTION,
        "#ifndef PROTOCOL_TX_H",
        "#define PROTOCOL_TX_H",
        "#include \"protocol_ids.h\"",
        "#include \"protocol.pb.h\"",
        "",
    ]
    for name, _ in ordered_by_id:
        lines.append("void protocol_tx_{}(const {} *msg);".format(name, name))
    lines.append("")
    lines.append("#endif /* PROTOCOL_TX_H */")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def emit_protocol_tx_c(ordered_by_id, out_path):
    lines = [
        CODEGEN_ATTRIBUTION,
        "#include \"protocol_tx.h\"",
        "#include \"protocol.pb.h\"",
        "#include \"uart_framing.h\"",
        "#include \"protocol_ids.h\"",
        "#include <pb_encode.h>",
        "#include <stddef.h>",
        "",
        "/* Stack-local encode buffer per call: avoids shared-state races across tasks. */",
        "#define PROTOCOL_TX_BUF_SIZE 110",
        "",
    ]
    for name, _ in ordered_by_id:
        lines.append("void protocol_tx_{}(const {} *msg)".format(name, name))
        lines.append("{")
        lines.append("  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];")
        lines.append("  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));")
        lines.append("  if (!pb_encode(&stream, {}_fields, msg))".format(name))
        lines.append("    return;")
        lines.append("  protocol_send_frame(MSG_ID_{}, tx_buf, stream.bytes_written);".format(name))
        lines.append("}")
        lines.append("")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def emit_protocol_ids_py(ordered_by_id, out_path):
    lines = [
        CODEGEN_ATTRIBUTION_PY,
        "",
    ]
    for name, wid in ordered_by_id:
        lines.append("MSG_ID_{} = {}".format(name, wid))
    count = ordered_by_id[-1][1] + 1 if ordered_by_id else 0
    lines.append("MSG_ID_COUNT = {}".format(count))
    lines.append("")
    lines.append("# Name list for dispatch (sorted by wire id)")
    lines.append("MSG_NAMES = (")
    for name, _ in ordered_by_id:
        lines.append("    \"{}\",".format(name))
    lines.append(")")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    os.makedirs(OUTPUT_DIR_C, exist_ok=True)
    os.makedirs(OUTPUT_DIR_PY, exist_ok=True)
    _ensure_host_options_pb2_py()
    if OUTPUT_DIR_PY not in sys.path:
        sys.path.insert(0, OUTPUT_DIR_PY)
    import host_options_pb2 as host_options_pb2_mod

    fds = load_file_descriptor_set()
    ordered = collect_wire_messages_from_fds(fds)
    if not ordered:
        sys.stderr.write("No messages found under %s\n" % PROTO_DIR)
        sys.exit(1)
    _validate_host_message_types(fds, ordered, host_options_pb2_mod)
    _validate_host_ui_modules(fds, ordered, host_options_pb2_mod)
    _validate_dense_ids(ordered)
    ordered_by_id = sorted(ordered, key=lambda x: (x[1], x[0]))
    emit_protocol_ids_h(ordered_by_id, os.path.join(OUTPUT_DIR_C, "protocol_ids.h"))
    emit_protocol_dispatch_h(ordered_by_id, os.path.join(OUTPUT_DIR_C, "protocol_dispatch.h"))
    emit_protocol_dispatch_c(ordered_by_id, os.path.join(OUTPUT_DIR_C, "protocol_dispatch.c"))
    emit_protocol_tx_h(ordered_by_id, os.path.join(OUTPUT_DIR_C, "protocol_tx.h"))
    emit_protocol_tx_c(ordered_by_id, os.path.join(OUTPUT_DIR_C, "protocol_tx.c"))
    emit_protocol_ids_py(ordered_by_id, os.path.join(OUTPUT_DIR_PY, "protocol_ids.py"))
    print("Generated C in", OUTPUT_DIR_C, "and Python in", OUTPUT_DIR_PY)


if __name__ == "__main__":
    main()
