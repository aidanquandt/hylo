#!/usr/bin/env python3
"""
Run protocol codegen (nanopb + project codegen) and verify outputs.
Use this to confirm codegen works, not just that the build uses existing files.

  python tools/protocol_codegen/verify_codegen.py

Exits 0 if both steps succeed and generated files look correct.
"""
from __future__ import print_function

import glob
import os
import subprocess
import sys

from protocol_barrel import protocol_py_protos_for_verify

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
CODEGEN_DIR = os.path.join(REPO_ROOT, "tools", "protocol_codegen")
GENERATED_C = os.path.join(REPO_ROOT, "generated", "protocol", "c")
GENERATED_PY = os.path.join(REPO_ROOT, "generated", "protocol", "python")


def run(cmd, description):
    print("Running:", description)
    result = subprocess.run(cmd, cwd=REPO_ROOT, shell=False)
    if result.returncode != 0:
        print("Failed:", description, file=sys.stderr)
        sys.exit(1)


def run_protoc_host_python(cmd, description):
    """Run protoc; suppress protocol.proto unused-import warnings (barrel file has no new symbols)."""
    print("Running:", description)
    result = subprocess.run(cmd, cwd=REPO_ROOT, shell=False, capture_output=True, text=True)
    if result.returncode != 0:
        if result.stdout:
            sys.stdout.write(result.stdout)
        if result.stderr:
            sys.stderr.write(result.stderr)
        print("Failed:", description, file=sys.stderr)
        sys.exit(1)
    if result.stdout:
        sys.stdout.write(result.stdout)
    for line in (result.stderr or "").splitlines():
        if "protocol.proto:" in line and "unused" in line.lower():
            continue
        if line.strip():
            sys.stderr.write(line + "\n")


def check_file(path, *required_substrings):
    path = os.path.join(REPO_ROOT, path)
    if not os.path.isfile(path):
        print("Missing:", path, file=sys.stderr)
        return False
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()
    for s in required_substrings:
        if s not in text:
            print("Expected %r not found in %s" % (s, path), file=sys.stderr)
            return False
    return True


def main():
    os.makedirs(GENERATED_PY, exist_ok=True)

    # 1. Nanopb generator (.pb.c / .pb.h) -> generated/protocol/c/
    run([sys.executable, os.path.join(CODEGEN_DIR, "run_nanopb_gen.py")], "nanopb generator")
    if not check_file(
        "generated/protocol/c/protocol.pb.h",
        "messages/protocol_ping.pb.h",
        "PB_PROTOCOL_PB_H_INCLUDED",
    ):
        sys.exit(1)
    if not check_file(
        "generated/protocol/c/messages/protocol_ping.pb.h",
        "PingRequest",
        "pb.h",
    ):
        sys.exit(1)
    if not check_file("generated/protocol/c/messages/protocol_common.pb.c", "PB_BIND"):
        sys.exit(1)

    # 2. host_options_pb2.py before project codegen (codegen validates host.host_message_type via extensions)
    run([
        "protoc",
        "--python_out=generated/protocol/python",
        "-I", "protocol",
        "host_options.proto",
    ], "protoc --python_out (host_options_pb2.py)")
    if not check_file(
        "generated/protocol/python/host_options_pb2.py",
        "HostUiModule",
        "HostMessageType",
        "host_ui_module",
        "default_host_ui_module",
        "host_message_type",
    ):
        sys.exit(1)

    # 3. Project codegen (ids, dispatch, tx -> c/; protocol_ids.py -> python/)
    run([sys.executable, os.path.join(CODEGEN_DIR, "codegen_protocol.py")], "project codegen")
    if not check_file("generated/protocol/c/protocol_ids.h", "MSG_ID_PingRequest", "MSG_ID_COUNT"):
        sys.exit(1)
    if not check_file("generated/protocol/c/protocol_dispatch.c", "protocol_dispatch", "protocol_rx_PingRequest"):
        sys.exit(1)
    if not check_file("generated/protocol/c/protocol_tx.c", "protocol_send_frame", "protocol_tx_PingRequest"):
        sys.exit(1)
    if not check_file("generated/protocol/python/protocol_ids.py", "MSG_ID_PingRequest", "MSG_NAMES"):
        sys.exit(1)

    # 4a. protoc --python_out for nanopb.proto -> generated/protocol/python/
    nanopb_proto_dir = "third_party/nanopb/generator/proto"
    nanopb_proto_file = os.path.join(REPO_ROOT, nanopb_proto_dir, "nanopb.proto")
    if os.path.isfile(nanopb_proto_file):
        run([
            "protoc",
            "--python_out=generated/protocol/python",
            "-I", nanopb_proto_dir,
            os.path.join(nanopb_proto_dir, "nanopb.proto"),
        ], "protoc --python_out (nanopb_pb2.py)")
        if not os.path.isfile(os.path.join(REPO_ROOT, "generated/protocol/python/nanopb_pb2.py")):
            print("Missing: generated/protocol/python/nanopb_pb2.py", file=sys.stderr)
            sys.exit(1)
    else:
        print("Warning: %s not found (nanopb submodule?). Run: git submodule update --init third_party/nanopb" % nanopb_proto_file, file=sys.stderr)
        print("Host tool (protocol_tool.py) will fail to import protocol_pb2 until nanopb_pb2.py exists.", file=sys.stderr)

    # 4b. protoc --python_out for host (messages/protocol_*_pb2.py + protocol_pb2.py) -> generated/protocol/python/
    for stale in glob.glob(os.path.join(GENERATED_PY, "protocol_*_pb2.py")):
        try:
            os.remove(stale)
        except OSError:
            pass
    protocol_py_protos = protocol_py_protos_for_verify()
    run_protoc_host_python(
        ["protoc", "--python_out=generated/protocol/python", "-I", "protocol", "-I", nanopb_proto_dir]
        + protocol_py_protos,
        "protoc --python_out (protocol*_pb2.py)",
    )
    if not os.path.isfile(os.path.join(REPO_ROOT, "generated/protocol/python/protocol_pb2.py")):
        print("Missing: generated/protocol/python/protocol_pb2.py", file=sys.stderr)
        sys.exit(1)
    if not check_file("generated/protocol/python/wire_catalog_pb2.py", "WireMessageId"):
        sys.exit(1)

    print("Codegen OK: nanopb + project codegen + protoc --python_out ran and outputs look correct.")
    print("Rebuild firmware to use the regenerated files.")


if __name__ == "__main__":
    main()
