#!/usr/bin/env python3
"""
Run protocol codegen (nanopb + project codegen) and verify outputs.
Use this to confirm codegen works, not just that the build uses existing files.

  python tools/protocol_codegen/verify_codegen.py

Exits 0 if both steps succeed and generated files look correct.
"""
from __future__ import print_function

import os
import subprocess
import sys

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
    # 1. Nanopb generator (.pb.c / .pb.h) -> generated/protocol/c/
    run([sys.executable, os.path.join(CODEGEN_DIR, "run_nanopb_gen.py")], "nanopb generator")
    if not check_file(
        "generated/protocol/c/protocol.pb.h",
        "PingRequest",
        "SetAddressRequest",
        "pb.h",
    ):
        sys.exit(1)
    if not check_file("generated/protocol/c/protocol.pb.c", "PB_BIND"):
        sys.exit(1)

    # 2. Project codegen (ids, dispatch, tx -> c/; protocol_ids.py -> python/)
    run([sys.executable, os.path.join(CODEGEN_DIR, "codegen_protocol.py")], "project codegen")
    if not check_file("generated/protocol/c/protocol_ids.h", "MSG_ID_PingRequest", "MSG_ID_COUNT"):
        sys.exit(1)
    if not check_file("generated/protocol/c/protocol_dispatch.c", "protocol_dispatch", "protocol_rx_PingRequest"):
        sys.exit(1)
    if not check_file("generated/protocol/c/protocol_tx.c", "protocol_send_frame", "protocol_tx_PingRequest"):
        sys.exit(1)
    if not check_file("generated/protocol/python/protocol_ids.py", "MSG_ID_PingRequest", "MSG_NAMES"):
        sys.exit(1)

    # 3a. protoc --python_out for nanopb.proto -> generated/protocol/python/
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

    # 3a2. protoc --python_out for host_options.proto -> generated/protocol/python/
    run([
        "protoc",
        "--python_out=generated/protocol/python",
        "-I", "protocol",
        "host_options.proto",
    ], "protoc --python_out (host_options_pb2.py)")
    if not check_file("generated/protocol/python/host_options_pb2.py", "HostUiModule", "host_ui_module"):
        sys.exit(1)

    # 3b. protoc --python_out for host (protocol_pb2.py) -> generated/protocol/python/
    run([
        "protoc",
        "--python_out=generated/protocol/python",
        "-I", "protocol",
        "-I", nanopb_proto_dir,
        "protocol/protocol.proto",
    ], "protoc --python_out (protocol_pb2.py)")
    if not os.path.isfile(os.path.join(REPO_ROOT, "generated/protocol/python/protocol_pb2.py")):
        print("Missing: generated/protocol/python/protocol_pb2.py", file=sys.stderr)
        sys.exit(1)

    print("Codegen OK: nanopb + project codegen + protoc --python_out ran and outputs look correct.")
    print("Rebuild firmware to use the regenerated files.")


if __name__ == "__main__":
    main()
