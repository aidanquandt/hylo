#!/usr/bin/env python3
"""
Run protocol codegen (nanopb + project codegen) and verify outputs.
Use this to confirm codegen works, not just that the build uses existing files.

  python protocol/verify_codegen.py

Exits 0 if both steps succeed and generated files look correct.
"""
from __future__ import print_function

import os
import subprocess
import sys

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
GENERATED = os.path.join(os.path.dirname(__file__), "generated")


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
    # 1. Nanopb generator (.pb.c / .pb.h)
    run([sys.executable, "protocol/run_nanopb_gen.py"], "nanopb generator")
    if not check_file(
        "protocol/generated/uart_protocol.pb.h",
        "PingRequest",
        "SetAddressRequest",
        "pb.h",
    ):
        sys.exit(1)
    if not check_file("protocol/generated/uart_protocol.pb.c", "PB_BIND"):
        sys.exit(1)

    # 2. Project codegen (ids, dispatch, tx, Python ids)
    run([sys.executable, "protocol/codegen_protocol.py"], "project codegen")
    if not check_file("protocol/generated/protocol_ids.h", "MSG_ID_PingRequest", "MSG_ID_COUNT"):
        sys.exit(1)
    if not check_file("protocol/generated/protocol_dispatch.c", "protocol_dispatch", "protocol_rx_PingRequest"):
        sys.exit(1)
    if not check_file("protocol/generated/protocol_tx.c", "protocol_send_frame", "protocol_tx_PingRequest"):
        sys.exit(1)
    if not check_file("protocol/generated/protocol_ids.py", "MSG_ID_PingRequest", "MSG_NAMES"):
        sys.exit(1)

    print("Codegen OK: nanopb + project codegen ran and outputs look correct.")
    print("Rebuild firmware to use the regenerated files.")


if __name__ == "__main__":
    main()
