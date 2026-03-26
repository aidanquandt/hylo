"""
Parse protocol/protocol.proto barrel imports — single source for message .proto list order.

Skips wire_catalog.proto (IDs only / not in firmware nanopb set). All other imports must be paths
relative to the protocol/ directory (e.g. messages/foo.proto).
"""
from __future__ import print_function

import os
import re

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))
PROTO_DIR = os.path.join(REPO_ROOT, "protocol")
BARREL_REL = "protocol.proto"
WIRE_CATALOG_SKIP = "wire_catalog.proto"

_IMPORT_RE = re.compile(
    r'^\s*import\s+(?:public\s+|weak\s+)?"([^"]+)"\s*;\s*(?://.*)?$'
)


def _strip_line_comment(line):
    return line.split("//", 1)[0].rstrip()


def load_message_proto_paths_relative_to_protocol_dir(barrel_path=None):
    """
    Return ordered import paths relative to protocol/ (e.g. messages/protocol_common.proto).
    Skips only wire_catalog.proto.
    """
    if barrel_path is None:
        barrel_path = os.path.join(PROTO_DIR, BARREL_REL)
    if not os.path.isfile(barrel_path):
        raise IOError("Barrel proto not found: %s" % barrel_path)
    out = []
    with open(barrel_path, "r", encoding="utf-8") as f:
        for raw in f:
            line = _strip_line_comment(raw)
            m = _IMPORT_RE.match(line)
            if not m:
                continue
            rel = m.group(1).replace("\\", "/")
            base = os.path.basename(rel)
            if base == WIRE_CATALOG_SKIP:
                continue
            out.append(rel)
    return out


def protocol_py_protos_for_verify():
    """
    Paths relative to repo root for verify_codegen.py protoc --python_out batch:
    host_options.proto, wire_catalog.proto, barrel message protos in order, protocol.proto last.
    """
    msgs = load_message_proto_paths_relative_to_protocol_dir()
    return (
        ["protocol/host_options.proto", "protocol/wire_catalog.proto"]
        + ["protocol/%s" % m for m in msgs]
        + ["protocol/protocol.proto"]
    )
