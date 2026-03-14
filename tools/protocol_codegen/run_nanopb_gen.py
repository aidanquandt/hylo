#!/usr/bin/env python3
"""Run nanopb generator with optional pkg_resources stub for environments without setuptools."""
from __future__ import absolute_import
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.normpath(os.path.join(script_dir, "..", ".."))

# Prefer vendored nanopb (third_party/nanopb); else NANOPB_SRC env or legacy build path
nanopb_vendored = os.path.join(repo_root, "third_party", "nanopb")
nanopb_legacy = os.path.join(repo_root, "build", "_deps", "nanopb-src")
nanopb_src = os.environ.get("NANOPB_SRC", nanopb_vendored if os.path.isdir(nanopb_vendored) else nanopb_legacy)
generator_dir = os.path.join(nanopb_src, "generator")
proto_dir = os.path.join(nanopb_src, "generator", "proto")

# Stub pkg_resources before any nanopb imports (Python 3.12+ may not have setuptools).
class _PkgResourcesStub:
    def resource_filename(self, package, resource):
        return proto_dir

if "pkg_resources" not in sys.modules:
    sys.modules["pkg_resources"] = type(sys)("pkg_resources")
    sys.modules["pkg_resources"].resource_filename = _PkgResourcesStub().resource_filename

if not os.path.isdir(generator_dir):
    sys.stderr.write(
        "Nanopb generator not found at %s.\n"
        "Run: git submodule update --init third_party/nanopb\n" % generator_dir
    )
    sys.exit(1)

sys.path.insert(0, generator_dir)
os.chdir(repo_root)

# Run generator: -D generated/protocol/c -I protocol -I generator/proto protocol/protocol.proto
out_dir = os.path.join(repo_root, "generated", "protocol", "c")
os.makedirs(out_dir, exist_ok=True)

# Invoke as module to use generator's main
import importlib.util
spec = importlib.util.spec_from_file_location("nanopb_generator", os.path.join(generator_dir, "nanopb_generator.py"))
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

# main_cli expects sys.argv; we set it and call main_cli
# Generate host_options first (protocol.proto imports it); nanopb finds
# google/protobuf/descriptor.proto via proto_dir (nanopb generator/proto).
protocol_dir = os.path.join(repo_root, "protocol")
old_argv = sys.argv
sys.argv = [
    "nanopb_generator",
    "-D", out_dir,
    "-I", protocol_dir,
    "-I", proto_dir,
    os.path.join(protocol_dir, "host_options.proto"),
    os.path.join(protocol_dir, "protocol.proto"),
]
try:
    gen.main_cli()
finally:
    sys.argv = old_argv
