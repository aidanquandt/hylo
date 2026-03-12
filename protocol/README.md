# UART protocol

Single `.proto` schema, nanopb-generated C, and project codegen for dispatch/TX and Python message IDs. No virtualenv required.

Nanopb is **vendored** as a git submodule at `third_party/nanopb` (pinned to 0.4.8). No FetchContent, no network at build time.

**First-time clone (or after `git pull` if submodule was added):** run once to fetch nanopb:
```bash
git submodule update --init third_party/nanopb
```
To upgrade nanopb later: `cd third_party/nanopb && git fetch && git checkout <tag>` then commit the updated submodule reference in the repo root.

## Regenerating after editing the .proto

**Regenerate and verify** (run from repo root):

```bash
make protocol-codegen
```

Or manually:

```bash
python protocol/run_nanopb_gen.py && python protocol/codegen_protocol.py
```

To **test that codegen actually works** (not just that the build uses existing files), run the verifier—it runs both steps and checks the outputs:

```bash
python protocol/verify_codegen.py
```

- **run_nanopb_gen.py** — regenerates `protocol/generated/uart_protocol.pb.c` and `.pb.h`.
- **codegen_protocol.py** — regenerates `protocol_ids.h`, `protocol_dispatch.*`, `protocol_tx.*`, and `protocol_ids.py`.

Both scripts use system Python. No `.venv` or `pip` needed if you install dependencies with your system package manager.

### MSYS2 (ucrt64)

From an MSYS2 UCRT64 shell:

```bash
pacman -S mingw-w64-ucrt-x86_64-python-protobuf
```

Then run the scripts above. For the host tool (`tools/host/serial/uart_protocol_tool.py`) you also need:

```bash
pacman -S mingw-w64-ucrt-x86_64-python-pyserial
```

### Other systems

If you prefer pip: `pip install protobuf` (and `pyserial` for the host tool). The nanopb generator is bundled in the repo (fetched by CMake); `run_nanopb_gen.py` stubs `pkg_resources` so setuptools is optional.
