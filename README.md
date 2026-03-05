# Hylo

UWB firmware (STM32H723) and host tooling. Build from repo root.

**Layout:** `cubemx/`, `platform/`, `lib/`, `config/`, `ports/`, `components/`, `common/`, `third_party/` (firmware) · `host/`, `testing/` (tools and test data) · `scripts/` (build, flash, check-deps).

## Build

Bash (MSYS2 UCRT64, Git Bash, or WSL):

```bash
make           # build (debug)
make release   # release build
make clean
make flash     # build + flash via OpenOCD (ST-Link)
make help      # list targets
```

**Requirements:** CMake ≥ 3.22, Ninja, ARM GCC (`arm-none-eabi-gcc`). Run `make check-deps` to verify.
