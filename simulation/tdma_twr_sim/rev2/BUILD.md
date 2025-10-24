# Building the simulator (Windows)

This project is self-contained under `firmware/simulation/tdma_twr_sim`. Configure and build from here so no files are written to the repo root.

## Prereqs
- CMake 3.16+
- One of:
  - Ninja + MinGW GCC (recommended)
  - MinGW Makefiles
  - MSVC (Build Tools) + NMake or VS generator

## Configure + build (recommended: Ninja + MinGW GCC)
1. Open a terminal in `firmware/simulation/tdma_twr_sim`.
2. Configure to a `build` subfolder:
   ```powershell
   cmake -G "Ninja" -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=C:/mingw64/bin/gcc.exe
   ```
3. Build and run:
   ```powershell
   cmake --build build
   .\build\sim.exe
   ```

The binary and any `trace.vcd` output will stay within this subfolder.

## Alternatives
- MinGW Makefiles
  ```powershell
  cmake -G "MinGW Makefiles" -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=C:/mingw64/bin/gcc.exe
  cmake --build build
  .\build\sim.exe
  ```
- MSVC / NMake (use a "x64 Native Tools for VS" prompt)
  ```powershell
  cmake -G "NMake Makefiles" -S . -B build
  cmake --build build
  .\build\sim.exe
  ```

## Notes
- To keep the repo clean, avoid generating at the root. Always use `-S . -B build` from this folder.
- If switching generators, clear the `build` folder:
  ```powershell
  Remove-Item -Recurse -Force .\build
  ```
