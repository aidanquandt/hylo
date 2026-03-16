# Flashing STM32 from the devcontainer (WSL2 + Docker)

When you run `make flash REV=0` or `make flash REV=1` inside the devcontainer and see **OpenOCD "Error: open failed"**, the ST-Link is not visible to the container because WSL2 does not pass USB through by default.

The scripts in `.devcontainer/scripts/` are **PowerShell (.ps1)** because they run on Windows and call `usbipd`, a Windows tool. The project’s `tools/scripts/*.sh` scripts run inside the container.

## Quick setup (one-time per computer)

### 1. Install usbipd-win (Windows)

- Download from [usbipd-win releases](https://github.com/dorssel/usbipd-win/releases) and install, or:
  ```powershell
  winget install usbipd
  ```

### 2. One-time setup (recommended)

These scripts run on **Windows** (not in the container). usbipd is a Windows tool.

1. Open **PowerShell as Administrator** (right‑click → Run as administrator).
2. Go to the repo’s scripts folder. Examples:
   - Repo in WSL: `cd \\wsl$\Ubuntu\home\<you>\<path>\hylo\.devcontainer\scripts`
   - Repo on C: drive: `cd C:\Users\<you>\<path>\hylo\.devcontainer\scripts`
3. Run once per computer:
   ```powershell
   .\setup-stlink-autostart.ps1
   ```

This adds udev rules in WSL (ST-Link, CH340, FTDI, CP210x) and creates a Windows Task that attaches these devices to WSL when you log in. The container also mounts `/dev` so serial ports (e.g. for the webapp) are visible.

### 3. Manual attach (if needed)

If you skipped step 2, or the ST-Link was unplugged after boot, run from **Windows PowerShell as Administrator** (same `cd` as step 2):

```powershell
.\attach-stlink.ps1
```

The script finds the ST-Link, binds it if needed, and attaches to WSL.

### 4. Flash in the devcontainer

```bash
make flash REV=0   # or REV=1
```

---

## Verify

Inside the devcontainer:

```bash
lsusb
```

You should see an STMicroelectronics device (0483:3748, 374e, 374f, or 3754).

---

## Troubleshooting

- **"No ST-Link found"**: Plug in the ST-Link and run `attach-stlink.ps1` again.
- **"usbipd attach failed"**: Check that port **3240** is not blocked by Windows Firewall.
- **`LIBUSB_ERROR_ACCESS` / "open failed"**: The ST-Link is visible but OpenOCD lacks permission. Re-run `.\setup-stlink-autostart.ps1` (it adds udev rules in WSL). Then detach and re-attach: `usbipd detach --busid 2-7` then `.\attach-stlink.ps1`.
- **Task not running**: Open Task Scheduler → Task Scheduler Library → `Hylo-STLink-AttachWSL`. To remove the task, delete it from Task Scheduler.
- **Different computer**: Clone the repo and run `setup-stlink-autostart.ps1` once.
- **Webapp shows no ports**: Rebuild the devcontainer (e.g. "Dev Containers: Rebuild Container") so the `/dev` mount takes effect. Then run `.\attach-stlink.ps1` to attach ST-Link + USB-serial devices.

---

## Optional: Flash from Windows

If you prefer not to use usbipd:

1. Build in the devcontainer: `make build REV=0` or `make build REV=1`
2. Flash from Windows with OpenOCD against the built `.elf` (e.g. under `build/arm-gcc-debug/hwconfig/...`).
