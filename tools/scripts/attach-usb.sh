#!/usr/bin/env bash
# Launch "Attach Hylo USB" Desktop shortcut.
# Requires: setup-stlink-autostart.ps1 was run once to create the shortcut.
# The shortcut triggers UAC; user clicks Yes to attach devices.
# Works from WSL; from container, falls back to instructions.

if powershell.exe -NoProfile -Command "Start-Process (Join-Path [Environment]::GetFolderPath('Desktop') 'Attach Hylo USB.lnk') -Verb RunAs" 2>/dev/null; then
    echo "Launched. Click Yes on the UAC prompt."
    exit 0
fi

# Fallback: print instructions
echo "Attach USB devices to WSL:"
echo "  1. Double-click 'Attach Hylo USB' on your Windows Desktop"
echo "  2. Click Yes on the UAC prompt"
echo ""
echo "If the shortcut is missing, run setup once (PowerShell as Admin):"
echo "  cd tools/scripts && powershell -ExecutionPolicy Bypass -Command \"& .\\setup-stlink-autostart.ps1\""
