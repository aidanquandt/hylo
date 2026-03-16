# One-time setup: add WSL udev rules for ST-Link, create Windows Task for auto-attach at login.
# Run once per computer (PowerShell as Administrator).
# Requires: repo cloned, usbipd-win installed.
# Usage: .\setup-stlink-autostart.ps1

$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "Run as Administrator." -ForegroundColor Yellow
    exit 1
}

# Add udev rules in WSL for ST-Link (flashing) + common USB-serial (webapp ports)
$udevRules = @'
# ST-Link (debug + VCP)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3754", MODE="0666"
# CH340, CH341 (common USB-serial)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="5523", MODE="0666"
# FTDI
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE="0666"
# Silicon Labs CP210x
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea70", MODE="0666"
'@
$udevRules | wsl -u root tee /etc/udev/rules.d/99-stlink.rules | Out-Null
wsl -u root udevadm control --reload-rules
wsl -u root udevadm trigger
Write-Host "Added udev rules (ST-Link, CH340, FTDI, CP210x) in WSL." -ForegroundColor Cyan

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$attachScript = Join-Path $scriptDir "attach-stlink.ps1"

if (-not (Test-Path $attachScript)) {
    Write-Host "attach-stlink.ps1 not found at: $attachScript" -ForegroundColor Red
    exit 1
}

# Resolve to a path Windows can use (handles WSL \\wsl$\ paths)
$attachScriptFull = (Resolve-Path $attachScript).Path

$taskName = "Hylo-STLink-AttachWSL"
$taskDesc = "Attach ST-Link + USB-serial to WSL2 for Hylo devcontainer (make flash, host-webapp)."
# Add 15s delay so WSL is ready; attach-stlink.ps1 -Autostart sleeps first
$action = New-ScheduledTaskAction -Execute "powershell.exe" `
    -Argument "-NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$attachScriptFull`" -Autostart"
$trigger = New-ScheduledTaskTrigger -AtLogOn -User $env:USERNAME
$settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable

# Remove existing task if present
Unregister-ScheduledTask -TaskName $taskName -Confirm:$false -ErrorAction SilentlyContinue

Register-ScheduledTask -TaskName $taskName -Action $action -Trigger $trigger -Settings $settings `
    -Description $taskDesc -RunLevel Highest | Out-Null

Write-Host "Task '$taskName' created. Devices auto-attach at login." -ForegroundColor Green

# Copy script to local folder (UNC paths like \\wsl$\... often fail when run as admin)
$localScriptDir = Join-Path $env:LOCALAPPDATA "Hylo"
New-Item -ItemType Directory -Force -Path $localScriptDir | Out-Null
$localScriptPath = Join-Path $localScriptDir "attach-stlink.ps1"
Copy-Item -Path $attachScript -Destination $localScriptPath -Force
Write-Host "Copied attach script to $localScriptPath" -ForegroundColor Gray

# Create Desktop shortcut (points to local copy so it works when elevated)
$shortcutPath = Join-Path ([Environment]::GetFolderPath('Desktop')) "Attach Hylo USB.lnk"
$WshShell = New-Object -ComObject WScript.Shell
$Shortcut = $WshShell.CreateShortcut($shortcutPath)
$Shortcut.TargetPath = "powershell.exe"
$Shortcut.Arguments = "-ExecutionPolicy Bypass -NoExit -File `"$localScriptPath`""
$Shortcut.WorkingDirectory = $localScriptDir
$Shortcut.Description = "Attach ST-Link + USB-serial to WSL for Hylo devcontainer"
$Shortcut.Save()
# Set "Run as administrator" (byte 0x15 |= 0x20)
$bytes = [System.IO.File]::ReadAllBytes($shortcutPath)
$bytes[0x15] = $bytes[0x15] -bor 0x20
[System.IO.File]::WriteAllBytes($shortcutPath, $bytes)
Write-Host "Desktop shortcut 'Attach Hylo USB' created. Double-click when you plug in devices." -ForegroundColor Green

Write-Host "From the devcontainer: make attach  (launches the shortcut, UAC prompt)" -ForegroundColor Cyan
Write-Host "To remove: Task Scheduler -> $taskName; delete shortcut from Desktop" -ForegroundColor Gray
