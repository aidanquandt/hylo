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
$taskDesc = "Attach ST-Link to WSL2 at login so make flash works in the Hylo devcontainer."
# Add 15s delay so WSL is ready; attach-stlink.ps1 -Autostart sleeps first
$action = New-ScheduledTaskAction -Execute "powershell.exe" `
    -Argument "-NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$attachScriptFull`" -Autostart"
$trigger = New-ScheduledTaskTrigger -AtLogOn -User $env:USERNAME
$settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable

# Remove existing task if present
Unregister-ScheduledTask -TaskName $taskName -Confirm:$false -ErrorAction SilentlyContinue

Register-ScheduledTask -TaskName $taskName -Action $action -Trigger $trigger -Settings $settings `
    -Description $taskDesc -RunLevel Highest | Out-Null

Write-Host "Task '$taskName' created. ST-Link will auto-attach to WSL when you log in." -ForegroundColor Green
Write-Host "If ST-Link was already attached: usbipd detach --busid <busid>, then .\attach-stlink.ps1" -ForegroundColor Gray
Write-Host "To remove: Task Scheduler -> Task Scheduler Library -> $taskName -> Delete" -ForegroundColor Gray
