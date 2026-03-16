# Attach ST-Link + common USB-serial devices to WSL2 for devcontainer (make flash, host-webapp).
# Run from Windows (PowerShell as Administrator).
# Auto-detects devices by VID:PID; binds if needed; attaches to WSL.
# Usage: .\attach-stlink.ps1
# Usage (Task Scheduler): .\attach-stlink.ps1 -Autostart
param([switch]$Autostart)
if ($Autostart) { Start-Sleep -Seconds 15 }

# ST-Link (flashing) + common USB-serial (webapp ports): CH340, FTDI, CP210x
$DeviceIds = @(
    '0483:3748', '0483:374e', '0483:374f', '0483:3754',  # ST-Link
    '1a86:7523', '1a86:5523',                             # CH340/CH341
    '0403:6001', '0403:6015',                             # FTDI
    '10c4:ea60', '10c4:ea70'                              # CP210x
)

# Require admin (usbipd bind/attach need elevation)
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "This script must run as Administrator (usbipd needs elevation)." -ForegroundColor Yellow
    Write-Host "Right-click PowerShell -> Run as Administrator, then run this script again." -ForegroundColor Yellow
    exit 1
}

# Check usbipd is installed
$usbipd = Get-Command usbipd -ErrorAction SilentlyContinue
if (-not $usbipd) {
    Write-Host "usbipd not found. Install usbipd-win from:" -ForegroundColor Red
    Write-Host "  https://github.com/dorssel/usbipd-win/releases" -ForegroundColor Red
    Write-Host "  or: winget install usbipd" -ForegroundColor Red
    exit 1
}

$list = usbipd list 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "usbipd list failed: $list" -ForegroundColor Red
    exit 1
}

$toAttach = @()
foreach ($line in ($list | Select-Object -Skip 1)) {
    foreach ($id in $DeviceIds) {
        if ($line -match "^\s*(\d+-\d+)\s+$id\s+") {
            $busId = $Matches[1]
            if ($list -notmatch "$busId\s+.*Attached") { $toAttach += $busId }
            break
        }
    }
}

if ($toAttach.Count -eq 0) {
    $attached = @()
    foreach ($line in ($list | Select-Object -Skip 1)) {
        foreach ($id in $DeviceIds) {
            if ($line -match "^\s*(\d+-\d+)\s+$id\s+.*Attached") {
                $attached += $Matches[1]
                break
            }
        }
    }
    if ($attached.Count -gt 0) {
        Write-Host "All devices already attached: $($attached -join ', ')" -ForegroundColor Green
    } else {
        Write-Host "No ST-Link or USB-serial device found. Plug in your device and try again." -ForegroundColor Yellow
        Write-Host "Looking for: ST-Link, CH340, FTDI, CP210x" -ForegroundColor Gray
    }
    exit 0
}

foreach ($busId in $toAttach) {
    if ($list -match "$busId\s+.*Not shared") {
        Write-Host "Binding ($busId)..." -ForegroundColor Cyan
        usbipd bind --busid $busId 2>&1 | Out-Null
    }
    Write-Host "Attaching ($busId) to WSL..." -ForegroundColor Cyan
    $err = usbipd attach --wsl --busid $busId 2>&1
    if ($LASTEXITCODE -ne 0) {
        if ($err -match "already attached") {
            Write-Host "  ($busId) already attached, skipping" -ForegroundColor Gray
        } else {
            Write-Host "usbipd attach failed for $busId. Check firewall (port 3240) and that WSL is running." -ForegroundColor Red
            exit 1
        }
    }
}

Write-Host "Devices attached. You can run 'make flash' and 'make host-webapp' in the devcontainer." -ForegroundColor Green
