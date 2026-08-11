<#
.SYNOPSIS
    Launch Wingflight SITL + the JSBSim bridge (+ optionally FlightGear) together.

.DESCRIPTION
    Convenience wrapper around scripts/jsbsim_bridge.py and the built SITL binary.
    Starts SITL (optionally building it first), starts the JSBSim bridge using the
    venv under tools/jsbsim-venv, and optionally launches FlightGear (fgfs.exe) as
    a visualization-only client of JSBSim's native FDM UDP output.

    Note: on a fresh/missing eeprom.bin, SITL's first launch writes the default
    config then exits (see docs/development/SITL JSBSim FlightGear Plan.md) - run
    this script once, let it exit, then run it again. Prefer keeping an existing
    eeprom.bin around for repeat runs (see sitl-rc-check.ps1 -FreshEeprom if you
    need to reset it).

.PARAMETER BuildSitl
    Build SITL (make TARGET=SITL DEBUG=GDB -j 8) before launching.

.PARAMETER Aircraft
    JSBSim aircraft model name (default: c172p).

.PARAMETER Rate
    JSBSim step/send rate in Hz (default: 120).

.PARAMETER FlightGear
    Also enable JSBSim's native FDM UDP output for FlightGear and print the
    matching fgfs launch command.

.PARAMETER FgfsPath
    Path to fgfs.exe. If given (and -FlightGear is set), FlightGear is launched
    automatically instead of just printing the command to run it manually.

.PARAMETER StopOnExit
    Stop the SITL/bridge/FlightGear child processes when this script exits.

.EXAMPLE
    .\scripts\sitl-jsbsim-flightgear-launch.ps1 -BuildSitl -FlightGear -StopOnExit

.EXAMPLE
    .\scripts\sitl-jsbsim-flightgear-launch.ps1 -FlightGear -FgfsPath "C:\FlightGear\bin\fgfs.exe" -StopOnExit
#>
param(
    [switch]$BuildSitl,
    [string]$Aircraft = "c172p",
    [double]$Rate = 120.0,
    [double]$AltitudeFt = 3000.0,
    [double]$AirspeedKts = 90.0,
    [switch]$FlightGear,
    [string]$FgfsPath = "",
    [int]$FgPort = 5550,
    [double]$FgRate = 30.0,
    [switch]$StopOnExit
)

$ErrorActionPreference = "Stop"

function Get-FirmwareRoot {
    return (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
}

$root = Get-FirmwareRoot
$objMain = Join-Path $root "obj\main"
$sitlExe = Join-Path $objMain "wingflight_SITL.elf"
$bridgePython = Join-Path $root "tools\jsbsim-venv\Scripts\python.exe"
$bridgeScript = Join-Path $root "scripts\jsbsim_bridge.py"

if ($BuildSitl) {
    Write-Host "[launch] Building SITL ..."
    Push-Location $root
    try {
        # Route through Write-Host (not the success stream) - see sitl-rc-check.ps1's
        # Build-Sitl for why a bare cmd.exe call here would be dangerous.
        cmd.exe /c "make TARGET=SITL DEBUG=GDB -j 8" | ForEach-Object { Write-Host $_ }
        if ($LASTEXITCODE -ne 0) {
            throw "SITL build failed"
        }
    } finally {
        Pop-Location
    }
}

if (-not (Test-Path $sitlExe)) {
    throw "SITL binary not found at $sitlExe - build it first (make TARGET=SITL), or pass -BuildSitl."
}
if (-not (Test-Path $bridgePython)) {
    throw "JSBSim venv not found at $bridgePython - see docs/development/SITL JSBSim FlightGear Plan.md for setup (python -m venv tools/jsbsim-venv; pip install jsbsim)."
}

Write-Host "[launch] Starting SITL ($sitlExe) ..."
$sitlProcess = Start-Process -FilePath $sitlExe -WorkingDirectory $objMain -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput (Join-Path $objMain "sitl_launch_stdout.log") `
    -RedirectStandardError (Join-Path $objMain "sitl_launch_stderr.log")
Start-Sleep -Seconds 1

$bridgeArgs = @(
    $bridgeScript,
    "--aircraft", $Aircraft,
    "--rate", $Rate,
    "--altitude-ft", $AltitudeFt,
    "--airspeed-kts", $AirspeedKts
)
if ($FlightGear) {
    $bridgeArgs += @("--flightgear", "--fg-port", $FgPort, "--fg-rate", $FgRate)
}

Write-Host "[launch] Starting JSBSim bridge ..."
$bridgeProcess = Start-Process -FilePath $bridgePython -ArgumentList $bridgeArgs -WorkingDirectory $root -PassThru -NoNewWindow

$fgProcess = $null
if ($FlightGear -and -not [string]::IsNullOrWhiteSpace($FgfsPath)) {
    if (Test-Path $FgfsPath) {
        Write-Host "[launch] Starting FlightGear ($FgfsPath) ..."
        $fgArgs = @(
            "--aircraft=$Aircraft",
            "--fdm=null",
            "--native-fdm=socket,in,$([int]$FgRate),,$FgPort,udp",
            "--disable-clouds3d"
        )
        $fgProcess = Start-Process -FilePath $FgfsPath -ArgumentList $fgArgs -PassThru
    } else {
        Write-Warning "FgfsPath '$FgfsPath' not found - skipping automatic FlightGear launch (start it manually with the command the bridge printed above)."
    }
} elseif ($FlightGear) {
    Write-Host "[launch] -FlightGear set but no -FgfsPath given - launch FlightGear manually with the command printed by the bridge above."
}

Write-Host "[launch] Running. Press Ctrl+C to stop."
try {
    while ($true) {
        Start-Sleep -Seconds 1
        if ($sitlProcess.HasExited) {
            Write-Warning "SITL process exited - this is expected on a fresh eeprom.bin's first boot (see docs/development/SITL JSBSim FlightGear Plan.md); re-run this script to connect to the now-valid config."
            break
        }
    }
} finally {
    if ($StopOnExit) {
        Write-Host "[launch] Stopping child processes ..."
        foreach ($p in @($sitlProcess, $bridgeProcess, $fgProcess)) {
            if ($p -and -not $p.HasExited) {
                Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue
            }
        }
    }
}
