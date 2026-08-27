<#
.SYNOPSIS
    Launch Wingflight SITL + the JSBSim bridge (+ optionally FlightGear and a
    joystick RC source) together.

.DESCRIPTION
    Convenience wrapper around scripts/jsbsim_bridge.py and the built SITL binary.
    Starts SITL (optionally building it first), starts the JSBSim bridge using the
    venv under tools/jsbsim-venv, and optionally launches FlightGear (fgfs.exe) as
    a visualization-only client of JSBSim's native FDM UDP output plus
    scripts/sitl-joystick-rc.py as a live RC source.

    Note: on a fresh/missing eeprom.bin, SITL's first launch writes the default
    config then exits (see docs/development/SITL JSBSim FlightGear Plan.md) - run
    this script once, let it exit, then run it again. Prefer keeping an existing
    eeprom.bin around for repeat runs (see sitl-rc-check.ps1 -FreshEeprom if you
    need to reset it).

    Without an RC source, SITL has no RC input at all (its RX is FEATURE_RX_MSP),
    so the mixer holds failsafe values and nothing will "fly". Use -Joystick, or
    run scripts/sitl-joystick-rc.py / scripts/sitl-rc-check.ps1 separately.

.PARAMETER SetupVenv
    Create tools/jsbsim-venv (if missing) and install the pinned requirements
    from scripts/requirements-jsbsim.txt and scripts/requirements-joystick.txt,
    then continue. Safe to re-run.

.PARAMETER BuildSitl
    Build SITL (make TARGET=SITL DEBUG=GDB -j 8) before launching.

.PARAMETER Aircraft
    JSBSim aircraft model name (default: c172p).

.PARAMETER Rate
    JSBSim step/send rate in Hz (default: 120).

.PARAMETER LatDeg / LonDeg
    Initial position. Defaults to KSFO (37.6136 / -122.3572), which the
    FlightGear base package ships scenery for. JSBSim's own default is lat/lon
    0/0 - open ocean with no scenery, which renders as a blue void in FlightGear.

.PARAMETER Trim
    Trim the aircraft for steady flight at the initial conditions before running
    (otherwise it starts in an untrimmed glide).

.PARAMETER FlightGear
    Also enable JSBSim's native FDM UDP output for FlightGear and print the
    matching fgfs launch command.

.PARAMETER FgfsPath
    Path to fgfs.exe. If given (and -FlightGear is set), FlightGear is launched
    automatically instead of just printing the command to run it manually.

.PARAMETER FgExtraArgs
    Extra arguments appended verbatim to the fgfs command line.

.PARAMETER Joystick
    Also start scripts/sitl-joystick-rc.py so a USB joystick/gamepad drives RC
    over MSP. Requires pygame in the venv (-SetupVenv installs it).

.PARAMETER StopOnExit
    Stop the SITL/bridge/FlightGear/joystick child processes when this script exits.

.EXAMPLE
    .\scripts\sitl-jsbsim-flightgear-launch.ps1 -SetupVenv -BuildSitl -Trim -StopOnExit

.EXAMPLE
    .\scripts\sitl-jsbsim-flightgear-launch.ps1 -FlightGear -Joystick -Trim `
        -FgfsPath "C:\Program Files\FlightGear 2024.1\bin\fgfs.exe" -StopOnExit
#>
param(
    [switch]$SetupVenv,
    [switch]$BuildSitl,
    [string]$Aircraft = "c172p",
    [double]$Rate = 120.0,
    [double]$AltitudeFt = 3000.0,
    [double]$AirspeedKts = 90.0,
    [double]$LatDeg = 37.6136,
    [double]$LonDeg = -122.3572,
    [double]$HeadingDeg = 0.0,
    [switch]$Trim,
    [switch]$FlightGear,
    [string]$FgfsPath = "",
    [int]$FgPort = 5550,
    [double]$FgRate = 30.0,
    [string[]]$FgExtraArgs = @(),
    [switch]$Joystick,
    [switch]$StopOnExit
)

$ErrorActionPreference = "Stop"

function Get-FirmwareRoot {
    return (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
}

function Test-UdpPortInUse {
    param([int]$Port)
    # A second bridge (or a leftover Gazebo/SITL run) already holding 9002/9003
    # produces a confusing "no packets" symptom rather than a clear error, so
    # check up front.
    try {
        return (Get-NetUDPEndpoint -LocalPort $Port -ErrorAction Stop | Measure-Object).Count -gt 0
    } catch {
        return $false
    }
}

$root = Get-FirmwareRoot
$objMain = Join-Path $root "obj\main"
$sitlExe = Join-Path $objMain "wingflight_SITL.elf"
$venvDir = Join-Path $root "tools\jsbsim-venv"
$bridgePython = Join-Path $venvDir "Scripts\python.exe"
$bridgeScript = Join-Path $root "scripts\jsbsim_bridge.py"
$joystickScript = Join-Path $root "scripts\sitl-joystick-rc.py"

if ($SetupVenv) {
    if (-not (Test-Path $bridgePython)) {
        Write-Host "[launch] Creating JSBSim venv at $venvDir ..."
        $sysPython = (Get-Command python -ErrorAction SilentlyContinue)
        if (-not $sysPython) {
            throw "No 'python' on PATH - install Python 3 (3.10+) first, then re-run with -SetupVenv."
        }
        & $sysPython.Source -m venv $venvDir
        if ($LASTEXITCODE -ne 0) { throw "python -m venv failed" }
    }
    Write-Host "[launch] Installing pinned requirements into the venv ..."
    & $bridgePython -m pip install --upgrade pip | ForEach-Object { Write-Host $_ }
    & $bridgePython -m pip install -r (Join-Path $root "scripts\requirements-jsbsim.txt") | ForEach-Object { Write-Host $_ }
    if ($LASTEXITCODE -ne 0) { throw "pip install -r scripts/requirements-jsbsim.txt failed" }
    & $bridgePython -m pip install -r (Join-Path $root "scripts\requirements-joystick.txt") | ForEach-Object { Write-Host $_ }
    if ($LASTEXITCODE -ne 0) { Write-Warning "pip install -r scripts/requirements-joystick.txt failed - -Joystick will not work" }
}

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
    throw "SITL binary not found at $sitlExe - build it first (make mingw_sdk_install; make TARGET=SITL), or pass -BuildSitl."
}
if (-not (Test-Path $bridgePython)) {
    throw "JSBSim venv not found at $bridgePython - re-run this script with -SetupVenv (or see docs/development/SITL JSBSim FlightGear Plan.md)."
}

foreach ($p in @(9002, 9003)) {
    if (Test-UdpPortInUse -Port $p) {
        Write-Warning "UDP port $p is already in use - another SITL/bridge instance is probably still running. Expect 'no packets' symptoms."
    }
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
    "--airspeed-kts", $AirspeedKts,
    "--lat-deg", $LatDeg,
    "--lon-deg", $LonDeg,
    "--heading-deg", $HeadingDeg
)
if ($Trim) { $bridgeArgs += "--trim" }
if ($FlightGear) {
    $bridgeArgs += @("--flightgear", "--fg-port", $FgPort, "--fg-rate", $FgRate)
}

Write-Host "[launch] Starting JSBSim bridge ..."
$bridgeProcess = Start-Process -FilePath $bridgePython -ArgumentList $bridgeArgs -WorkingDirectory $root -PassThru -NoNewWindow

$joystickProcess = $null
if ($Joystick) {
    if (-not (Test-Path $joystickScript)) {
        Write-Warning "$joystickScript not found - skipping joystick RC."
    } else {
        Write-Host "[launch] Starting joystick RC bridge ..."
        $joystickProcess = Start-Process -FilePath $bridgePython -ArgumentList @($joystickScript) `
            -WorkingDirectory $root -PassThru
    }
} else {
    Write-Host "[launch] No RC source started (-Joystick not set). SITL's RX is MSP-only, so without one the mixer stays at failsafe."
}

$fgProcess = $null
if ($FlightGear -and -not [string]::IsNullOrWhiteSpace($FgfsPath)) {
    if (Test-Path $FgfsPath) {
        Write-Host "[launch] Starting FlightGear ($FgfsPath) ..."
        # --fdm=null: FlightGear runs no physics of its own, it only renders the
        # state JSBSim streams in. --lat/--lon/--altitude only pre-position the
        # camera/scenery load; the incoming native-FDM packets take over
        # immediately, but without them FlightGear loads scenery in the wrong
        # place (or none at all).
        $fgArgs = @(
            "--aircraft=$Aircraft",
            "--fdm=null",
            "--native-fdm=socket,in,$([int]$FgRate),,$FgPort,udp",
            "--lat=$LatDeg",
            "--lon=$LonDeg",
            "--altitude=$AltitudeFt",
            "--heading=$HeadingDeg",
            "--timeofday=noon",
            "--disable-real-weather-fetch",
            "--disable-clouds3d"
        ) + $FgExtraArgs
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
        if ($bridgeProcess.HasExited) {
            Write-Warning "JSBSim bridge exited (exit code $($bridgeProcess.ExitCode)) - see its output above."
            break
        }
        if ($fgProcess -and $fgProcess.HasExited) {
            Write-Warning "FlightGear exited."
            break
        }
    }
} finally {
    if ($StopOnExit) {
        Write-Host "[launch] Stopping child processes ..."
        foreach ($p in @($sitlProcess, $bridgeProcess, $joystickProcess, $fgProcess)) {
            if ($p -and -not $p.HasExited) {
                Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue
            }
        }
    }
}
