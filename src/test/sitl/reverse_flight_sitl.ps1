# Runs the bidirectional (4D) throttle / reverse flight checks against a native
# Windows SITL build:
#
#   make mingw_sdk_install       # once
#   make TARGET=SITL CCACHE=     # from PowerShell (needs the Windows TEMP for LTO)
#   .\src\test\sitl\reverse_flight_sitl.ps1
#
# Starts SITL three times in obj/sitl_4d: once to create eeprom.bin, once to
# apply the 4D test config through the CLI (save makes SITL exit), and once for
# the checks in reverse_flight_sitl.py. Output goes to the console and to
# obj/sitl_4d/test.txt.
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Resolve-Path (Join-Path $here "..\..\..")
$work = Join-Path $root "obj\sitl_4d"
New-Item -ItemType Directory -Force $work | Out-Null
$exe = Join-Path $work "wingflight_SITL.exe"
Copy-Item (Join-Path $root "obj\main\wingflight_SITL.elf") $exe -Force
$py = (Get-Command python).Source
$harness = Join-Path $here "reverse_flight_sitl.py"
Push-Location $work

function Start-Sitl([string]$name) {
    Start-Process -FilePath $exe -WorkingDirectory $work -PassThru -NoNewWindow `
        -RedirectStandardOutput "$name.log" -RedirectStandardError "$name.err"
}

try {
    Remove-Item -Force -ErrorAction SilentlyContinue eeprom.bin, blackbox_flash.bin
    $p = Start-Sitl "fresh"
    if (-not $p.WaitForExit(20000)) { $p.Kill() }

    foreach ($phase in "config", "test") {
        $p = Start-Sitl $phase
        Start-Sleep -Seconds 3
        & $py -u $harness $phase *> "$phase.txt"
        Start-Sleep -Seconds 2
        if (-not $p.HasExited) { $p.Kill() }
    }
    Get-Content "test.txt"
} finally {
    Pop-Location
}
