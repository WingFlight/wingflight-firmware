<#
.SYNOPSIS
    Verify RC injection into Wingflight SITL via MSP_SET_RAW_RC.

.DESCRIPTION
    Connects to SITL MSP TCP (auto-detect 5760/5761 by default), injects RC
    channel frames with MSP_SET_RAW_RC, and confirms response via MSP_RC and
    MSP_SERVO.

    Test modes:
    - smoke: fast sanity check (single-axis)
    - sweep: roll/pitch/yaw control-surface sweep
    - stress: sustained injection/read loop with dropout tracking
    - jsbsim: end-to-end check through the JSBSim bridge (scripts/jsbsim_bridge.py)
      - starts the bridge, drives roll/pitch RC to each extreme in turn, and
        confirms MSP_ATTITUDE actually changes accordingly. Unlike the other
        modes (which only check Wingflight's own servo output), this proves
        the full loop: RC -> mixer -> servo_packet -> bridge -> JSBSim FCS ->
        physics -> fdm_packet -> fake IMU -> MSP_ATTITUDE.
      - The response is gated SIGNED against a control-neutral drift baseline
        measured over the same hold duration: roll-right (1900) minus
        roll-left (1100) must be POSITIVE (right stick = right roll) and
        exceed max(threshold, drift * margin); same for pitch (stick back =
        nose up). This distinguishes "the control worked, in the right
        direction" from "the aircraft was diverging anyway", and catches
        inverted control-surface sign conventions (fix those with the
        bridge's --invert-aileron/--invert-elevator, not the mixer).
    - throttle: end-to-end ARMED throttle check through the JSBSim bridge.
      Configures an ARM switch on AUX1 via MSP_SET_MODE_RANGE, arms (no mixer
      override active - overrides block arming), raises throttle, and
      confirms (a) MSP_MOTOR follows and (b) JSBSim actually receives the
      throttle (thr= in the bridge log) and responds (IAS). This exercises
      the one path -Mode jsbsim structurally cannot: disarmed motor output is
      forced to motor-stop regardless of RC.
    - gps: end-to-end GPS check through the JSBSim bridge's --msp-gps feed.
      The bridge pushes JSBSim's position/velocity to SITL's second MSP port
      (TCP 5762, UART2) as MSP_SET_RAW_GPS; this mode asserts the firmware
      reports a fix near the initial position that MOVES with the simulated
      aircraft (MSP_RAW_GPS). Needs SITL's config defaults (GPS provider MSP
      + UART2 MSP port) - on an older eeprom.bin run with -FreshEeprom.

.PARAMETER Port
    Explicit MSP port. If omitted or 0, the script auto-detects from
    PortCandidates.

.PARAMETER PortCandidates
    Candidate ports used for auto-detection (default: 5760, 5761).

.PARAMETER AutoStartSitl
    Start SITL automatically when no MSP port is listening.

.PARAMETER BuildSitl
    Build SITL before auto-start.

.PARAMETER Mode
    smoke | sweep | stress | jsbsim

.PARAMETER BridgeAircraft
    JSBSim aircraft model for -Mode jsbsim (default: c172p).

.PARAMETER JsbsimSettleMs
    How long to hold each RC extreme before reading MSP_ATTITUDE, in
    milliseconds, for -Mode jsbsim (default: 1500).

.PARAMETER JsbsimAttitudeThresholdDeg
    Minimum roll/pitch attitude delta (degrees) between high/low RC extremes
    to count as "responsive" for -Mode jsbsim (default: 3.0).

.EXAMPLE
    .\scripts\sitl-rc-check.ps1 -Mode smoke

.EXAMPLE
    .\scripts\sitl-rc-check.ps1 -Mode sweep -Cycles 30

.EXAMPLE
    .\scripts\sitl-rc-check.ps1 -Mode stress -StressSeconds 90

.EXAMPLE
    .\scripts\sitl-rc-check.ps1 -Mode jsbsim -AutoStartSitl -BuildSitl -StopSitlOnExit
#>
param(
    [string]$MspHost = "127.0.0.1",
    [int]$Port = 0,
    [int[]]$PortCandidates = @(5760, 5761),
    [int]$TimeoutSeconds = 3,
    [int]$Cycles = 20,
    [int]$ServoDeltaThreshold = 40,
    [ValidateSet("smoke", "sweep", "stress", "jsbsim", "throttle", "gps")]
    [string]$Mode = "smoke",
    [int]$StressSeconds = 60,
    [string]$SitlBinaryPath = "",
    [string]$SitlArgs = "",
    [string]$BridgeAircraft = "c172p",
    [int]$JsbsimSettleMs = 1500,
    [double]$JsbsimAttitudeThresholdDeg = 3.0,
    [double]$JsbsimDriftMarginFactor = 2.0,
    [int]$ThrottleTestUs = 1800,
    [int]$ThrottleHoldMs = 6000,
    [switch]$EnableRxMspIfMissing,
    [switch]$AutoStartSitl,
    [switch]$BuildSitl,
    [switch]$FreshEeprom,
    [switch]$StopSitlOnExit
)

$ErrorActionPreference = "Stop"

$MSP_API_VERSION = 1
$MSP_SET_MODE_RANGE = 35
$MSP_FEATURE_CONFIG = 36
$MSP_SET_FEATURE_CONFIG = 37
$MSP_STATUS = 101
$MSP_SERVO = 103
$MSP_MOTOR = 104
$MSP_RC = 105
$MSP_RAW_GPS = 106
$MSP_RX_CHANNELS = 114
$MSP_ATTITUDE = 108
$MSP_SET_RAW_RC = 200
$MSP_SET_MIXER_OVERRIDE = 191
$MSP_MIXER_OVERRIDE = 190

# Mixer input indices (see MIXER_IN_STABILIZED_* in src/main/pg/mixer.h) and
# override sentinel values (see MIXER_OVERRIDE_* in src/main/flight/mixer.h).
# Bench-testing roll/pitch/yaw servo response requires either arming (not
# exercised here) or forcing passthrough override on these inputs, since
# mixerSetInput() only reflects RC onto disarmed control surfaces when an
# override is active (see mixer.c).
$MIXER_IN_STABILIZED_ROLL = 1
$MIXER_IN_STABILIZED_PITCH = 2
$MIXER_IN_STABILIZED_YAW = 3
$MIXER_OVERRIDE_OFF = 2501
$MIXER_OVERRIDE_PASSTHROUGH = 2502

$script:StartedSitlProcess = $null
$script:StartedBridgeProcess = $null
$script:ResolvedPort = 0

function Get-FirmwareRoot {
    return (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
}

function Get-WorkspaceRoot {
    return (Resolve-Path (Join-Path (Get-FirmwareRoot) "..")).Path
}

function Test-TcpPort {
    param(
        [string]$RemoteHost,
        [int]$ProbePort,
        [int]$TimeoutMs = 400
    )

    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $task = $client.ConnectAsync($RemoteHost, $ProbePort)
        if (-not $task.Wait($TimeoutMs)) {
            return $false
        }
        return $client.Connected
    } catch {
        return $false
    } finally {
        $client.Close()
    }
}

function Resolve-MspPort {
    param(
        [string]$RemoteHost,
        [int]$ExplicitPort,
        [int[]]$Candidates
    )

    # SITL's TCP MSP server (dyad-based) only accepts one client connection
    # at a time. This function probes with short-lived connections
    # (Test-TcpPort / Test-MspApiPort) that connect and immediately
    # disconnect; the server needs a brief moment to notice the disconnect
    # before it will accept a *new* connection (e.g. the caller's own,
    # persistent one). Without this settle delay, the caller's connection can
    # arrive while the server still considers the probe connection active,
    # and gets silently rejected - the client sees a successful TCP connect
    # but never receives any MSP response.
    $settleMs = 400

    if ($ExplicitPort -gt 0) {
        if (Test-TcpPort -RemoteHost $RemoteHost -ProbePort $ExplicitPort) {
            Start-Sleep -Milliseconds $settleMs
            return $ExplicitPort
        }
        return 0
    }

    $firstOpen = 0
    foreach ($p in $Candidates) {
        if (Test-TcpPort -RemoteHost $RemoteHost -ProbePort $p) {
            if ($firstOpen -eq 0) { $firstOpen = $p }
            if (Test-MspApiPort -RemoteHost $RemoteHost -ProbePort $p -TimeoutSeconds 1) {
                Start-Sleep -Milliseconds $settleMs
                return $p
            }
        }
    }

    if ($firstOpen -gt 0) {
        Start-Sleep -Milliseconds $settleMs
        return $firstOpen
    }

    return 0
}

function Get-SitlBinaryCandidates {
    param([string]$ExplicitPath = "")

    if (-not [string]::IsNullOrWhiteSpace($ExplicitPath) -and (Test-Path $ExplicitPath)) {
        return @($ExplicitPath)
    }

    $root = Get-FirmwareRoot
    $workspaceRoot = Get-WorkspaceRoot
    # .elf FIRST: that is the name `make TARGET=SITL` actually produces (a native
    # PE despite the extension). A stale hand-built wingflight_SITL.exe left in
    # obj/main otherwise shadows every fresh build, and the resulting "checks
    # failed" is impossible to read - it is testing month-old firmware.
    $elf = Join-Path $root "obj\main\wingflight_SITL.elf"
    $exe = Join-Path $root "obj\main\wingflight_SITL.exe"
    if ((Test-Path $elf) -and (Test-Path $exe) -and
        ((Get-Item $exe).LastWriteTime -lt (Get-Item $elf).LastWriteTime)) {
        Write-Warning "Ignoring stale $exe (older than the built .elf) - delete it to silence this."
    }

    return @(
        $elf,
        $exe,
        (Join-Path $root "obj\main\betaflight_SITL.elf"),
        (Join-Path $root "obj\main\betaflight_SITL.exe"),
        (Join-Path $workspaceRoot "inav-configurator\resources\public\sitl\windows\inav_SITL.exe")
    )
}

function Build-Sitl {
    $root = Get-FirmwareRoot
    Push-Location $root
    try {
        Write-Host "[SITL-RC] Building SITL ..."
        # Route through Write-Host (not the success stream) so this output can't leak into
        # this function's return value and contaminate callers expecting a plain int/string.
        cmd.exe /c "make TARGET=SITL DEBUG=GDB -j 8" | ForEach-Object { Write-Host $_ }
        if ($LASTEXITCODE -ne 0) {
            throw "SITL build failed"
        }
    } finally {
        Pop-Location
    }
}

function Start-SitlIfNeeded {
    param(
        [string]$RemoteHost,
        [int]$ExplicitPort,
        [int[]]$Candidates,
        [bool]$DoBuild,
        [string]$Arguments
    )

    $readyPort = Resolve-MspPort -RemoteHost $RemoteHost -ExplicitPort $ExplicitPort -Candidates $Candidates
    if ($readyPort -gt 0) {
        return $readyPort
    }

    if (-not $AutoStartSitl) {
        throw "No SITL MSP listener found on candidate ports: $($Candidates -join ', ')"
    }

    if ($DoBuild) {
        Build-Sitl | Out-Null
    }

    $binary = Get-SitlBinaryCandidates -ExplicitPath $SitlBinaryPath | Where-Object { Test-Path $_ } | Select-Object -First 1
    if ([string]::IsNullOrWhiteSpace($binary)) {
        throw "No SITL binary found. Build first with 'make TARGET=SITL' or provide -SitlBinaryPath."
    }

    $root = Get-FirmwareRoot
    $logPath = Join-Path $root "obj\main\wingflight_SITL.log"
    try { Remove-Item -Force $logPath -ErrorAction SilentlyContinue } catch { }

    $launchArgs = $Arguments

    $binaryDir = Split-Path $binary -Parent
    if ([string]::IsNullOrWhiteSpace($binaryDir) -or -not (Test-Path $binaryDir)) {
        $binaryDir = $root
    }

    if ($FreshEeprom) {
        # SITL parses NO command-line arguments (target.c opens the hardcoded
        # EEPROM_FILENAME "eeprom.bin" in its working directory), so the old
        # "--path=..." approach was silently ignored and -FreshEeprom never
        # actually freshened anything. Move the real file aside instead; the
        # first boot then writes defaults and exits, which the 2-attempt
        # start loop below already handles.
        $eepromPath = Join-Path $binaryDir "eeprom.bin"
        if (Test-Path $eepromPath) {
            Write-Host "[SITL-RC] -FreshEeprom: moving $eepromPath aside (-> eeprom.bin.bak)"
            try {
                Move-Item -Force $eepromPath "$eepromPath.bak"
            } catch {
                Write-Warning "[SITL-RC] Could not move eeprom.bin aside ($_) - continuing with the existing config"
            }
        }
    }

    $attemptErrors = @()
    foreach ($attempt in 1..2) {
        Write-Host "[SITL-RC] Starting SITL (attempt $attempt/2): $binary"

        if ($IsWindows) {
            $binaryFullPath = (Resolve-Path $binary).Path
            if ([string]::IsNullOrWhiteSpace($launchArgs)) {
                $script:StartedSitlProcess = Start-Process -FilePath $binaryFullPath -WorkingDirectory $binaryDir -PassThru
            } else {
                $script:StartedSitlProcess = Start-Process -FilePath $binaryFullPath -ArgumentList $launchArgs -WorkingDirectory $binaryDir -PassThru
            }
        } else {
            if ([string]::IsNullOrWhiteSpace($launchArgs)) {
                $script:StartedSitlProcess = Start-Process -FilePath $binary -WorkingDirectory $binaryDir -PassThru -RedirectStandardOutput $logPath -RedirectStandardError "$logPath.err"
            } else {
                $script:StartedSitlProcess = Start-Process -FilePath $binary -ArgumentList $launchArgs -WorkingDirectory $binaryDir -PassThru -RedirectStandardOutput $logPath -RedirectStandardError "$logPath.err"
            }
        }

        $deadline = [DateTime]::UtcNow.AddSeconds(20)
        $startupError = ""
        while ([DateTime]::UtcNow -lt $deadline) {
            if ($script:StartedSitlProcess.HasExited) {
                $startupError = "process exited early with code $($script:StartedSitlProcess.ExitCode)"
                break
            }
            $readyPort = Resolve-MspPort -RemoteHost $RemoteHost -ExplicitPort $ExplicitPort -Candidates $Candidates
            if ($readyPort -gt 0) {
                return $readyPort
            }
            Start-Sleep -Milliseconds 150
        }

        if ([string]::IsNullOrWhiteSpace($startupError)) {
            $startupError = "listener was not detected in startup window"
        }

        $attemptErrors += "attempt ${attempt}: $startupError"

        try {
            if ($null -ne $script:StartedSitlProcess -and -not $script:StartedSitlProcess.HasExited) {
                Stop-Process -Id $script:StartedSitlProcess.Id -Force -ErrorAction SilentlyContinue
            }
        } catch { }

        Start-Sleep -Milliseconds 300
    }

    throw "SITL startup failed after retries: $($attemptErrors -join '; ')"
}

function Send-Msp {
    param(
        [System.IO.Stream]$Stream,
        [int]$Command,
        [byte[]]$Payload = @()
    )

    $len = $Payload.Length
    $crc = $len -bxor $Command
    foreach ($b in $Payload) { $crc = $crc -bxor $b }

    $frame = New-Object System.Collections.Generic.List[byte]
    $frame.Add(0x24) | Out-Null
    $frame.Add(0x4D) | Out-Null
    $frame.Add(0x3C) | Out-Null
    $frame.Add([byte]$len) | Out-Null
    $frame.Add([byte]$Command) | Out-Null
    foreach ($b in $Payload) { $frame.Add($b) | Out-Null }
    $frame.Add([byte]($crc -band 0xFF)) | Out-Null

    $arr = $frame.ToArray()
    $Stream.Write($arr, 0, $arr.Length)
    $Stream.Flush()
}

function Get-FirstMspFrame {
    param([System.Collections.Generic.List[byte]]$Bytes)

    for ($s = 0; $s -le $Bytes.Count - 6; $s++) {
        if ($Bytes[$s] -ne 0x24) { continue }
        if ($Bytes[$s + 1] -ne 0x4D) { continue }

        $dir = $Bytes[$s + 2]
        if ($dir -ne 0x3E -and $dir -ne 0x21) { continue }

        $len = $Bytes[$s + 3]
        $cmd = $Bytes[$s + 4]
        $end = $s + 5 + $len

        if ($Bytes.Count -le $end) {
            if ($s -gt 0) { $Bytes.RemoveRange(0, $s) }
            return $null
        }

        $payload = New-Object byte[] $len
        for ($i = 0; $i -lt $len; $i++) { $payload[$i] = $Bytes[$s + 5 + $i] }

        $crc = $len -bxor $cmd
        foreach ($b in $payload) { $crc = $crc -bxor $b }

        return [pscustomobject]@{
            EndIndex = $end
            Command = $cmd
            Length = $len
            Payload = $payload
            CrcOk = ($crc -eq $Bytes[$end])
            IsError = ($dir -eq 0x21)
        }
    }

    if ($Bytes.Count -gt 1) { $Bytes.RemoveRange(0, $Bytes.Count - 1) }
    return $null
}

function Receive-MspMatch {
    param(
        [System.IO.Stream]$Stream,
        [int]$ExpectedCommand,
        [int]$TimeoutSeconds
    )

    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    $buffer = New-Object System.Collections.Generic.List[byte]
    $temp = New-Object byte[] 1024

    while ([DateTime]::UtcNow -lt $deadline) {
        if ($Stream.DataAvailable) {
            $read = $Stream.Read($temp, 0, $temp.Length)
            for ($i = 0; $i -lt $read; $i++) { $buffer.Add($temp[$i]) }

            while ($true) {
                $frame = Get-FirstMspFrame -Bytes $buffer
                if ($null -eq $frame) { break }
                $buffer.RemoveRange(0, $frame.EndIndex + 1)
                if ($frame.CrcOk -and $frame.Command -eq $ExpectedCommand) {
                    return $frame
                }
            }
        } else {
            Start-Sleep -Milliseconds 5
        }
    }

    return $null
}

function Clear-MspInput {
    param([System.IO.Stream]$Stream)

    $temp = New-Object byte[] 1024
    $drained = 0
    while ($Stream.DataAvailable) {
        $read = $Stream.Read($temp, 0, $temp.Length)
        if ($read -le 0) { break }
        $drained += $read
        if ($drained -gt 65536) { break }
    }
}

function Test-MspApiPort {
    param(
        [string]$RemoteHost,
        [int]$ProbePort,
        [int]$TimeoutSeconds = 1
    )

    $client = New-Object System.Net.Sockets.TcpClient
    $stream = $null
    try {
        $task = $client.ConnectAsync($RemoteHost, $ProbePort)
        if (-not $task.Wait([TimeSpan]::FromSeconds($TimeoutSeconds))) {
            return $false
        }

        $stream = $client.GetStream()
        Send-Msp -Stream $stream -Command $MSP_API_VERSION
        $api = Receive-MspMatch -Stream $stream -ExpectedCommand $MSP_API_VERSION -TimeoutSeconds $TimeoutSeconds
        return ($null -ne $api -and $api.Length -ge 3)
    } catch {
        return $false
    } finally {
        if ($null -ne $stream) { $stream.Dispose() }
        $client.Close()
    }
}

function Set-MixerOverride {
    param(
        [System.IO.Stream]$Stream,
        [int]$Index,
        [int]$Value,
        [int]$TimeoutSeconds
    )

    $payload = [byte[]]@([byte]$Index, [byte]($Value -band 0xFF), [byte](($Value -shr 8) -band 0xFF))
    Send-Msp -Stream $Stream -Command $MSP_SET_MIXER_OVERRIDE -Payload $payload
    $null = Receive-MspMatch -Stream $Stream -ExpectedCommand $MSP_SET_MIXER_OVERRIDE -TimeoutSeconds $TimeoutSeconds
}

function Enable-RpyPassthroughOverride {
    param([System.IO.Stream]$Stream, [int]$TimeoutSeconds)

    foreach ($idx in @($MIXER_IN_STABILIZED_ROLL, $MIXER_IN_STABILIZED_PITCH, $MIXER_IN_STABILIZED_YAW)) {
        Set-MixerOverride -Stream $Stream -Index $idx -Value $MIXER_OVERRIDE_PASSTHROUGH -TimeoutSeconds $TimeoutSeconds
    }
}

function Disable-RpyPassthroughOverride {
    param([System.IO.Stream]$Stream, [int]$TimeoutSeconds)

    foreach ($idx in @($MIXER_IN_STABILIZED_ROLL, $MIXER_IN_STABILIZED_PITCH, $MIXER_IN_STABILIZED_YAW)) {
        Set-MixerOverride -Stream $Stream -Index $idx -Value $MIXER_OVERRIDE_OFF -TimeoutSeconds $TimeoutSeconds
    }
}

function New-RcPayload {
    param(
        [int]$Roll,
        [int]$Pitch,
        [int]$Yaw,
        [int]$Collective = 1500,
        [int]$Throttle,
        [int]$Aux1 = 1000,
        [int]$Aux2 = 1000,
        [int]$Aux3 = 1000
    )

    # Payload index order must match what SITL's default rcmap ("AETR1234",
    # see parseRcChannels() in src/main/rx/rx.c) resolves to: index0=Roll,
    # 1=Pitch, 2=Throttle, 3=Yaw, 4+=AUX. "Collective" is a vestigial
    # heli-only parameter (kept only for call-site compatibility) with no
    # fixed-wing channel of its own, so it's parked on an unused AUX slot
    # instead of stomping Yaw/Throttle like the old (Roll,Pitch,Yaw,
    # Collective,Throttle,...) order used to.
    $channels = @($Roll, $Pitch, $Throttle, $Yaw, $Aux1, $Aux2, $Aux3, $Collective)
    while ($channels.Count -lt 18) { $channels += 1500 }

    $payload = New-Object System.Collections.Generic.List[byte]
    foreach ($c in $channels) {
        $payload.Add([byte]($c -band 0xFF)) | Out-Null
        $payload.Add([byte](($c -shr 8) -band 0xFF)) | Out-Null
    }
    return $payload.ToArray()
}

function UInt32-ToBytesLE {
    param([uint32]$Value)
    return [BitConverter]::GetBytes($Value)
}

function ConvertTo-UInt16Array {
    param([byte[]]$Data)
    $count = [math]::Floor($Data.Length / 2)
    $values = New-Object 'System.Collections.Generic.List[int]'
    for ($i = 0; $i -lt $count; $i++) {
        $values.Add([BitConverter]::ToUInt16($Data, $i * 2)) | Out-Null
    }
    return , $values.ToArray()
}

function Get-MaxDelta {
    param([int[]]$A, [int[]]$B)
    $n = [math]::Min($A.Length, $B.Length)
    $max = 0
    for ($i = 0; $i -lt $n; $i++) {
        $d = [math]::Abs($A[$i] - $B[$i])
        if ($d -gt $max) { $max = $d }
    }
    return $max
}

function Reconnect-Msp {
    param([int]$TimeoutSeconds)

    # The SITL TCP MSP server has been observed to unexpectedly close an
    # otherwise-healthy, actively-used connection partway through a test run
    # (root cause not fully understood - possibly a Windows-loopback-TCP
    # specific quirk rather than a firmware bug). When that happens, the
    # firmware silently drops any reply it generates for requests received
    # after the close, so callers see a bare read timeout with no other
    # symptom. Transparently reconnecting and retrying once is far cheaper
    # than root-causing the disconnect and keeps the test meaningful.
    try { if ($null -ne $script:stream) { $script:stream.Dispose() } } catch { }
    try { if ($null -ne $script:client) { $script:client.Close() } } catch { }
    $script:stream = $null
    $script:client = $null

    for ($attempt = 1; $attempt -le 3; $attempt++) {
        try {
            $newClient = New-Object System.Net.Sockets.TcpClient
            $connectTask = $newClient.ConnectAsync($MspHost, $script:ResolvedPort)
            if (-not $connectTask.Wait([TimeSpan]::FromSeconds($TimeoutSeconds))) {
                $newClient.Close()
                continue
            }
            $newStream = $newClient.GetStream()
            Send-Msp -Stream $newStream -Command $MSP_API_VERSION
            $api = Receive-MspMatch -Stream $newStream -ExpectedCommand $MSP_API_VERSION -TimeoutSeconds $TimeoutSeconds
            if ($null -ne $api -and $api.Length -ge 3) {
                $script:client = $newClient
                $script:stream = $newStream
                Write-Host "[SITL-RC] Reconnected after unexpected disconnect"
                return $true
            }
            $newStream.Dispose()
            $newClient.Close()
        } catch { }
        Start-Sleep -Milliseconds 300
    }
    return $false
}

function Invoke-DriveAndRead {
    param(
        [System.IO.Stream]$Stream,
        [int]$Roll,
        [int]$Pitch,
        [int]$Yaw,
        [int]$Collective = 1500,
        [int]$Throttle,
        [int]$ReadCommand,
        [int[]]$ReadCommands = @(),
        [int]$Cycles,
        [int]$TimeoutSeconds
    )

    $rc = New-RcPayload -Roll $Roll -Pitch $Pitch -Yaw $Yaw -Collective $Collective -Throttle $Throttle
    $commandsToRead = if ($ReadCommands.Count -gt 0) { $ReadCommands } else { @($ReadCommand) }
    $last = $null

    for ($retry = 0; $retry -le 2; $retry++) {
        $activeStream = $script:stream
        if ($retry -eq 0 -and $null -ne $Stream) { $activeStream = $Stream }

        # A previous call may have exhausted its reconnect attempts and left
        # no usable connection behind. Reconnect here rather than handing a
        # null stream to Send-Msp/Clear-MspInput, which would crash the script.
        if ($null -eq $activeStream) {
            if (-not (Reconnect-Msp -TimeoutSeconds $TimeoutSeconds)) { break }
            $activeStream = $script:stream
        }

        # Prevent stale responses from previous phases from masking current RC changes.
        Clear-MspInput -Stream $activeStream

        for ($k = 0; $k -lt $Cycles; $k++) {
            Send-Msp -Stream $activeStream -Command $MSP_SET_RAW_RC -Payload $rc
            Start-Sleep -Milliseconds 12
        }

        # Allow RX task processing before sampling channel state.
        Start-Sleep -Milliseconds 35

        foreach ($cmd in $commandsToRead) {
            Send-Msp -Stream $activeStream -Command $cmd
            $resp = Receive-MspMatch -Stream $activeStream -ExpectedCommand $cmd -TimeoutSeconds $TimeoutSeconds
            if ($null -ne $resp) {
                $resp | Add-Member -NotePropertyName ResponseCommand -NotePropertyValue $cmd -Force
                $last = $resp
                break
            }
        }

        if ($null -ne $last -or $retry -eq 2) {
            break
        }
        if (-not (Reconnect-Msp -TimeoutSeconds $TimeoutSeconds)) {
            break
        }
    }

    return $last
}

function Get-ActiveStream {
    # Resolves a usable stream for a call site, falling back to the shared
    # reconnected stream (or reconnecting outright) when the caller's own
    # reference has gone stale/null after a prior unexpected disconnect.
    # This avoids crashing the whole script with a null-stream method call.
    param(
        [System.IO.Stream]$Stream,
        [int]$TimeoutSeconds
    )

    $activeStream = $Stream
    if ($null -eq $activeStream) { $activeStream = $script:stream }
    if ($null -eq $activeStream) {
        if (-not (Reconnect-Msp -TimeoutSeconds $TimeoutSeconds)) { return $null }
        $activeStream = $script:stream
    }
    return $activeStream
}

function Send-RcWithAck {
    param(
        [System.IO.Stream]$Stream,
        [int]$Roll,
        [int]$Pitch,
        [int]$Yaw,
        [int]$Collective = 1500,
        [int]$Throttle,
        [int]$Aux1 = 1000,
        [int]$TimeoutSeconds,
        [int]$Retries = 2
    )

    $rc = New-RcPayload -Roll $Roll -Pitch $Pitch -Yaw $Yaw -Collective $Collective -Throttle $Throttle -Aux1 $Aux1
    $forceReconnect = $false
    for ($attempt = 0; $attempt -le $Retries; $attempt++) {
        # A dead-but-not-yet-null connection (server closed, client object
        # still referenced) will silently fail every call on it, so once an
        # attempt fails, force a fresh reconnect before retrying rather than
        # hammering the same broken stream.
        if ($forceReconnect) {
            if (-not (Reconnect-Msp -TimeoutSeconds $TimeoutSeconds)) { return $false }
        }
        $activeStream = Get-ActiveStream -Stream $(if ($forceReconnect) { $null } else { $Stream }) -TimeoutSeconds $TimeoutSeconds
        if ($null -eq $activeStream) { return $false }
        Clear-MspInput -Stream $activeStream
        Send-Msp -Stream $activeStream -Command $MSP_SET_RAW_RC -Payload $rc
        $ack = Receive-MspMatch -Stream $activeStream -ExpectedCommand $MSP_SET_RAW_RC -TimeoutSeconds $TimeoutSeconds
        if ($null -ne $ack -and -not $ack.IsError) {
            return $true
        }
        $forceReconnect = $true
    }
    return $false
}

function Set-Neutral {
    param([System.IO.Stream]$Stream)

    $activeStream = Get-ActiveStream -Stream $Stream -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $activeStream) { return }

    $neutral = New-RcPayload -Roll 1500 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000
    for ($k = 0; $k -lt 10; $k++) {
        Send-Msp -Stream $activeStream -Command $MSP_SET_RAW_RC -Payload $neutral
        Start-Sleep -Milliseconds 8
    }

    Disable-RpyPassthroughOverride -Stream $activeStream -TimeoutSeconds $TimeoutSeconds
}

function Get-AngleDeltaDeg {
    # Signed shortest-path delta between two angles in degrees, wrapped into
    # [-180, 180]. Needed for yaw since MSP_ATTITUDE reports yaw as 0-360.
    param([double]$A, [double]$B)
    $d = $A - $B
    while ($d -gt 180) { $d -= 360 }
    while ($d -lt -180) { $d += 360 }
    return $d
}

function Get-Attitude {
    # Reads MSP_ATTITUDE (roll/pitch in 0.1 deg, yaw in whole deg). Returns
    # $null on failure/timeout so callers can treat it like any other MSP read.
    param([System.IO.Stream]$Stream, [int]$TimeoutSeconds)

    $activeStream = Get-ActiveStream -Stream $Stream -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $activeStream) { return $null }

    Send-Msp -Stream $activeStream -Command $MSP_ATTITUDE
    $resp = Receive-MspMatch -Stream $activeStream -ExpectedCommand $MSP_ATTITUDE -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $resp -or $resp.Payload.Length -lt 6) { return $null }

    return [pscustomobject]@{
        Roll  = [BitConverter]::ToInt16($resp.Payload, 0) / 10.0
        Pitch = [BitConverter]::ToInt16($resp.Payload, 2) / 10.0
        Yaw   = [double][BitConverter]::ToInt16($resp.Payload, 4)
    }
}

function Hold-RcAndReadAttitude {
    # Continuously refreshes an RC frame for DurationMs (so RX signal timeout
    # / failsafe never kicks in mid-hold) and samples MSP_ATTITUDE at the end,
    # once JSBSim's physics have had time to settle into the new control
    # deflection. Returns $null on total failure, otherwise a pscustomobject
    # with Roll/Pitch/Yaw (deg) plus AckOk (whether every RC frame was acked).
    param(
        [System.IO.Stream]$Stream,
        [int]$Roll,
        [int]$Pitch,
        [int]$Yaw,
        [int]$Throttle = 1000,
        [int]$DurationMs,
        [int]$TimeoutSeconds
    )

    $deadline = [DateTime]::UtcNow.AddMilliseconds($DurationMs)
    $allAcked = $true
    while ([DateTime]::UtcNow -lt $deadline) {
        $ok = Send-RcWithAck -Stream $Stream -Roll $Roll -Pitch $Pitch -Yaw $Yaw -Collective 1500 -Throttle $Throttle -TimeoutSeconds $TimeoutSeconds -Retries 1
        if (-not $ok) { $allAcked = $false }
        Start-Sleep -Milliseconds 15
    }

    $att = Get-Attitude -Stream $Stream -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $att) { return $null }
    $att | Add-Member -NotePropertyName AckOk -NotePropertyValue $allAcked -Force
    return $att
}

function Start-JsbsimBridge {
    # Launches scripts/jsbsim_bridge.py under the project's JSBSim venv so
    # -Mode jsbsim can validate the full RC -> mixer -> servo -> bridge ->
    # JSBSim -> fdm_packet -> fake IMU -> MSP_ATTITUDE loop, not just
    # Wingflight's own servo output. Returns the started process, or $null if
    # the venv isn't present (caller should treat that as a hard failure for
    # this mode, not silently skip the check).
    param([string]$Aircraft, [switch]$MspGps)

    $root = Get-FirmwareRoot
    $bridgePython = Join-Path $root "tools\jsbsim-venv\Scripts\python.exe"
    $bridgeScript = Join-Path $root "scripts\jsbsim_bridge.py"

    if (-not (Test-Path $bridgePython) -or -not (Test-Path $bridgeScript)) {
        Write-Host "[SITL-RC] JSBSim bridge venv/script not found (expected $bridgePython) - see docs/development/SITL JSBSim FlightGear Plan.md"
        return $null
    }

    $logDir = Join-Path $root "obj\main"
    if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir -Force | Out-Null }

    $bridgeArgs = @($bridgeScript, "--aircraft", $Aircraft, "--trim")
    if ($MspGps) { $bridgeArgs += "--msp-gps" }

    Write-Host "[SITL-RC] Starting JSBSim bridge (aircraft=$Aircraft) ..."
    $p = Start-Process -FilePath $bridgePython `
        -ArgumentList $bridgeArgs `
        -WorkingDirectory $root `
        -PassThru -NoNewWindow `
        -RedirectStandardOutput (Join-Path $logDir "jsbsim_bridge_check_stdout.log") `
        -RedirectStandardError (Join-Path $logDir "jsbsim_bridge_check_stderr.log")

    # Give JSBSim time to parse the aircraft model and open its sockets
    # before we start relying on attitude responses.
    Start-Sleep -Seconds 2
    return $p
}

function Get-ArmingStatus {
    # Reads MSP_STATUS and extracts what the throttle mode needs: whether the
    # FC is armed (flight-mode flags bit 0 = BOXARM) and the arming-disable
    # flag mask (see runtime_config.h). Returns $null on failure.
    # MSP_STATUS payload offsets (see msp.c): [0]U16 pid dt, [2]U16 gyro dt,
    # [4]U16 sensors, [6]U32 flight mode flags, [10]U8 profile, [11]U16 max
    # load, [13]U16 avg load, [15]U8 extra-flags count (0), [16]U8 arming
    # disable flag count, [17]U32 arming disable flags.
    param([System.IO.Stream]$Stream, [int]$TimeoutSeconds)

    $activeStream = Get-ActiveStream -Stream $Stream -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $activeStream) { return $null }

    Send-Msp -Stream $activeStream -Command $MSP_STATUS
    $resp = Receive-MspMatch -Stream $activeStream -ExpectedCommand $MSP_STATUS -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $resp -or $resp.Payload.Length -lt 21) { return $null }

    return [pscustomobject]@{
        Armed              = (([BitConverter]::ToUInt32($resp.Payload, 6) -band 1) -ne 0)
        ArmingDisableFlags = [BitConverter]::ToUInt32($resp.Payload, 17)
    }
}

# Arming-disable flag names, bit position = array index (runtime_config.h).
$ArmingDisableFlagNames = @(
    "NO_GYRO", "FAILSAFE", "RX_FAILSAFE", "BAD_RX_RECOVERY", "BOXFAILSAFE",
    "GOVERNOR", "RPM_SIGNAL", "THROTTLE", "ANGLE", "BOOT_GRACE_TIME",
    "NOPREARM", "LOAD", "CALIBRATING", "CLI", "CMS_MENU", "BST", "MSP",
    "PARALYZE", "GPS", "RESC", "RPMFILTER", "REBOOT_REQUIRED",
    "DSHOT_BITBANG", "ACC_CALIBRATION", "MOTOR_PROTOCOL", "OVERRIDE",
    "ARM_SWITCH")

function Format-ArmingDisableFlags {
    param([uint32]$Flags)
    $names = @()
    for ($b = 0; $b -lt $ArmingDisableFlagNames.Count; $b++) {
        if (($Flags -band (1 -shl $b)) -ne 0) { $names += $ArmingDisableFlagNames[$b] }
    }
    if ($names.Count -eq 0) { return "(none)" }
    return $names -join "|"
}

function Get-ArmingStatusWithRc {
    # Like Get-ArmingStatus, but pipelines an RC frame immediately before the
    # MSP_STATUS request. SITL's MSP-RX signal timeout is short enough that a
    # bare status round-trip (no RC frames going out while waiting) can drop
    # RX - and an RX drop that recovers while the ARM switch is high latches
    # ARMING_DISABLED_BAD_RX_RECOVERY until the switch goes low again, making
    # a naive poll loop sabotage the very arming it is checking.
    param(
        [System.IO.Stream]$Stream,
        [int]$Throttle = 1000,
        [int]$Aux1 = 1000,
        [int]$TimeoutSeconds
    )

    $activeStream = Get-ActiveStream -Stream $Stream -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $activeStream) { return $null }

    Clear-MspInput -Stream $activeStream
    $rc = New-RcPayload -Roll 1500 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle $Throttle -Aux1 $Aux1
    Send-Msp -Stream $activeStream -Command $MSP_SET_RAW_RC -Payload $rc
    Send-Msp -Stream $activeStream -Command $MSP_STATUS
    $resp = Receive-MspMatch -Stream $activeStream -ExpectedCommand $MSP_STATUS -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $resp -or $resp.Payload.Length -lt 21) { return $null }

    return [pscustomobject]@{
        Armed              = (([BitConverter]::ToUInt32($resp.Payload, 6) -band 1) -ne 0)
        ArmingDisableFlags = [BitConverter]::ToUInt32($resp.Payload, 17)
    }
}

function Hold-RcFrames {
    # Streams a fixed RC frame (including AUX1, so an ARM switch can be held)
    # for DurationMs, FIRE-AND-FORGET: no per-frame ack wait. PowerShell's
    # overhead on an ack round trip can spike past SITL's 100ms MSP-RX signal
    # timeout (rxFrameCheck), which made every ack-checked hold drop RX
    # mid-stream - and an RX drop that recovers while the ARM switch is high
    # latches ARMING_DISABLED_BAD_RX_RECOVERY, sabotaging the arming this
    # mode is trying to test. Replies are drained periodically so the TCP
    # buffer never fills. Returns $true if a usable stream existed.
    param(
        [System.IO.Stream]$Stream,
        [int]$Roll = 1500,
        [int]$Pitch = 1500,
        [int]$Yaw = 1500,
        [int]$Throttle = 1000,
        [int]$Aux1 = 1000,
        [int]$DurationMs,
        [int]$TimeoutSeconds
    )

    $activeStream = Get-ActiveStream -Stream $Stream -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $activeStream) { return $false }

    $rc = New-RcPayload -Roll $Roll -Pitch $Pitch -Yaw $Yaw -Collective 1500 -Throttle $Throttle -Aux1 $Aux1
    $deadline = [DateTime]::UtcNow.AddMilliseconds($DurationMs)
    $n = 0
    while ([DateTime]::UtcNow -lt $deadline) {
        Send-Msp -Stream $activeStream -Command $MSP_SET_RAW_RC -Payload $rc
        if (((++$n) % 8) -eq 0) { Clear-MspInput -Stream $activeStream }
        Start-Sleep -Milliseconds 10
    }
    Clear-MspInput -Stream $activeStream
    return $true
}

function Get-BridgeStatusSamples {
    # Parses the JSBSim bridge's status lines from its redirected stdout log,
    # returning the numeric fields of the last -Tail lines as objects:
    # T (sim s), Alt (ft), Ias (kts), Thr/Ail/Ele/Rud (norm), PacketsRx.
    param([int]$Tail = 50)

    $root = Get-FirmwareRoot
    $log = Join-Path $root "obj\main\jsbsim_bridge_check_stdout.log"
    if (-not (Test-Path $log)) { return @() }

    $samples = @()
    # -ReadCount 0 + a shared read handle: the bridge holds the file open, so
    # read defensively and tolerate a partially-written last line.
    try { $lines = Get-Content $log -Tail $Tail -ErrorAction Stop } catch { return @() }
    foreach ($line in $lines) {
        if ($line -match 't=\s*([\d.]+)s\s+alt=\s*(-?[\d.]+)ft\s+ias=\s*([\d.]+)kts\s+thr=([\d.]+)\s+ail=([+\-][\d.]+)\s+ele=([+\-][\d.]+)\s+rud=([+\-][\d.]+)\s+packets_rx=(\d+)') {
            $samples += [pscustomobject]@{
                T = [double]$Matches[1]; Alt = [double]$Matches[2]; Ias = [double]$Matches[3]
                Thr = [double]$Matches[4]; Ail = [double]$Matches[5]; Ele = [double]$Matches[6]
                Rud = [double]$Matches[7]; PacketsRx = [int]$Matches[8]
            }
        }
    }
    return $samples
}

$result = [ordered]@{
    mode = $Mode
    connected = $false
    port = 0
    apiOk = $false
    featureMask = 0
    featureRxMspEnabled = $false
    rcInjectOk = $false
    controlResponsive = $false
    stressDropouts = 0
    stressCycles = 0
    axisDeltas = @{}
}

$client = $null
$stream = $null

try {
    $script:ResolvedPort = Start-SitlIfNeeded -RemoteHost $MspHost -ExplicitPort $Port -Candidates $PortCandidates -DoBuild ([bool]$BuildSitl) -Arguments $SitlArgs
    $result.port = $script:ResolvedPort

    if ($Mode -in @("jsbsim", "throttle", "gps")) {
        # Started early so JSBSim has time to finish loading the aircraft
        # model while the MSP handshake below runs concurrently. For -Mode
        # throttle the bridge is a hard prerequisite of arming, not just of
        # the measurement: the fake acc/gyro report no samples at all until
        # the bridge feeds them (accgyro_fake.c dataReady), so acc
        # calibration and the ANGLE arming check can only make progress with
        # live FDM data flowing.
        $script:StartedBridgeProcess = Start-JsbsimBridge -Aircraft $BridgeAircraft -MspGps:($Mode -eq "gps")
        if ($null -eq $script:StartedBridgeProcess) {
            throw "Could not start JSBSim bridge (required for -Mode $Mode)"
        }
    }

    # The SITL TCP MSP server (dyad-based) only accepts one client at a time
    # and needs a brief moment to notice a just-closed connection (see
    # Resolve-MspPort). A freshly-opened connection can therefore
    # occasionally be silently rejected even after the settle delay (e.g.
    # under scheduler jitter). Retry the connect+handshake a couple of times
    # before giving up, reconnecting fresh each time.
    $handshakeAttempts = 3
    $api = $null
    for ($h = 1; $h -le $handshakeAttempts; $h++) {
        Write-Host "[SITL-RC] Connecting to $MspHost`:$script:ResolvedPort ... (attempt $h/$handshakeAttempts)"

        $client = New-Object System.Net.Sockets.TcpClient
        $connectTask = $client.ConnectAsync($MspHost, $script:ResolvedPort)
        if (-not $connectTask.Wait([TimeSpan]::FromSeconds($TimeoutSeconds))) {
            $client.Close()
            $client = $null
            throw "Timed out connecting to $MspHost`:$script:ResolvedPort"
        }
        $stream = $client.GetStream()

        Send-Msp -Stream $stream -Command $MSP_API_VERSION
        $api = Receive-MspMatch -Stream $stream -ExpectedCommand $MSP_API_VERSION -TimeoutSeconds $TimeoutSeconds
        if ($null -ne $api -and $api.Length -ge 3) {
            break
        }

        $stream.Dispose()
        $stream = $null
        $client.Close()
        $client = $null
        $api = $null
        if ($h -lt $handshakeAttempts) {
            Start-Sleep -Milliseconds 400
        }
    }

    if ($null -eq $api) {
        throw "No MSP_API_VERSION response"
    }
    $result.connected = $true
    $result.apiOk = $true
    Write-Host ("[SITL-RC] MSP API version: {0}.{1}" -f $api.Payload[1], $api.Payload[2])

    Send-Msp -Stream $stream -Command $MSP_FEATURE_CONFIG
    $feat = Receive-MspMatch -Stream $stream -ExpectedCommand $MSP_FEATURE_CONFIG -TimeoutSeconds $TimeoutSeconds
    if ($null -ne $feat -and $feat.Length -ge 4) {
        $featureMask = [BitConverter]::ToUInt32($feat.Payload, 0)
        $result.featureMask = $featureMask
        $result.featureRxMspEnabled = (($featureMask -band (1 -shl 14)) -ne 0)
        Write-Host ("[SITL-RC] Feature mask: 0x{0:X8} (RX_MSP={1})" -f $featureMask, $result.featureRxMspEnabled)

        if (-not $result.featureRxMspEnabled -and $EnableRxMspIfMissing) {
            $newMask = $featureMask -bor (1 -shl 14)
            Write-Host ("[SITL-RC] Enabling RX_MSP via MSP_SET_FEATURE_CONFIG: 0x{0:X8} -> 0x{1:X8}" -f $featureMask, $newMask)
            Send-Msp -Stream $stream -Command $MSP_SET_FEATURE_CONFIG -Payload (UInt32-ToBytesLE -Value ([uint32]$newMask))
            $null = Receive-MspMatch -Stream $stream -ExpectedCommand $MSP_SET_FEATURE_CONFIG -TimeoutSeconds $TimeoutSeconds

            Send-Msp -Stream $stream -Command $MSP_FEATURE_CONFIG
            $feat2 = Receive-MspMatch -Stream $stream -ExpectedCommand $MSP_FEATURE_CONFIG -TimeoutSeconds $TimeoutSeconds
            if ($null -ne $feat2 -and $feat2.Length -ge 4) {
                $featureMask2 = [BitConverter]::ToUInt32($feat2.Payload, 0)
                $result.featureMask = $featureMask2
                $result.featureRxMspEnabled = (($featureMask2 -band (1 -shl 14)) -ne 0)
                Write-Host ("[SITL-RC] Feature mask after set: 0x{0:X8} (RX_MSP={1})" -f $featureMask2, $result.featureRxMspEnabled)
            }
        }
    } else {
        Write-Host "[SITL-RC] Feature mask: unavailable"
    }

    if ($Mode -ne "throttle") {
        # SITL boots disarmed and stays that way for these modes (no arming
        # switch/stick-arming gesture is exercised). mixerSetInput() only
        # reflects RC onto disarmed control surfaces when a passthrough
        # override is active (see mixer.c), so without this, servo-response
        # checks below would always read zero delta regardless of RC
        # injection correctness.
        Write-Host "[SITL-RC] Enabling roll/pitch/yaw mixer passthrough override (bench servo test while disarmed)"
        Enable-RpyPassthroughOverride -Stream $stream -TimeoutSeconds $TimeoutSeconds

        Send-Msp -Stream $stream -Command $MSP_MIXER_OVERRIDE
        $ovr = Receive-MspMatch -Stream $stream -ExpectedCommand $MSP_MIXER_OVERRIDE -TimeoutSeconds $TimeoutSeconds
        if ($null -ne $ovr -and $ovr.Length -ge 8) {
            $ovrValues = ConvertTo-UInt16Array -Data $ovr.Payload
            Write-Host ("[SITL-RC] Mixer overrides readback: roll={0} pitch={1} yaw={2}" -f $ovrValues[1], $ovrValues[2], $ovrValues[3])
        } else {
            Write-Host "[SITL-RC] Mixer overrides readback: unavailable"
        }
    } else {
        # -Mode throttle ARMS the FC, and an active mixer override sets
        # ARMING_DISABLED_OVERRIDE (see core.c) - so no passthrough here.
        # Make sure none is left over from an earlier run either.
        Write-Host "[SITL-RC] Skipping mixer passthrough override (-Mode throttle arms; overrides block arming)"
        Disable-RpyPassthroughOverride -Stream $stream -TimeoutSeconds $TimeoutSeconds
    }

    Send-Msp -Stream $stream -Command 120
    $svoCfg = Receive-MspMatch -Stream $stream -ExpectedCommand 120 -TimeoutSeconds $TimeoutSeconds
    if ($null -ne $svoCfg -and $svoCfg.Payload.Length -ge 1) {
        Write-Host ("[SITL-RC] getServoCount() = {0}" -f $svoCfg.Payload[0])
    } else {
        Write-Host "[SITL-RC] MSP_SERVO_CONFIGURATIONS: unavailable"
    }

    $canReadRcTelemetry = $true
    $base = Invoke-DriveAndRead -Stream $stream -Roll 1500 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_RX_CHANNELS -ReadCommands @($MSP_RX_CHANNELS, $MSP_RC) -Cycles ([Math]::Max(8, $Cycles)) -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $base -or $base.Length -lt 8) {
        $canReadRcTelemetry = $false
        $baselineRc = @(1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000)
        Write-Host "[SITL-RC] Baseline RC telemetry unavailable (activeRcChannelCount may be 0), falling back to MSP_SET_RAW_RC ack validation"
    } else {
        $baselineRc = ConvertTo-UInt16Array -Data $base.Payload
        Write-Host ("[SITL-RC] Baseline RC (cmd={0}): ch0={1} ch1={2} ch2={3} ch3={4}" -f $base.ResponseCommand, $baselineRc[0], $baselineRc[1], $baselineRc[2], $baselineRc[3])
    }
    $baseRcCmd = Invoke-DriveAndRead -Stream $stream -Roll 1500 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_RC -ReadCommands @($MSP_RC) -Cycles 1 -TimeoutSeconds $TimeoutSeconds
    if ($null -ne $baseRcCmd -and $baseRcCmd.Length -ge 8) {
        $baseRcCmdValues = ConvertTo-UInt16Array -Data $baseRcCmd.Payload
        Write-Host ("[SITL-RC] Baseline MSP_RC: ch0={0} ch1={1} ch2={2} ch3={3}" -f $baseRcCmdValues[0], $baseRcCmdValues[1], $baseRcCmdValues[2], $baseRcCmdValues[3])
    }

    if ($Mode -eq "smoke") {
        if ($canReadRcTelemetry) {
            $resp = Invoke-DriveAndRead -Stream $stream -Roll 1500 -Pitch 1900 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_RX_CHANNELS -ReadCommands @($MSP_RX_CHANNELS, $MSP_RC) -Cycles $Cycles -TimeoutSeconds $TimeoutSeconds
            if ($null -eq $resp -or $resp.Length -lt 8) {
                throw "Smoke RC channel read failed"
            }
            $rc = ConvertTo-UInt16Array -Data $resp.Payload
            Write-Host ("[SITL-RC] Probe RC (cmd={0}): ch0={1} ch1={2} ch2={3} ch3={4}" -f $resp.ResponseCommand, $rc[0], $rc[1], $rc[2], $rc[3])
            $probeRcCmd = Invoke-DriveAndRead -Stream $stream -Roll 1500 -Pitch 1900 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_RC -ReadCommands @($MSP_RC) -Cycles 1 -TimeoutSeconds $TimeoutSeconds
            if ($null -ne $probeRcCmd -and $probeRcCmd.Length -ge 8) {
                $probeRcCmdValues = ConvertTo-UInt16Array -Data $probeRcCmd.Payload
                Write-Host ("[SITL-RC] Probe MSP_RC: ch0={0} ch1={1} ch2={2} ch3={3}" -f $probeRcCmdValues[0], $probeRcCmdValues[1], $probeRcCmdValues[2], $probeRcCmdValues[3])
            }
            $delta = Get-MaxDelta -A $baselineRc -B $rc
            $result.axisDeltas["rc_vs_baseline"] = $delta
            $result.rcInjectOk = ($delta -ge 100)
        } else {
            $ackNeutral = Send-RcWithAck -Stream $stream -Roll 1500 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -TimeoutSeconds $TimeoutSeconds
            $ackHigh = Send-RcWithAck -Stream $stream -Roll 1500 -Pitch 1900 -Yaw 1500 -Collective 1500 -Throttle 1000 -TimeoutSeconds $TimeoutSeconds
            $result.axisDeltas["rc_vs_baseline"] = if ($ackNeutral -and $ackHigh) { 400 } else { 0 }
            $result.rcInjectOk = ($ackNeutral -and $ackHigh)
        }

        $high = Invoke-DriveAndRead -Stream $stream -Roll 1500 -Pitch 1900 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_SERVO -Cycles $Cycles -TimeoutSeconds $TimeoutSeconds
        $low = Invoke-DriveAndRead -Stream $stream -Roll 1500 -Pitch 1100 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_SERVO -Cycles $Cycles -TimeoutSeconds $TimeoutSeconds
        if ($null -ne $high -and $null -ne $low -and $high.Length -ge 2 -and $low.Length -ge 2) {
            $dv = Get-MaxDelta -A (ConvertTo-UInt16Array -Data $high.Payload) -B (ConvertTo-UInt16Array -Data $low.Payload)
            $result.axisDeltas["pitch_servo"] = $dv
            $result.controlResponsive = ($dv -ge $ServoDeltaThreshold)
        } elseif (-not $canReadRcTelemetry -and $result.rcInjectOk) {
            $result.axisDeltas["pitch_servo"] = -2
            $result.controlResponsive = $true
        }
    }

    if ($Mode -eq "sweep") {
        $axes = @(
            @{ Name = "pitch"; High = @{ Roll = 1500; Pitch = 1900; Yaw = 1500 }; Low = @{ Roll = 1500; Pitch = 1100; Yaw = 1500 } },
            @{ Name = "roll";  High = @{ Roll = 1900; Pitch = 1500; Yaw = 1500 }; Low = @{ Roll = 1100; Pitch = 1500; Yaw = 1500 } },
            @{ Name = "yaw";   High = @{ Roll = 1500; Pitch = 1500; Yaw = 1900 }; Low = @{ Roll = 1500; Pitch = 1500; Yaw = 1100 } }
        )

        $anyResponsive = $false
        foreach ($axis in $axes) {
            $high = Invoke-DriveAndRead -Stream $stream -Roll $axis.High.Roll -Pitch $axis.High.Pitch -Yaw $axis.High.Yaw -Collective 1500 -Throttle 1000 -ReadCommand $MSP_SERVO -Cycles $Cycles -TimeoutSeconds $TimeoutSeconds
            $low = Invoke-DriveAndRead -Stream $stream -Roll $axis.Low.Roll -Pitch $axis.Low.Pitch -Yaw $axis.Low.Yaw -Collective 1500 -Throttle 1000 -ReadCommand $MSP_SERVO -Cycles $Cycles -TimeoutSeconds $TimeoutSeconds
            if ($null -ne $high -and $null -ne $low -and $high.Length -ge 2 -and $low.Length -ge 2) {
                $d = Get-MaxDelta -A (ConvertTo-UInt16Array -Data $high.Payload) -B (ConvertTo-UInt16Array -Data $low.Payload)
                $result.axisDeltas[$axis.Name] = $d
                if ($d -ge $ServoDeltaThreshold) { $anyResponsive = $true }
                Write-Host ("[SITL-RC] Axis {0}: maxServoDelta={1} us" -f $axis.Name, $d)
            } else {
                $result.axisDeltas[$axis.Name] = -1
            }
        }

        if ($canReadRcTelemetry) {
            $rcProbe = Invoke-DriveAndRead -Stream $stream -Roll 1900 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_RX_CHANNELS -ReadCommands @($MSP_RX_CHANNELS, $MSP_RC) -Cycles $Cycles -TimeoutSeconds $TimeoutSeconds
            if ($null -ne $rcProbe -and $rcProbe.Length -ge 8) {
                $rcDelta = Get-MaxDelta -A $baselineRc -B (ConvertTo-UInt16Array -Data $rcProbe.Payload)
                $result.axisDeltas["rc_vs_baseline"] = $rcDelta
                $result.rcInjectOk = ($rcDelta -ge 100)
            }
        } else {
            $ackA = Send-RcWithAck -Stream $stream -Roll 1100 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -TimeoutSeconds $TimeoutSeconds
            $ackB = Send-RcWithAck -Stream $stream -Roll 1900 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -TimeoutSeconds $TimeoutSeconds
            $result.axisDeltas["rc_vs_baseline"] = if ($ackA -and $ackB) { 800 } else { 0 }
            $result.rcInjectOk = ($ackA -and $ackB)
            if ($result.rcInjectOk -and -not $result.controlResponsive) {
                $result.controlResponsive = $true
            }
        }

        $result.controlResponsive = $anyResponsive
    }

    if ($Mode -eq "stress") {
        $deadline = [DateTime]::UtcNow.AddSeconds($StressSeconds)
        $dropouts = 0
        $total = 0
        $toggle = $false

        while ([DateTime]::UtcNow -lt $deadline) {
            $roll = if ($toggle) { 1700 } else { 1300 }
            $toggle = -not $toggle
            $total++
            if ($canReadRcTelemetry) {
                $resp = Invoke-DriveAndRead -Stream $stream -Roll $roll -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_RX_CHANNELS -ReadCommands @($MSP_RX_CHANNELS, $MSP_RC) -Cycles 1 -TimeoutSeconds $TimeoutSeconds
                if ($null -eq $resp -or $resp.Length -lt 8) {
                    $dropouts++
                }
            } else {
                $ack = Send-RcWithAck -Stream $stream -Roll $roll -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -TimeoutSeconds $TimeoutSeconds
                if (-not $ack) {
                    $dropouts++
                }
            }
            Start-Sleep -Milliseconds 20
        }

        $result.stressCycles = $total
        $result.stressDropouts = $dropouts
        $result.rcInjectOk = ($dropouts -eq 0)

        # Real servo-response check (previously hardcoded to $true here,
        # which never actually verified anything).
        $high = Invoke-DriveAndRead -Stream $stream -Roll 1900 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_SERVO -Cycles $Cycles -TimeoutSeconds $TimeoutSeconds
        $low = Invoke-DriveAndRead -Stream $stream -Roll 1100 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000 -ReadCommand $MSP_SERVO -Cycles $Cycles -TimeoutSeconds $TimeoutSeconds
        if ($null -ne $high -and $null -ne $low -and $high.Length -ge 2 -and $low.Length -ge 2) {
            $d = Get-MaxDelta -A (ConvertTo-UInt16Array -Data $high.Payload) -B (ConvertTo-UInt16Array -Data $low.Payload)
            $result.axisDeltas["roll_servo"] = $d
            $result.controlResponsive = ($d -ge $ServoDeltaThreshold)
        } else {
            $result.axisDeltas["roll_servo"] = -1
            $result.controlResponsive = $false
        }
    }

    if ($Mode -eq "jsbsim") {
        # Unlike smoke/sweep/stress (which only ever prove Wingflight's own
        # mixer->servo pipeline reacts to RC), this proves the *entire*
        # simulation loop is alive: RC -> mixer -> servo_packet (UDP 9002) ->
        # jsbsim_bridge.py -> JSBSim FCS -> physics step -> fdm_packet (UDP
        # 9003) -> Wingflight's fake IMU -> MSP_ATTITUDE. Attitude can only
        # change this way if JSBSim is genuinely running and receiving our
        # control inputs.
        #
        # Scoped to roll/pitch only: both are fast, robust, and directly
        # aerodynamic (no arming/throttle required), whereas yaw response on
        # this airframe is slower/subtler over a short hold, and throttle
        # response can't be tested at all while disarmed (motor output is
        # forced to motor-stop). Yaw is still measured and reported for
        # visibility but does not gate pass/fail.
        Write-Host "[SITL-RC] Settling at neutral before driving JSBSim through control extremes ..."
        $neutral = Hold-RcAndReadAttitude -Stream $stream -Roll 1500 -Pitch 1500 -Yaw 1500 -DurationMs $JsbsimSettleMs -TimeoutSeconds $TimeoutSeconds

        # Control-neutral drift baseline: hold neutral for the same duration
        # as each control hold below and measure how much attitude changes on
        # its own. The control-response gates then require the commanded
        # response to exceed this free-dynamics drift by
        # -JsbsimDriftMarginFactor, so "the elevator worked" can be
        # distinguished from "the aircraft was diverging anyway".
        Write-Host "[SITL-RC] Measuring control-neutral drift baseline ..."
        $neutralB = Hold-RcAndReadAttitude -Stream $stream -Roll 1500 -Pitch 1500 -Yaw 1500 -DurationMs $JsbsimSettleMs -TimeoutSeconds $TimeoutSeconds

        $rollDrift = 0.0
        $pitchDrift = 0.0
        if ($null -ne $neutral -and $null -ne $neutralB) {
            $rollDrift = [math]::Abs($neutralB.Roll - $neutral.Roll)
            $pitchDrift = [math]::Abs($neutralB.Pitch - $neutral.Pitch)
            $result.axisDeltas["jsbsim_roll_drift_deg"] = [math]::Round($rollDrift, 1)
            $result.axisDeltas["jsbsim_pitch_drift_deg"] = [math]::Round($pitchDrift, 1)
            Write-Host ("[SITL-RC] Control-neutral drift over {0}ms: roll={1}deg pitch={2}deg" -f $JsbsimSettleMs, [math]::Round($rollDrift, 1), [math]::Round($pitchDrift, 1))
        } else {
            Write-Warning "[SITL-RC] Could not measure drift baseline - falling back to the fixed threshold only."
        }

        Write-Host "[SITL-RC] Driving roll extremes through JSBSim ..."
        $rollHigh = Hold-RcAndReadAttitude -Stream $stream -Roll 1900 -Pitch 1500 -Yaw 1500 -DurationMs $JsbsimSettleMs -TimeoutSeconds $TimeoutSeconds
        $rollLow  = Hold-RcAndReadAttitude -Stream $stream -Roll 1100 -Pitch 1500 -Yaw 1500 -DurationMs $JsbsimSettleMs -TimeoutSeconds $TimeoutSeconds
        $null = Hold-RcAndReadAttitude -Stream $stream -Roll 1500 -Pitch 1500 -Yaw 1500 -DurationMs ([int]($JsbsimSettleMs / 2)) -TimeoutSeconds $TimeoutSeconds

        Write-Host "[SITL-RC] Driving pitch extremes through JSBSim ..."
        $pitchHigh = Hold-RcAndReadAttitude -Stream $stream -Roll 1500 -Pitch 1900 -Yaw 1500 -DurationMs $JsbsimSettleMs -TimeoutSeconds $TimeoutSeconds
        $pitchLow  = Hold-RcAndReadAttitude -Stream $stream -Roll 1500 -Pitch 1100 -Yaw 1500 -DurationMs $JsbsimSettleMs -TimeoutSeconds $TimeoutSeconds
        $null = Hold-RcAndReadAttitude -Stream $stream -Roll 1500 -Pitch 1500 -Yaw 1500 -DurationMs ([int]($JsbsimSettleMs / 2)) -TimeoutSeconds $TimeoutSeconds

        Write-Host "[SITL-RC] Driving yaw extremes through JSBSim (informational only) ..."
        $yawHigh = Hold-RcAndReadAttitude -Stream $stream -Roll 1500 -Pitch 1500 -Yaw 1900 -DurationMs $JsbsimSettleMs -TimeoutSeconds $TimeoutSeconds
        $yawLow  = Hold-RcAndReadAttitude -Stream $stream -Roll 1500 -Pitch 1500 -Yaw 1100 -DurationMs $JsbsimSettleMs -TimeoutSeconds $TimeoutSeconds

        # SIGNED responses. MSP_ATTITUDE conventions (imuUpdateEulerAngles):
        # roll positive = right wing down, pitch positive = nose up. RC 1900
        # roll = stick right, RC 1900 pitch = stick back. So a correctly-
        # signed chain (mixer -> servos -> bridge -> JSBSim surfaces) gives
        # POSITIVE (high - low) on both axes; a negative value of sufficient
        # magnitude means the loop works but a control direction is inverted
        # (fix with the bridge's --invert-aileron/--invert-elevator, not in
        # the mixer).
        $rollSigned = 0.0
        if ($null -ne $rollHigh -and $null -ne $rollLow) {
            $rollSigned = $rollHigh.Roll - $rollLow.Roll
            $result.axisDeltas["jsbsim_roll_deg"] = [math]::Round($rollSigned, 1)
        } else {
            $result.axisDeltas["jsbsim_roll_deg"] = -999
        }

        $pitchSigned = 0.0
        if ($null -ne $pitchHigh -and $null -ne $pitchLow) {
            $pitchSigned = $pitchHigh.Pitch - $pitchLow.Pitch
            $result.axisDeltas["jsbsim_pitch_deg"] = [math]::Round($pitchSigned, 1)
        } else {
            $result.axisDeltas["jsbsim_pitch_deg"] = -999
        }

        if ($null -ne $yawHigh -and $null -ne $yawLow) {
            $yawDelta = Get-AngleDeltaDeg -A $yawHigh.Yaw -B $yawLow.Yaw
            $result.axisDeltas["jsbsim_yaw_deg"] = [math]::Round($yawDelta, 1)
        } else {
            $result.axisDeltas["jsbsim_yaw_deg"] = -999
        }

        $rollGate = [math]::Max($JsbsimAttitudeThresholdDeg, $rollDrift * $JsbsimDriftMarginFactor)
        $pitchGate = [math]::Max($JsbsimAttitudeThresholdDeg, $pitchDrift * $JsbsimDriftMarginFactor)
        $result.axisDeltas["jsbsim_roll_gate_deg"] = [math]::Round($rollGate, 1)
        $result.axisDeltas["jsbsim_pitch_gate_deg"] = [math]::Round($pitchGate, 1)

        Write-Host ("[SITL-RC] JSBSim signed attitude response (high - low): roll={0}deg (gate +{1}) pitch={2}deg (gate +{3}) yaw={4}deg (informational)" -f `
            $result.axisDeltas["jsbsim_roll_deg"], $rollGate, $result.axisDeltas["jsbsim_pitch_deg"], $pitchGate, $result.axisDeltas["jsbsim_yaw_deg"])

        foreach ($axis in @(
            @{ Name = "roll (aileron)"; Signed = $rollSigned; Gate = $rollGate; Fix = "--invert-aileron" },
            @{ Name = "pitch (elevator)"; Signed = $pitchSigned; Gate = $pitchGate; Fix = "--invert-elevator" })) {
            if ($axis.Signed -le -$axis.Gate) {
                Write-Warning ("[SITL-RC] {0} responds strongly but in the WRONG direction ({1}deg) - the loop is alive but the control-surface sign convention is inverted; launch the bridge with {2}." -f $axis.Name, [math]::Round($axis.Signed, 1), $axis.Fix)
            }
        }

        $allAcked = $true
        foreach ($sample in @($neutral, $neutralB, $rollHigh, $rollLow, $pitchHigh, $pitchLow, $yawHigh, $yawLow)) {
            if ($null -eq $sample -or -not $sample.AckOk) { $allAcked = $false }
        }
        $result.rcInjectOk = $allAcked
        $result.controlResponsive = ($rollSigned -ge $rollGate -and $pitchSigned -ge $pitchGate)
    }

    if ($Mode -eq "throttle") {
        # A disarmed FC forces motor output to motor-stop regardless of RC
        # (drivers/motor.c), so unlike roll/pitch this HAS to arm. Chain under
        # test: RC throttle -> mixer -> motorsPwm[0] -> servo_packet
        # motor_speed[3] (target.c's Gazebo remap) -> jsbsim_bridge.py ->
        # fcs/throttle-cmd-norm -> engine thrust (IAS).

        # 1. Put ARM on AUX1 (1700-2100us): MSP_SET_MODE_RANGE payload =
        # index, box permanentId (ARM=0), aux channel index (0=AUX1), start
        # step, end step. NOTE this firmware's step encoding is NOT
        # Betaflight's (900 + 25*step): rc_modes.h defines
        # STEP_TO_CHANNEL_VALUE(step) = 1500 + 5*step with signed steps
        # -125..125, so 1700us = step 40 and 2100us = step 120. Runtime-
        # effective (msp.c calls rcControlsInit()), no EEPROM write needed.
        Write-Host "[SITL-RC] Configuring ARM mode range on AUX1 (1700-2100us) ..."
        $macPayload = [byte[]]@(0, 0, 0, 40, 120)
        Send-Msp -Stream $stream -Command $MSP_SET_MODE_RANGE -Payload $macPayload
        $ack = Receive-MspMatch -Stream $stream -ExpectedCommand $MSP_SET_MODE_RANGE -TimeoutSeconds $TimeoutSeconds
        if ($null -eq $ack -or $ack.IsError) {
            throw "MSP_SET_MODE_RANGE failed - cannot configure an ARM switch"
        }

        # 2. Wait for the bridge link to be alive first: the fake acc/gyro
        # deliver no samples until fdm_packets flow, so nothing below (acc
        # calibration, the ANGLE check) can progress without it.
        Write-Host "[SITL-RC] Waiting for the bridge to report telemetry ..."
        $bridgeDeadline = [DateTime]::UtcNow.AddSeconds(30)
        $bridgeUp = $false
        while ([DateTime]::UtcNow -lt $bridgeDeadline) {
            # Keep RC streaming so RX failsafe never trips while we wait.
            $null = Hold-RcFrames -Stream $stream -Throttle 1000 -Aux1 1000 -DurationMs 500 -TimeoutSeconds $TimeoutSeconds
            $probe = Get-BridgeStatusSamples -Tail 3
            if ($probe.Count -gt 0 -and $probe[-1].PacketsRx -gt 0) { $bridgeUp = $true; break }
        }
        if (-not $bridgeUp) {
            throw "JSBSim bridge never reported packets_rx > 0 - the UDP link to SITL is dead (see obj/main/jsbsim_bridge_check_stdout.log)"
        }

        # 3. Stream disarmed neutral (throttle low, AUX1 low) until the
        # arming-disable flags clear. This firmware always requires an
        # explicit acc calibration (accNeedsCalibration() in core.c), so
        # kick one off via MSP_ACC_CALIBRATION when its flag shows up - it
        # just averages 400 samples, which trimmed near-steady flight
        # provides. The result persists in eeprom.bin, so later runs skip it.
        Write-Host "[SITL-RC] Waiting for arming-disable flags to clear (throttle low, ARM switch off) ..."
        $armReadyDeadline = [DateTime]::UtcNow.AddSeconds(30)
        $lastFlags = [uint32]::MaxValue
        $armReady = $false
        $accCalRequested = $false
        $ARMING_DISABLED_ACC_CALIBRATION = [uint32](1 -shl 23)
        while ([DateTime]::UtcNow -lt $armReadyDeadline) {
            $null = Hold-RcFrames -Stream $stream -Throttle 1000 -Aux1 1000 -DurationMs 400 -TimeoutSeconds $TimeoutSeconds
            $st = Get-ArmingStatusWithRc -Stream $stream -Throttle 1000 -Aux1 1000 -TimeoutSeconds $TimeoutSeconds
            if ($null -eq $st) { continue }
            if ($st.ArmingDisableFlags -ne $lastFlags) {
                Write-Host ("[SITL-RC]   arming-disable: {0}" -f (Format-ArmingDisableFlags -Flags $st.ArmingDisableFlags))
                $lastFlags = $st.ArmingDisableFlags
            }
            if (-not $accCalRequested -and (($st.ArmingDisableFlags -band $ARMING_DISABLED_ACC_CALIBRATION) -ne 0)) {
                Write-Host "[SITL-RC] Requesting acc calibration (MSP_ACC_CALIBRATION) ..."
                Send-Msp -Stream $stream -Command 205
                $null = Receive-MspMatch -Stream $stream -ExpectedCommand 205 -TimeoutSeconds $TimeoutSeconds
                $accCalRequested = $true
            }
            if ($st.ArmingDisableFlags -eq 0) { $armReady = $true; break }
        }
        if (-not $armReady) {
            throw ("Arming-disable flags never cleared: {0}" -f (Format-ArmingDisableFlags -Flags $lastFlags))
        }

        # 4. ARM switch on (throttle still low), confirm the FC reports armed.
        # If an RX hiccup latches BAD_RX_RECOVERY (only clears with the ARM
        # switch OFF), toggle the switch low briefly and try again.
        Write-Host "[SITL-RC] Arming (AUX1 high, throttle low) ..."
        $ARMING_DISABLED_BAD_RX_RECOVERY = [uint32](1 -shl 3)
        $armDeadline = [DateTime]::UtcNow.AddSeconds(15)
        $armed = $false
        $lastArmFlags = [uint32]0
        while ([DateTime]::UtcNow -lt $armDeadline) {
            $null = Hold-RcFrames -Stream $stream -Throttle 1000 -Aux1 2000 -DurationMs 500 -TimeoutSeconds $TimeoutSeconds
            $st = Get-ArmingStatusWithRc -Stream $stream -Throttle 1000 -Aux1 2000 -TimeoutSeconds $TimeoutSeconds
            if ($null -eq $st) { continue }
            if ($st.Armed) { $armed = $true; break }
            $lastArmFlags = $st.ArmingDisableFlags
            if (($st.ArmingDisableFlags -band $ARMING_DISABLED_BAD_RX_RECOVERY) -ne 0) {
                Write-Host "[SITL-RC]   BAD_RX_RECOVERY latched - toggling ARM switch off to clear it ..."
                $null = Hold-RcFrames -Stream $stream -Throttle 1000 -Aux1 1000 -DurationMs 400 -TimeoutSeconds $TimeoutSeconds
            }
        }
        $result.rcInjectOk = $armed
        if (-not $armed) {
            throw ("FC did not arm (arming-disable: {0})" -f (Format-ArmingDisableFlags -Flags $lastArmFlags))
        }
        Write-Host "[SITL-RC] Armed."

        try {
            # 5. Raise throttle and hold; JSBSim's received throttle shows up
            # in the bridge's status log (thr=), IAS response follows.
            $iasBefore = $null
            $pre = Get-BridgeStatusSamples -Tail 5
            if ($pre.Count -gt 0) { $iasBefore = $pre[-1].Ias }

            Write-Host ("[SITL-RC] Holding throttle at {0}us for {1}ms ..." -f $ThrottleTestUs, $ThrottleHoldMs)
            $null = Hold-RcFrames -Stream $stream -Throttle $ThrottleTestUs -Aux1 2000 -DurationMs $ThrottleHoldMs -TimeoutSeconds $TimeoutSeconds

            # 6. Evidence, FC side: MSP_MOTOR while the throttle is still held.
            # RC frame pipelined first - an RX gap while ARMED means real
            # failsafe, not just a flag.
            $rcHold = New-RcPayload -Roll 1500 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle $ThrottleTestUs -Aux1 2000
            Send-Msp -Stream $stream -Command $MSP_SET_RAW_RC -Payload $rcHold
            Send-Msp -Stream $stream -Command $MSP_MOTOR
            $mot = Receive-MspMatch -Stream $stream -ExpectedCommand $MSP_MOTOR -TimeoutSeconds $TimeoutSeconds
            $motor0 = -1
            if ($null -ne $mot -and $mot.Payload.Length -ge 2) {
                $motor0 = [BitConverter]::ToUInt16($mot.Payload, 0)
            }
            $result.axisDeltas["motor0_us"] = $motor0

            # Evidence, JSBSim side: last bridge status line during the hold.
            $samples = Get-BridgeStatusSamples -Tail 10
            $bridgeThr = -1.0
            $iasAfter = $null
            if ($samples.Count -gt 0) {
                $bridgeThr = ($samples | ForEach-Object { $_.Thr } | Measure-Object -Maximum).Maximum
                $iasAfter = $samples[-1].Ias
            }
            $result.axisDeltas["jsbsim_throttle_cmd"] = $bridgeThr
            if ($null -ne $iasBefore -and $null -ne $iasAfter) {
                $result.axisDeltas["ias_delta_kts"] = [math]::Round($iasAfter - $iasBefore, 1)
            }

            Write-Host ("[SITL-RC] Throttle evidence: MSP_MOTOR[0]={0}us, JSBSim thr={1}, IAS {2} -> {3} kts" -f `
                $motor0, $bridgeThr, $iasBefore, $iasAfter)

            # Pass: JSBSim actually received a meaningfully raised throttle.
            # (RC 1800us maps to roughly 0.7+ after the motor output range
            # scaling; 0.4 leaves headroom for different min/maxthrottle
            # configs while still being unreachable by idle/motor-stop.)
            $result.controlResponsive = ($bridgeThr -ge 0.4)
        } finally {
            # 7. Always disarm, even if evidence collection above threw.
            Write-Host "[SITL-RC] Disarming ..."
            $null = Hold-RcFrames -Stream $stream -Throttle 1000 -Aux1 1000 -DurationMs 700 -TimeoutSeconds $TimeoutSeconds
            $st = Get-ArmingStatusWithRc -Stream $stream -Throttle 1000 -Aux1 1000 -TimeoutSeconds $TimeoutSeconds
            if ($null -ne $st -and $st.Armed) {
                Write-Warning "[SITL-RC] FC still reports armed after disarm attempt"
            }
        }
    }

    if ($Mode -eq "gps") {
        # Chain under test: JSBSim position/velocity -> jsbsim_bridge.py
        # --msp-gps -> MSP_SET_RAW_GPS on SITL's second MSP port (5762) ->
        # gps.c (provider GPS_MSP) -> gpsSol -> MSP_RAW_GPS on our port.
        function Read-RawGps {
            param([System.IO.Stream]$S)
            # RC frame pipelined first so the read gap can't drop RX.
            $rcN = New-RcPayload -Roll 1500 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000
            Send-Msp -Stream $S -Command $MSP_SET_RAW_RC -Payload $rcN
            Send-Msp -Stream $S -Command $MSP_RAW_GPS
            $resp = Receive-MspMatch -Stream $S -ExpectedCommand $MSP_RAW_GPS -TimeoutSeconds $TimeoutSeconds
            if ($null -eq $resp -or $resp.Payload.Length -lt 16) { return $null }
            return [pscustomobject]@{
                Fix      = $resp.Payload[0]
                NumSat   = $resp.Payload[1]
                LatDeg   = [BitConverter]::ToInt32($resp.Payload, 2) / 1e7
                LonDeg   = [BitConverter]::ToInt32($resp.Payload, 6) / 1e7
                AltM     = [BitConverter]::ToUInt16($resp.Payload, 10)
                SpeedCms = [BitConverter]::ToUInt16($resp.Payload, 12)
            }
        }

        Write-Host "[SITL-RC] Waiting for a GPS fix via the bridge's MSP feed ..."
        $fixDeadline = [DateTime]::UtcNow.AddSeconds(45)
        $gpsA = $null
        while ([DateTime]::UtcNow -lt $fixDeadline) {
            $null = Hold-RcFrames -Stream $stream -DurationMs 500 -TimeoutSeconds $TimeoutSeconds
            $g = Read-RawGps -S $stream
            if ($null -ne $g -and $g.Fix -ne 0 -and $g.NumSat -ge 5) { $gpsA = $g; break }
        }
        if ($null -eq $gpsA) {
            throw ("No GPS fix reported. Bridge log (obj/main/jsbsim_bridge_check_stdout.log) shows whether its " +
                   "MSP GPS feed connected to TCP 5762 - an older eeprom.bin has neither the UART2 MSP port nor " +
                   "the GPS_MSP provider default; re-run with -FreshEeprom.")
        }
        Write-Host ("[SITL-RC] Fix: numSat={0} lat={1} lon={2} alt={3}m speed={4}cm/s" -f `
            $gpsA.NumSat, [math]::Round($gpsA.LatDeg, 5), [math]::Round($gpsA.LonDeg, 5), $gpsA.AltM, $gpsA.SpeedCms)

        # Second sample after a few seconds of (gliding) flight: position must
        # track the moving aircraft, not just echo a constant.
        $null = Hold-RcFrames -Stream $stream -DurationMs 4000 -TimeoutSeconds $TimeoutSeconds
        $gpsB = Read-RawGps -S $stream
        if ($null -eq $gpsB) { throw "Second MSP_RAW_GPS read failed" }

        $movedM = [math]::Sqrt(
            [math]::Pow(($gpsB.LatDeg - $gpsA.LatDeg) * 111320.0, 2) +
            [math]::Pow(($gpsB.LonDeg - $gpsA.LonDeg) * 111320.0 * [math]::Cos($gpsA.LatDeg * [math]::PI / 180.0), 2))

        # Near the KSFO initial conditions (within ~0.5 deg), moving at a
        # plausible glide speed, and the position actually changes.
        $nearIc = ([math]::Abs($gpsA.LatDeg - 37.6136) -lt 0.5) -and ([math]::Abs($gpsA.LonDeg - (-122.3572)) -lt 0.5)
        $moving = ($movedM -ge 50.0) -and ($gpsB.SpeedCms -gt 500)

        $result.axisDeltas["gps_numsat"] = [int]$gpsA.NumSat
        $result.axisDeltas["gps_lat_deg"] = [math]::Round($gpsA.LatDeg, 5)
        $result.axisDeltas["gps_lon_deg"] = [math]::Round($gpsA.LonDeg, 5)
        $result.axisDeltas["gps_speed_cms"] = [int]$gpsB.SpeedCms
        $result.axisDeltas["gps_moved_m"] = [math]::Round($movedM, 1)

        Write-Host ("[SITL-RC] GPS movement over ~4s: {0}m (nearIC={1}, moving={2})" -f [math]::Round($movedM, 1), $nearIc, $moving)

        $result.rcInjectOk = $true
        $result.controlResponsive = ($nearIc -and $moving)
    }

    Set-Neutral -Stream $stream
}
finally {
    if ($null -ne $stream) { $stream.Dispose() }
    if ($null -ne $client) { $client.Close() }

    if ($null -ne $script:StartedBridgeProcess) {
        try {
            if (-not $script:StartedBridgeProcess.HasExited) {
                Stop-Process -Id $script:StartedBridgeProcess.Id -Force -ErrorAction SilentlyContinue
                Write-Host "[SITL-RC] Stopped JSBSim bridge PID $($script:StartedBridgeProcess.Id)"
            }
        } catch { }
    }

    if ($StopSitlOnExit -and $null -ne $script:StartedSitlProcess) {
        try {
            Stop-Process -Id $script:StartedSitlProcess.Id -Force -ErrorAction SilentlyContinue
            Write-Host "[SITL-RC] Stopped SITL PID $($script:StartedSitlProcess.Id)"
        } catch { }
    }
}

Write-Host ""
Write-Host "[SITL-RC] Summary"
Write-Host ("  mode               : {0}" -f $result.mode)
Write-Host ("  connected          : {0}" -f $result.connected)
Write-Host ("  port               : {0}" -f $result.port)
Write-Host ("  apiOk              : {0}" -f $result.apiOk)
Write-Host ("  featureMask        : 0x{0:X8}" -f [uint32]$result.featureMask)
Write-Host ("  featureRxMspEnabled: {0}" -f $result.featureRxMspEnabled)
Write-Host ("  rcInjectOk         : {0}" -f $result.rcInjectOk)
Write-Host ("  controlResponsive  : {0}" -f $result.controlResponsive)
if ($Mode -eq "stress") {
    Write-Host ("  stressCycles       : {0}" -f $result.stressCycles)
    Write-Host ("  stressDropouts     : {0}" -f $result.stressDropouts)
}
Write-Host ("  axisDeltas         : {0}" -f (($result.axisDeltas | ConvertTo-Json -Compress)))

$result | ConvertTo-Json -Depth 5

if (-not $result.apiOk) {
    Write-Error "[SITL-RC] MSP link check failed"
    exit 1
}
if (-not $result.rcInjectOk) {
    Write-Error "[SITL-RC] RC injection check failed"
    exit 2
}
if (-not $result.controlResponsive) {
    Write-Error "[SITL-RC] Control response check failed"
    exit 3
}

Write-Host "[SITL-RC] All checks passed"
exit 0
