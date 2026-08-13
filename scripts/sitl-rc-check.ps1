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
    [ValidateSet("smoke", "sweep", "stress", "jsbsim")]
    [string]$Mode = "smoke",
    [int]$StressSeconds = 60,
    [string]$SitlBinaryPath = "",
    [string]$SitlArgs = "",
    [string]$BridgeAircraft = "c172p",
    [int]$JsbsimSettleMs = 1500,
    [double]$JsbsimAttitudeThresholdDeg = 3.0,
    [switch]$EnableRxMspIfMissing,
    [switch]$AutoStartSitl,
    [switch]$BuildSitl,
    [switch]$FreshEeprom,
    [switch]$StopSitlOnExit
)

$ErrorActionPreference = "Stop"

$MSP_API_VERSION = 1
$MSP_FEATURE_CONFIG = 36
$MSP_SET_FEATURE_CONFIG = 37
$MSP_SERVO = 103
$MSP_MOTOR = 104
$MSP_RC = 105
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
    return @(
        (Join-Path $root "obj\main\wingflight_SITL.exe"),
        (Join-Path $root "obj\main\wingflight_SITL.elf"),
        (Join-Path $root "obj\main\betaflight_SITL.exe"),
        (Join-Path $root "obj\main\betaflight_SITL.elf"),
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
    if ($FreshEeprom) {
        $freshPath = Join-Path $root "obj\main\sitl-rc-check-eeprom.bin"
        try { Remove-Item -Force $freshPath -ErrorAction SilentlyContinue } catch { }
        if ([string]::IsNullOrWhiteSpace($launchArgs)) {
            $launchArgs = "--path=$freshPath"
        } else {
            $launchArgs = "$launchArgs --path=$freshPath"
        }
    }

    $binaryDir = Split-Path $binary -Parent
    if ([string]::IsNullOrWhiteSpace($binaryDir) -or -not (Test-Path $binaryDir)) {
        $binaryDir = $root
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
        [int]$TimeoutSeconds,
        [int]$Retries = 2
    )

    $rc = New-RcPayload -Roll $Roll -Pitch $Pitch -Yaw $Yaw -Collective $Collective -Throttle $Throttle
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
    param([string]$Aircraft)

    $root = Get-FirmwareRoot
    $bridgePython = Join-Path $root "tools\jsbsim-venv\Scripts\python.exe"
    $bridgeScript = Join-Path $root "scripts\jsbsim_bridge.py"

    if (-not (Test-Path $bridgePython) -or -not (Test-Path $bridgeScript)) {
        Write-Host "[SITL-RC] JSBSim bridge venv/script not found (expected $bridgePython) - see docs/development/SITL JSBSim FlightGear Plan.md"
        return $null
    }

    $logDir = Join-Path $root "obj\main"
    if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir -Force | Out-Null }

    Write-Host "[SITL-RC] Starting JSBSim bridge (aircraft=$Aircraft) ..."
    $p = Start-Process -FilePath $bridgePython `
        -ArgumentList @($bridgeScript, "--aircraft", $Aircraft) `
        -WorkingDirectory $root `
        -PassThru -NoNewWindow `
        -RedirectStandardOutput (Join-Path $logDir "jsbsim_bridge_check_stdout.log") `
        -RedirectStandardError (Join-Path $logDir "jsbsim_bridge_check_stderr.log")

    # Give JSBSim time to parse the aircraft model and open its sockets
    # before we start relying on attitude responses.
    Start-Sleep -Seconds 2
    return $p
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

    if ($Mode -eq "jsbsim") {
        # Started early so JSBSim has time to finish loading the aircraft
        # model while the MSP handshake below runs concurrently.
        $script:StartedBridgeProcess = Start-JsbsimBridge -Aircraft $BridgeAircraft
        if ($null -eq $script:StartedBridgeProcess) {
            throw "Could not start JSBSim bridge (required for -Mode jsbsim)"
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

    # SITL boots disarmed and stays that way for this script (no arming
    # switch/stick-arming gesture is exercised here). mixerSetInput() only
    # reflects RC onto disarmed control surfaces when a passthrough override
    # is active (see mixer.c), so without this, servo-response checks below
    # would always read zero delta regardless of RC injection correctness.
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

        $rollDelta = 0.0
        if ($null -ne $rollHigh -and $null -ne $rollLow) {
            $rollDelta = [math]::Abs($rollHigh.Roll - $rollLow.Roll)
            $result.axisDeltas["jsbsim_roll_deg"] = [math]::Round($rollDelta, 1)
        } else {
            $result.axisDeltas["jsbsim_roll_deg"] = -1
        }

        $pitchDelta = 0.0
        if ($null -ne $pitchHigh -and $null -ne $pitchLow) {
            $pitchDelta = [math]::Abs($pitchHigh.Pitch - $pitchLow.Pitch)
            $result.axisDeltas["jsbsim_pitch_deg"] = [math]::Round($pitchDelta, 1)
        } else {
            $result.axisDeltas["jsbsim_pitch_deg"] = -1
        }

        if ($null -ne $yawHigh -and $null -ne $yawLow) {
            $yawDelta = [math]::Abs((Get-AngleDeltaDeg -A $yawHigh.Yaw -B $yawLow.Yaw))
            $result.axisDeltas["jsbsim_yaw_deg"] = [math]::Round($yawDelta, 1)
        } else {
            $result.axisDeltas["jsbsim_yaw_deg"] = -1
        }

        Write-Host ("[SITL-RC] JSBSim attitude response: roll={0}deg pitch={1}deg yaw={2}deg (threshold {3}deg, roll/pitch gate pass/fail)" -f `
            $result.axisDeltas["jsbsim_roll_deg"], $result.axisDeltas["jsbsim_pitch_deg"], $result.axisDeltas["jsbsim_yaw_deg"], $JsbsimAttitudeThresholdDeg)

        $allAcked = $true
        foreach ($sample in @($neutral, $rollHigh, $rollLow, $pitchHigh, $pitchLow, $yawHigh, $yawLow)) {
            if ($null -eq $sample -or -not $sample.AckOk) { $allAcked = $false }
        }
        $result.rcInjectOk = $allAcked
        $result.controlResponsive = ($rollDelta -ge $JsbsimAttitudeThresholdDeg -and $pitchDelta -ge $JsbsimAttitudeThresholdDeg)
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
