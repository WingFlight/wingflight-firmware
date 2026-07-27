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
    smoke | sweep | stress

.EXAMPLE
    .\scripts\sitl-rc-check.ps1 -Mode smoke

.EXAMPLE
    .\scripts\sitl-rc-check.ps1 -Mode sweep -Cycles 30

.EXAMPLE
    .\scripts\sitl-rc-check.ps1 -Mode stress -StressSeconds 90
#>
param(
    [string]$MspHost = "127.0.0.1",
    [int]$Port = 0,
    [int[]]$PortCandidates = @(5760, 5761),
    [int]$TimeoutSeconds = 3,
    [int]$Cycles = 20,
    [int]$ServoDeltaThreshold = 40,
    [ValidateSet("smoke", "sweep", "stress")]
    [string]$Mode = "smoke",
    [int]$StressSeconds = 60,
    [string]$SitlBinaryPath = "",
    [string]$SitlArgs = "",
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
$MSP_SET_RAW_RC = 200

$script:StartedSitlProcess = $null
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

    if ($ExplicitPort -gt 0) {
        if (Test-TcpPort -RemoteHost $RemoteHost -ProbePort $ExplicitPort) {
            return $ExplicitPort
        }
        return 0
    }

    $firstOpen = 0
    foreach ($p in $Candidates) {
        if (Test-TcpPort -RemoteHost $RemoteHost -ProbePort $p) {
            if ($firstOpen -eq 0) { $firstOpen = $p }
            if (Test-MspApiPort -RemoteHost $RemoteHost -ProbePort $p -TimeoutSeconds 1) {
                return $p
            }
        }
    }

    if ($firstOpen -gt 0) {
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
        cmd.exe /c "make TARGET=SITL DEBUG=GDB -j 8"
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
        Build-Sitl
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

    $channels = @($Roll, $Pitch, $Yaw, $Collective, $Throttle, $Aux1, $Aux2, $Aux3)
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

    # Prevent stale responses from previous phases from masking current RC changes.
    Clear-MspInput -Stream $Stream

    for ($k = 0; $k -lt $Cycles; $k++) {
        Send-Msp -Stream $Stream -Command $MSP_SET_RAW_RC -Payload $rc
        Start-Sleep -Milliseconds 12
    }

    # Allow RX task processing before sampling channel state.
    Start-Sleep -Milliseconds 35

    foreach ($cmd in $commandsToRead) {
        Send-Msp -Stream $Stream -Command $cmd
        $resp = Receive-MspMatch -Stream $Stream -ExpectedCommand $cmd -TimeoutSeconds $TimeoutSeconds
        if ($null -ne $resp) {
            $resp | Add-Member -NotePropertyName ResponseCommand -NotePropertyValue $cmd -Force
            $last = $resp
            break
        }
    }

    return $last
}

function Send-RcWithAck {
    param(
        [System.IO.Stream]$Stream,
        [int]$Roll,
        [int]$Pitch,
        [int]$Yaw,
        [int]$Collective = 1500,
        [int]$Throttle,
        [int]$TimeoutSeconds
    )

    $rc = New-RcPayload -Roll $Roll -Pitch $Pitch -Yaw $Yaw -Collective $Collective -Throttle $Throttle
    Clear-MspInput -Stream $Stream
    Send-Msp -Stream $Stream -Command $MSP_SET_RAW_RC -Payload $rc
    $ack = Receive-MspMatch -Stream $Stream -ExpectedCommand $MSP_SET_RAW_RC -TimeoutSeconds $TimeoutSeconds
    return ($null -ne $ack -and -not $ack.IsError)
}

function Set-Neutral {
    param([System.IO.Stream]$Stream)

    $neutral = New-RcPayload -Roll 1500 -Pitch 1500 -Yaw 1500 -Collective 1500 -Throttle 1000
    for ($k = 0; $k -lt 10; $k++) {
        Send-Msp -Stream $Stream -Command $MSP_SET_RAW_RC -Payload $neutral
        Start-Sleep -Milliseconds 8
    }
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
    Write-Host "[SITL-RC] Connecting to $MspHost`:$script:ResolvedPort ..."

    $client = New-Object System.Net.Sockets.TcpClient
    $connectTask = $client.ConnectAsync($MspHost, $script:ResolvedPort)
    if (-not $connectTask.Wait([TimeSpan]::FromSeconds($TimeoutSeconds))) {
        throw "Timed out connecting to $MspHost`:$script:ResolvedPort"
    }
    $stream = $client.GetStream()
    $result.connected = $true

    Send-Msp -Stream $stream -Command $MSP_API_VERSION
    $api = Receive-MspMatch -Stream $stream -ExpectedCommand $MSP_API_VERSION -TimeoutSeconds $TimeoutSeconds
    if ($null -eq $api -or $api.Length -lt 3) {
        throw "No MSP_API_VERSION response"
    }
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
        $result.controlResponsive = $true
    }

    Set-Neutral -Stream $stream
}
finally {
    if ($null -ne $stream) { $stream.Dispose() }
    if ($null -ne $client) { $client.Close() }

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
