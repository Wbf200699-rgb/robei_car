param(
    [string]$PortName,
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_-]{0,31}$')]
    [string]$Experiment = 'capture',
    [ValidateRange(1, 600)]
    [int]$Seconds = 20,
    [ValidateRange(250, 5000)]
    [int]$ProbeMilliseconds = 1500,
    [switch]$Quiet,
    [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Get-FtdiPorts {
    $result = @()
    $devices = Get-CimInstance Win32_PnPEntity |
        Where-Object {
            $_.PNPDeviceID -match 'VID_0403(?:&|\+)PID_6011' -and
            $_.Name -match '\((COM[0-9]+)\)'
        }

    foreach ($device in $devices) {
        $match = [regex]::Match($device.Name, '\((COM[0-9]+)\)')
        if ($match.Success) {
            $result += $match.Groups[1].Value
        }
    }

    return @($result | Sort-Object { [int]($_ -replace '[^0-9]', '') } -Unique)
}

function New-SensorSerialPort([string]$Name, [int]$ReadTimeout = 200) {
    $port = [System.IO.Ports.SerialPort]::new(
        $Name,
        115200,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One
    )
    $port.Handshake = [System.IO.Ports.Handshake]::None
    $port.DtrEnable = $false
    $port.RtsEnable = $false
    $port.ReadTimeout = $ReadTimeout
    $port.NewLine = "`n"
    return $port
}

function Test-MotionSensorPort([string]$Name, [int]$Milliseconds) {
    $port = $null
    try {
        $port = New-SensorSerialPort $Name 100
        $port.Open()
        $port.DiscardInBuffer()
        $buffer = ''
        $deadline = [DateTime]::UtcNow.AddMilliseconds($Milliseconds)
        while ([DateTime]::UtcNow -lt $deadline) {
            Start-Sleep -Milliseconds 50
            $buffer += $port.ReadExisting()
            if ($buffer -match '@M[0-9A-Fa-f]{4},[0-9A-Fa-f]{2},') {
                return $true
            }
            if ($buffer.Length -gt 4096) {
                $buffer = $buffer.Substring($buffer.Length - 1024)
            }
        }
        return $false
    }
    catch {
        return $false
    }
    finally {
        if ($null -ne $port) {
            if ($port.IsOpen) { $port.Close() }
            $port.Dispose()
        }
    }
}

function Convert-HexSigned16([string]$Text) {
    $unsigned = [Convert]::ToUInt16($Text, 16)
    if ($unsigned -ge 32768) {
        return [int]$unsigned - 65536
    }
    return [int]$unsigned
}

function Convert-HexSigned32([string]$Text) {
    $unsigned = [Convert]::ToUInt32($Text, 16)
    if ($unsigned -ge 2147483648) {
        return [long]$unsigned - 4294967296
    }
    return [long]$unsigned
}

function Convert-MotionFrame([string]$Line) {
    $parts = $Line.Trim() -split ','
    $headerMatch = [regex]::Match($parts[0], '^@M([0-9A-Fa-f]{4})$')
    if ($parts.Count -ne 21 -or -not $headerMatch.Success) {
        return $null
    }

    for ($index = 1; $index -le 8; $index++) {
        if ($parts[$index] -notmatch '^[0-9A-Fa-f]{2}$') { return $null }
    }
    foreach ($index in @(9, 10, 13, 14, 15, 16, 17, 18, 19, 20)) {
        if ($parts[$index] -notmatch '^[0-9A-Fa-f]{4}$') { return $null }
    }
    foreach ($index in @(11, 12)) {
        if ($parts[$index] -notmatch '^[0-9A-Fa-f]{8}$') { return $null }
    }

    $sequence = [Convert]::ToUInt16($headerMatch.Groups[1].Value, 16)
    $status = [Convert]::ToByte($parts[1], 16)
    return [pscustomobject][ordered]@{
        host_time = [DateTime]::Now.ToString('yyyy-MM-dd HH:mm:ss.fff')
        sequence = $sequence
        status_hex = ('{0:X2}' -f $status)
        pmw_online = [int](($status -band 0x01) -ne 0)
        bmi0_online = [int](($status -band 0x02) -ne 0)
        bmi1_online = [int](($status -band 0x04) -ne 0)
        calibration_done = [int](($status -band 0x08) -ne 0)
        fused_ready = [int](($status -band 0x10) -ne 0)
        imu_degraded = [int](($status -band 0x20) -ne 0)
        imu_disagreement = [int](($status -band 0x40) -ne 0)
        flow_seen = [int](($status -band 0x80) -ne 0)
        squal = [Convert]::ToByte($parts[2], 16)
        pmw_id = $parts[3].ToUpperInvariant()
        pmw_inverse_id = $parts[4].ToUpperInvariant()
        bmi0_accel_id = $parts[5].ToUpperInvariant()
        bmi0_gyro_id = $parts[6].ToUpperInvariant()
        bmi1_accel_id = $parts[7].ToUpperInvariant()
        bmi1_gyro_id = $parts[8].ToUpperInvariant()
        flow_dx = Convert-HexSigned16 $parts[9]
        flow_dy = Convert-HexSigned16 $parts[10]
        flow_accum_x = Convert-HexSigned32 $parts[11]
        flow_accum_y = Convert-HexSigned32 $parts[12]
        accel_x = Convert-HexSigned16 $parts[13]
        accel_y = Convert-HexSigned16 $parts[14]
        accel_z = Convert-HexSigned16 $parts[15]
        gyro_x = Convert-HexSigned16 $parts[16]
        gyro_y = Convert-HexSigned16 $parts[17]
        gyro_z = Convert-HexSigned16 $parts[18]
        bmi0_raw_gyro_z = Convert-HexSigned16 $parts[19]
        bmi1_raw_gyro_z = Convert-HexSigned16 $parts[20]
    }
}

if ($SelfTest) {
    $sample = '@M0001,9F,2D,49,B6,1E,0F,1E,0F,0020,FFF8,00000060,FFFFFFD8,006E,FF42,1586,012C,0064,012C,012C,0064'
    $decoded = Convert-MotionFrame $sample
    if ($null -eq $decoded -or
        $decoded.sequence -ne 1 -or
        $decoded.flow_dx -ne 32 -or
        $decoded.flow_dy -ne -8 -or
        $decoded.flow_accum_x -ne 96 -or
        $decoded.flow_accum_y -ne -40 -or
        $decoded.pmw_online -ne 1 -or
        $decoded.imu_disagreement -ne 0 -or
        $decoded.flow_seen -ne 1) {
        throw 'Motion sensor UART parser self-test failed.'
    }
    Write-Host 'RESULT TEST_PASS motion_sensor_uart_parser'
    return
}

if ([string]::IsNullOrWhiteSpace($PortName)) {
    $candidates = @(Get-FtdiPorts)
    if ($candidates.Count -eq 0) {
        throw 'No FT4232H virtual COM ports were found. Check the USB cable and FTDI driver.'
    }

    Write-Host "Probing FT4232H ports: $($candidates -join ', ')"
    foreach ($candidate in $candidates) {
        Write-Host "  $candidate ..." -NoNewline
        if (Test-MotionSensorPort $candidate $ProbeMilliseconds) {
            Write-Host ' found @M telemetry'
            $PortName = $candidate
            break
        }
        Write-Host ' no frame'
    }

    if ([string]::IsNullOrWhiteSpace($PortName)) {
        throw 'No @M telemetry was found. Confirm the FPGA is programmed and J1 drives the USB-UART RX path.'
    }
}

$captureDirectory = Join-Path $PSScriptRoot '..\little_car\build\motion_sensor_capture'
$captureDirectory = [System.IO.Path]::GetFullPath($captureDirectory)
New-Item -ItemType Directory -Force -Path $captureDirectory | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$experimentSlug = $Experiment.ToLowerInvariant()
$baseName = "motion_sensor_${experimentSlug}_${stamp}"
$rawPath = Join-Path $captureDirectory "${baseName}.log"
$csvPath = Join-Path $captureDirectory "${baseName}.csv"
$metadataPath = Join-Path $captureDirectory "${baseName}.json"

$records = [System.Collections.Generic.List[object]]::new()
$serial = New-SensorSerialPort $PortName 300
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    Write-Host "Capturing $PortName at 115200 8N1 for $Seconds seconds..."
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $line = $serial.ReadLine().TrimEnd("`r")
        }
        catch [System.TimeoutException] {
            continue
        }

        Add-Content -LiteralPath $rawPath -Value $line -Encoding ascii
        $record = Convert-MotionFrame $line
        if ($null -ne $record) {
            $records.Add($record)
            if (-not $Quiet) {
                Write-Host (
                    'seq={0,5} st={1} ids={2}/{3},{4}/{5},{6}/{7} flow=({8,6},{9,6}) imu_g=({10,6},{11,6},{12,6})' -f
                    $record.sequence, $record.status_hex,
                    $record.pmw_id, $record.pmw_inverse_id,
                    $record.bmi0_accel_id, $record.bmi0_gyro_id,
                    $record.bmi1_accel_id, $record.bmi1_gyro_id,
                    $record.flow_dx, $record.flow_dy,
                    $record.gyro_x, $record.gyro_y, $record.gyro_z
                )
            }
        }
    }
}
finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}

if ($records.Count -eq 0) {
    throw "No valid motion-sensor frame was decoded. Raw input, if any, is in $rawPath"
}

$records | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding utf8
$last = $records[$records.Count - 1]
$sequenceGaps = 0
for ($index = 1; $index -lt $records.Count; $index++) {
    $delta = ($records[$index].sequence - $records[$index - 1].sequence + 65536) % 65536
    if ($delta -ne 1) { $sequenceGaps++ }
}
$metadata = [ordered]@{
    schema = 'little-car-motion-capture/v1'
    experiment = $experimentSlug
    captured_at = (Get-Date).ToString('o')
    port = $PortName
    baud = 115200
    requested_seconds = $Seconds
    decoded_frames = $records.Count
    first_sequence = $records[0].sequence
    last_sequence = $last.sequence
    sequence_gaps = $sequenceGaps
    status_values = @($records | ForEach-Object { $_.status_hex } | Sort-Object -Unique)
    raw_file = [System.IO.Path]::GetFileName($rawPath)
    csv_file = [System.IO.Path]::GetFileName($csvPath)
}
$metadata | ConvertTo-Json -Depth 3 | Set-Content -LiteralPath $metadataPath -Encoding utf8
Write-Host ''
Write-Host "Decoded frames : $($records.Count)"
Write-Host "Sequence gaps  : $sequenceGaps"
Write-Host "Last IDs       : $($last.pmw_id)/$($last.pmw_inverse_id), $($last.bmi0_accel_id)/$($last.bmi0_gyro_id), $($last.bmi1_accel_id)/$($last.bmi1_gyro_id)"
Write-Host "Last status    : 0x$($last.status_hex)"
Write-Host "Raw capture    : $rawPath"
Write-Host "Decoded CSV    : $csvPath"
Write-Host "Metadata       : $metadataPath"
