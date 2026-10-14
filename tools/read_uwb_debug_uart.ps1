param(
    [string]$PortName,
    [ValidateRange(1, 600)]
    [int]$Seconds = 10,
    [ValidateRange(500, 10000)]
    [int]$ProbeMilliseconds = 2500,
    [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Get-FtdiPorts {
    $ports = Get-CimInstance Win32_PnPEntity |
        Where-Object {
            $_.PNPDeviceID -match 'VID_0403(?:&|\+)PID_6011' -and
            $_.Name -match '\((COM[0-9]+)\)'
        } |
        ForEach-Object { [regex]::Match($_.Name, '\((COM[0-9]+)\)').Groups[1].Value }
    return @($ports | Sort-Object { [int]($_ -replace '[^0-9]', '') } -Unique)
}

function New-DebugSerialPort([string]$Name, [int]$ReadTimeout = 250) {
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

function Convert-UwbDebugFrame([string]$Line) {
    $match = [regex]::Match(
        $Line.Trim(),
        '^@U([0-9A-Fa-f]{4}),([0-9A-Fa-f]{4}),([0-9A-Fa-f]{4}),([01]),([01]),([0-9A-Fa-f]{2}),([0-3])$'
    )
    if (-not $match.Success) {
        return $null
    }

    return [pscustomobject][ordered]@{
        host_time = [DateTime]::Now.ToString('yyyy-MM-dd HH:mm:ss.fff')
        rx_packet_count = [Convert]::ToUInt16($match.Groups[1].Value, 16)
        rx_crc_error_count = [Convert]::ToUInt16($match.Groups[2].Value, 16)
        tx_packet_count = [Convert]::ToUInt16($match.Groups[3].Value, 16)
        uwb_link_online = [int]$match.Groups[4].Value
        last_ack_valid = [int]$match.Groups[5].Value
        last_sequence = [Convert]::ToByte($match.Groups[6].Value, 16)
        last_result = [int]$match.Groups[7].Value
        raw = $Line.Trim()
    }
}

function Test-UwbDebugPort([string]$Name, [int]$Milliseconds) {
    $port = $null
    try {
        $port = New-DebugSerialPort $Name 100
        $port.Open()
        $port.DiscardInBuffer()
        $buffer = ''
        $deadline = [DateTime]::UtcNow.AddMilliseconds($Milliseconds)
        while ([DateTime]::UtcNow -lt $deadline) {
            Start-Sleep -Milliseconds 50
            $buffer += $port.ReadExisting()
            if ($buffer -match '@U[0-9A-Fa-f]{4},[0-9A-Fa-f]{4},[0-9A-Fa-f]{4},[01],[01],[0-9A-Fa-f]{2},[0-3]') {
                return $true
            }
            if ($buffer.Length -gt 2048) {
                $buffer = $buffer.Substring($buffer.Length - 512)
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

if ($SelfTest) {
    $decoded = Convert-UwbDebugFrame '@U1234,ABCD,00EF,1,1,B2,0'
    if ($null -eq $decoded -or
        $decoded.rx_packet_count -ne 0x1234 -or
        $decoded.rx_crc_error_count -ne 0xABCD -or
        $decoded.tx_packet_count -ne 0x00EF -or
        $decoded.uwb_link_online -ne 1 -or
        $decoded.last_ack_valid -ne 1 -or
        $decoded.last_sequence -ne 0xB2 -or
        $decoded.last_result -ne 0) {
        throw 'UWB debug UART parser self-test failed.'
    }
    Write-Host 'RESULT TEST_PASS uwb_debug_uart_parser'
    return
}

if ([string]::IsNullOrWhiteSpace($PortName)) {
    $candidates = @(Get-FtdiPorts)
    if ($candidates.Count -eq 0) {
        throw 'No FT4232H virtual COM ports were found.'
    }

    Write-Host "Probing FT4232H ports: $($candidates -join ', ')"
    foreach ($candidate in $candidates) {
        Write-Host "  $candidate ..." -NoNewline
        if (Test-UwbDebugPort $candidate $ProbeMilliseconds) {
            Write-Host ' found @U telemetry'
            $PortName = $candidate
            break
        }
        Write-Host ' no frame'
    }
    if ([string]::IsNullOrWhiteSpace($PortName)) {
        throw 'No @U telemetry was found. Program the updated main bitstream and retry.'
    }
}

$captureDirectory = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\little_car\build\uwb_debug_capture')
)
New-Item -ItemType Directory -Force -Path $captureDirectory | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$rawPath = Join-Path $captureDirectory "uwb_debug_${stamp}.log"
$csvPath = Join-Path $captureDirectory "uwb_debug_${stamp}.csv"
$records = [System.Collections.Generic.List[object]]::new()

$serial = New-DebugSerialPort $PortName 300
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
        $record = Convert-UwbDebugFrame $line
        if ($null -ne $record) {
            $records.Add($record)
            Write-Host (
                'rx={0,5} crc={1,5} tx={2,5} link={3} ack={4} seq=0x{5:X2} result={6}' -f
                $record.rx_packet_count,
                $record.rx_crc_error_count,
                $record.tx_packet_count,
                $record.uwb_link_online,
                $record.last_ack_valid,
                $record.last_sequence,
                $record.last_result
            )
        }
    }
}
finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}

if ($records.Count -eq 0) {
    throw "No valid @U frame was decoded. Raw input, if any, is in $rawPath"
}

$records | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding utf8
$first = $records[0]
$last = $records[$records.Count - 1]
Write-Host "RESULT frames=$($records.Count) port=$PortName"
Write-Host "RESULT rx_delta=$(($last.rx_packet_count - $first.rx_packet_count + 65536) % 65536)"
Write-Host "RESULT crc_delta=$(($last.rx_crc_error_count - $first.rx_crc_error_count + 65536) % 65536)"
Write-Host "RESULT tx_delta=$(($last.tx_packet_count - $first.tx_packet_count + 65536) % 65536)"
Write-Host "RESULT csv=$csvPath"
