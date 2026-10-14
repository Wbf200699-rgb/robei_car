param(
    [string]$PortName = "",
    [int]$Seconds = 20,
    [string]$OutputPath = ""
)

$ErrorActionPreference = "Stop"

if (-not $PortName) {
    $ft4232ChannelC = @(
        Get-CimInstance Win32_PnPEntity |
            Where-Object {
                $_.PNPDeviceID -match '^FTDIBUS\\VID_0403\+PID_6011\+0ZZC01C\\'
            } |
            ForEach-Object {
                if ($_.Name -match '\((COM\d+)\)') { $Matches[1] }
            }
    )
    if ($ft4232ChannelC.Count -ne 1) {
        throw "Expected FT4232H channel C debug UART, found $($ft4232ChannelC.Count): $($ft4232ChannelC -join ', ')"
    }
    $PortName = $ft4232ChannelC[0]
}

if (-not $OutputPath) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $OutputPath = Join-Path $PSScriptRoot "..\little_car\test_results\sd_rw_hardware_$stamp.txt"
}
$OutputPath = [System.IO.Path]::GetFullPath($OutputPath)
[System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName($OutputPath)) | Out-Null

$serial = [System.IO.Ports.SerialPort]::new(
    $PortName,
    115200,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 500
$serial.NewLine = "`n"

$lines = [System.Collections.Generic.List[string]]::new()
$validReports = [System.Collections.Generic.List[string]]::new()
$deadline = [DateTime]::UtcNow.AddSeconds($Seconds)

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    while ([DateTime]::UtcNow -lt $deadline -and $validReports.Count -lt 2) {
        try {
            $line = $serial.ReadLine().TrimEnd("`r")
        } catch [System.TimeoutException] {
            continue
        }
        if (-not $line) {
            continue
        }
        $lines.Add($line)
        Write-Host $line
        if ($line.StartsWith("@SDW,")) {
            $fields = $line.Split(',')
            if ($fields.Count -eq 16 -and
                $fields[1] -match '^[01]{3}$' -and
                $fields[2] -eq '1' -and
                $fields[3] -eq '1' -and
                $fields[4] -eq '1' -and
                $fields[5] -eq '00' -and
                $fields[7] -match '^[0-9A-F]{2}$' -and
                (([Convert]::ToByte($fields[7], 16) -band 0x1f) -eq 0x05) -and
                $fields[8] -eq '1' -and
                $fields[10] -eq '00000C00' -and
                $fields[11] -eq '0000' -and
                $fields[14] -eq $fields[15]) {
                $validReports.Add($line)
            }
        }
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
    [System.IO.File]::WriteAllLines($OutputPath, $lines)
}

if ($validReports.Count -lt 2) {
    throw "Did not receive two valid @SDW reports. Raw capture: $OutputPath"
}

Write-Host "RESULT port=$PortName"
Write-Host "RESULT valid_reports=$($validReports.Count)"
Write-Host "RESULT capture=$OutputPath"
Write-Host "RESULT TEST_PASS sd_rw_hardware"
