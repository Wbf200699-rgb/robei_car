param(
    [string]$PortName = 'COM20',
    [ValidateRange(5, 300)]
    [int]$Seconds = 30,
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_-]{0,31}$')]
    [string]$Experiment = 'stationary'
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Convert-HexSigned16([string]$Text) {
    $value = [Convert]::ToUInt16($Text, 16)
    if ($value -ge 32768) { return [int]$value - 65536 }
    return [int]$value
}

function Convert-HexUnsigned16([string]$Text) {
    return [Convert]::ToUInt16($Text, 16)
}

function Get-AxisStats($Rows, [string]$Name) {
    $values = @($Rows | ForEach-Object { [double]($_.$Name) })
    $mean = ($values | Measure-Object -Average).Average
    $minimum = ($values | Measure-Object -Minimum).Minimum
    $maximum = ($values | Measure-Object -Maximum).Maximum
    $variance = 0.0
    foreach ($value in $values) { $variance += ($value - $mean) * ($value - $mean) }
    $variance /= [Math]::Max(1, $values.Count - 1)
    return [ordered]@{
        mean = [Math]::Round($mean, 3)
        stddev = [Math]::Round([Math]::Sqrt($variance), 3)
        min = [int]$minimum
        max = [int]$maximum
        peak_to_peak = [int]($maximum - $minimum)
    }
}

$pattern = '^@I(?<seq>[0-9A-Fa-f]{4}),(?<status>[0-9A-Fa-f]{2}),' +
    '(?<calflags>[0-9A-Fa-f]{2}),(?<calaxes>[0-9A-Fa-f]{2})' +
    '(?:,(?<v>[0-9A-Fa-f]{4})){20}$'
$port = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 300
$port.DtrEnable = $false
$port.RtsEnable = $false
$rows = [System.Collections.Generic.List[object]]::new()
$raw = [System.Collections.Generic.List[string]]::new()

try {
    $port.Open()
    $port.DiscardInBuffer()
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line = $port.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        $raw.Add($line)
        $match = [regex]::Match($line, $pattern)
        if (-not $match.Success) { continue }
        $valueHex = @($match.Groups['v'].Captures | ForEach-Object { $_.Value })
        $values = @($valueHex | ForEach-Object {
            Convert-HexSigned16 $_
        })
        if ($values.Count -ne 20) { continue }
        $rows.Add([pscustomobject][ordered]@{
            host_time = [DateTime]::Now.ToString('yyyy-MM-dd HH:mm:ss.fff')
            sequence = [Convert]::ToUInt16($match.Groups['seq'].Value, 16)
            status_hex = $match.Groups['status'].Value.ToUpperInvariant()
            calibration_flags_hex = $match.Groups['calflags'].Value.ToUpperInvariant()
            calibration_axes_hex = $match.Groups['calaxes'].Value.ToUpperInvariant()
            bmi0_accel_x = $values[0]
            bmi0_accel_y = $values[1]
            bmi0_accel_z = $values[2]
            bmi0_gyro_x = $values[3]
            bmi0_gyro_y = $values[4]
            bmi0_gyro_z = $values[5]
            bmi1_accel_x = $values[6]
            bmi1_accel_y = $values[7]
            bmi1_accel_z = $values[8]
            bmi1_gyro_x = $values[9]
            bmi1_gyro_y = $values[10]
            bmi1_gyro_z = $values[11]
            static_cal_imu0_gyro_peak = Convert-HexUnsigned16 $valueHex[12]
            static_cal_imu1_gyro_peak = Convert-HexUnsigned16 $valueHex[13]
            static_cal_imu0_gyro_mean_x = $values[14]
            static_cal_imu0_gyro_mean_y = $values[15]
            static_cal_imu0_gyro_mean_z = $values[16]
            static_cal_imu1_gyro_mean_x = $values[17]
            static_cal_imu1_gyro_mean_y = $values[18]
            static_cal_imu1_gyro_mean_z = $values[19]
        })
    }
}
finally {
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}

if ($rows.Count -lt 20) { throw "Only $($rows.Count) valid @I frames were decoded" }

$captureDir = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\little_car\build\motion_sensor_capture'))
New-Item -ItemType Directory -Force -Path $captureDir | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$base = "dual_imu_$($Experiment.ToLowerInvariant())_$stamp"
$rawPath = Join-Path $captureDir "$base.log"
$csvPath = Join-Path $captureDir "$base.csv"
$jsonPath = Join-Path $captureDir "$base.json"
$raw | Set-Content -LiteralPath $rawPath -Encoding ascii
$rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding utf8

$axisNames = @(
    'bmi0_accel_x','bmi0_accel_y','bmi0_accel_z',
    'bmi0_gyro_x','bmi0_gyro_y','bmi0_gyro_z',
    'bmi1_accel_x','bmi1_accel_y','bmi1_accel_z',
    'bmi1_gyro_x','bmi1_gyro_y','bmi1_gyro_z')
$stats = [ordered]@{}
foreach ($name in $axisNames) { $stats[$name] = Get-AxisStats $rows $name }

function Get-DominantAccelAxis([string]$Prefix) {
    $candidates = @('x','y','z') | ForEach-Object {
        [pscustomobject]@{ axis = $_; magnitude = [Math]::Abs($stats["${Prefix}_accel_$_"].mean) }
    }
    return ($candidates | Sort-Object magnitude -Descending | Select-Object -First 1).axis
}

$g0 = Get-DominantAccelAxis 'bmi0'
$g1 = Get-DominantAccelAxis 'bmi1'
$sequenceGaps = 0
for ($i = 1; $i -lt $rows.Count; $i++) {
    $delta = ($rows[$i].sequence - $rows[$i - 1].sequence + 65536) % 65536
    if ($delta -ne 1) { $sequenceGaps++ }
}
$summary = [ordered]@{
    schema = 'little-car-dual-imu-static/v2'
    experiment = $Experiment.ToLowerInvariant()
    captured_at = (Get-Date).ToString('o')
    port = $PortName
    baud = 115200
    decoded_frames = $rows.Count
    sequence_gaps = $sequenceGaps
    status_values = @($rows | ForEach-Object status_hex | Sort-Object -Unique)
    calibration_flags_values = @($rows | ForEach-Object calibration_flags_hex | Sort-Object -Unique)
    calibration_axes_values = @($rows | ForEach-Object calibration_axes_hex | Sort-Object -Unique)
    bmi0_gravity_axis = $g0
    bmi1_gravity_axis = $g1
    axis_statistics = $stats
    artifacts = [ordered]@{ raw = $rawPath; csv = $csvPath }
}
$completedRows = @($rows | Where-Object calibration_flags_hex -ne '00')
if ($completedRows.Count -gt 0) {
    $lastCompleted = $completedRows[-1]
    $summary['last_completed_calibration'] = [ordered]@{
        flags_hex = $lastCompleted.calibration_flags_hex
        axes_hex = $lastCompleted.calibration_axes_hex
        imu0_gyro_peak = $lastCompleted.static_cal_imu0_gyro_peak
        imu1_gyro_peak = $lastCompleted.static_cal_imu1_gyro_peak
        imu0_gyro_mean = @($lastCompleted.static_cal_imu0_gyro_mean_x, $lastCompleted.static_cal_imu0_gyro_mean_y, $lastCompleted.static_cal_imu0_gyro_mean_z)
        imu1_gyro_mean = @($lastCompleted.static_cal_imu1_gyro_mean_x, $lastCompleted.static_cal_imu1_gyro_mean_y, $lastCompleted.static_cal_imu1_gyro_mean_z)
    }
}
$summary | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $jsonPath -Encoding utf8

Write-Host "RESULT frames=$($rows.Count) gaps=$sequenceGaps status=$($summary.status_values -join '/')"
Write-Host "RESULT onboard_cal_flags=$($summary.calibration_flags_values -join '/') axes=$($summary.calibration_axes_values -join '/')"
if ($completedRows.Count -gt 0) {
    Write-Host "RESULT onboard_cal_internal peak=$($lastCompleted.static_cal_imu0_gyro_peak),$($lastCompleted.static_cal_imu1_gyro_peak) mean0=$($lastCompleted.static_cal_imu0_gyro_mean_x),$($lastCompleted.static_cal_imu0_gyro_mean_y),$($lastCompleted.static_cal_imu0_gyro_mean_z) mean1=$($lastCompleted.static_cal_imu1_gyro_mean_x),$($lastCompleted.static_cal_imu1_gyro_mean_y),$($lastCompleted.static_cal_imu1_gyro_mean_z)"
}
Write-Host "RESULT bmi0_accel_mean=$($stats.bmi0_accel_x.mean),$($stats.bmi0_accel_y.mean),$($stats.bmi0_accel_z.mean) gravity=$g0"
Write-Host "RESULT bmi1_accel_mean=$($stats.bmi1_accel_x.mean),$($stats.bmi1_accel_y.mean),$($stats.bmi1_accel_z.mean) gravity=$g1"
Write-Host "RESULT bmi0_gyro_std=$($stats.bmi0_gyro_x.stddev),$($stats.bmi0_gyro_y.stddev),$($stats.bmi0_gyro_z.stddev)"
Write-Host "RESULT bmi1_gyro_std=$($stats.bmi1_gyro_x.stddev),$($stats.bmi1_gyro_y.stddev),$($stats.bmi1_gyro_z.stddev)"
Write-Host "CSV=$csvPath"
Write-Host "JSON=$jsonPath"
