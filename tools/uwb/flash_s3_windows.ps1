param(
    [string]$ServerHost = "auto",
    [int]$ServerPort = 18765,
    [string]$DevicePort = "auto",
    [switch]$Tls,
    [switch]$CompileOnly,
    [switch]$SkipInstall
)

$ErrorActionPreference = "Stop"
$runner = Join-Path (Split-Path -Parent $PSScriptRoot) "uwb_portable.py"
$arguments = @(
    $runner, "flash-s3", "--server-host", $ServerHost,
    "--server-port", $ServerPort, "--device-port", $DevicePort
)
if ($CompileOnly) { $arguments += "--compile-only" }
if ($Tls) { $arguments += "--tls" }
if ($SkipInstall) { $arguments += "--skip-install" }
& python @arguments
if ($LASTEXITCODE -ne 0) { throw "ESP32-S3 flashing failed" }
