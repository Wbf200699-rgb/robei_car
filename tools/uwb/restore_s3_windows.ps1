param(
    [string]$DevicePort = "auto",
    [switch]$CompileOnly,
    [switch]$SkipInstall
)

$ErrorActionPreference = "Stop"
$script = Join-Path $PSScriptRoot "flash_s3_windows.ps1"
& $script -ServerHost "relay.example.invalid" -ServerPort 443 `
    -DevicePort $DevicePort -Tls -CompileOnly:$CompileOnly `
    -SkipInstall:$SkipInstall
