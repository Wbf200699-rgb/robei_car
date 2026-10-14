param(
    [string]$ServerHost = "auto",
    [int]$Port = 18765,
    [switch]$Foreground,
    [switch]$SkipInstall
)

$ErrorActionPreference = "Stop"
$runner = Join-Path (Split-Path -Parent $PSScriptRoot) "uwb_portable.py"
$arguments = @($runner, "server-start", "--server-host", $ServerHost, "--port", $Port)
if ($Foreground) { $arguments += "--foreground" }
if ($SkipInstall) { $arguments += "--skip-install" }
& python @arguments
if ($LASTEXITCODE -ne 0) { throw "UWB relay deployment failed" }
