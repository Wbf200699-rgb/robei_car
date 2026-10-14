$ErrorActionPreference = "Stop"
$runner = Join-Path (Split-Path -Parent $PSScriptRoot) "uwb_portable.py"
& python $runner server-stop
if ($LASTEXITCODE -ne 0) { throw "UWB relay stop failed" }
