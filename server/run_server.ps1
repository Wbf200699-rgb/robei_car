param(
    [string]$ListenAddress = "127.0.0.1",
    [int]$Port = 8765
)

$ErrorActionPreference = "Stop"
$serverRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$envFile = Join-Path $serverRoot ".env"

if (Test-Path -LiteralPath $envFile) {
    foreach ($line in Get-Content -LiteralPath $envFile) {
        $trimmed = $line.Trim()
        if (-not $trimmed -or $trimmed.StartsWith("#")) {
            continue
        }
        $name, $value = $trimmed -split "=", 2
        if ($name -and $null -ne $value) {
            Set-Item -Path "Env:$($name.Trim())" -Value $value.Trim()
        }
    }
}

$python = Join-Path $serverRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python)) {
    throw "Missing server/.venv. Run: python -m venv server/.venv; server/.venv/Scripts/python.exe -m pip install -r server/requirements.txt"
}

Push-Location $serverRoot
try {
    & $python -m uvicorn app.main:app --host $ListenAddress --port $Port
} finally {
    Pop-Location
}
