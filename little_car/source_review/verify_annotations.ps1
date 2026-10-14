$ErrorActionPreference = "Stop"

$verilogDir = Join-Path $PSScriptRoot "verilog"
$files = Get-ChildItem -LiteralPath $verilogDir -Filter "*.v"
if ($files.Count -eq 0) {
    throw "No exported Verilog files found in $verilogDir"
}

$checks = @(
    [pscustomobject]@{ Name = 'declarations'; Pattern = '^\s*(?:\(\*.*\*\)\s*)?(input|output|inout|reg|wire|integer)\b'; Marker = '//'; SameLine = $true }
    [pscustomobject]@{ Name = 'constants'; Pattern = '^\s*(localparam|parameter)\b'; Marker = '//'; SameLine = $true }
    [pscustomobject]@{ Name = 'assigns'; Pattern = '^\s*assign\s+'; Marker = '\[ASSIGN\]'; SameLine = $false }
    [pscustomobject]@{ Name = 'always'; Pattern = '^\s*always(?:\s+@|\s+#)'; Marker = '\[PROCESS\]'; SameLine = $false }
    [pscustomobject]@{ Name = 'functions'; Pattern = '^\s*function\b'; Marker = '\[FUNCTION\]'; SameLine = $false }
    [pscustomobject]@{ Name = 'tasks'; Pattern = '^\s*task\b'; Marker = '\[TASK\]'; SameLine = $false }
    [pscustomobject]@{ Name = 'initial'; Pattern = '^\s*initial\b'; Marker = '\[INITIAL\]'; SameLine = $false }
)

$missing = [System.Collections.Generic.List[string]]::new()
foreach ($check in $checks) {
    $total = 0
    $categoryMissing = 0

    foreach ($file in $files) {
        $lines = @(Get-Content -LiteralPath $file.FullName -Encoding UTF8)
        for ($lineIndex = 0; $lineIndex -lt $lines.Count; $lineIndex++) {
            if ($lines[$lineIndex] -notmatch $check.Pattern) {
                continue
            }

            $total++
            if ($check.SameLine) {
                $documented = $lines[$lineIndex] -match $check.Marker
            } else {
                $start = [Math]::Max(0, $lineIndex - 8)
                $window = if ($lineIndex -gt 0) {
                    $lines[$start..($lineIndex - 1)] -join "`n"
                } else {
                    ""
                }
                $documented = $window -match $check.Marker
            }

            if (-not $documented) {
                $categoryMissing++
                $missing.Add("$($check.Name) $($file.Name):$($lineIndex + 1): $($lines[$lineIndex])")
            }
        }
    }

    $documentedCount = $total - $categoryMissing
    Write-Host "$($check.Name) total=$total documented=$documentedCount missing=$categoryMissing"
}

if ($missing.Count -ne 0) {
    $missing | ForEach-Object { Write-Error $_ }
    throw "Annotation coverage check failed with $($missing.Count) missing entries"
}

Write-Host "ANNOTATION_CHECK_PASS"
