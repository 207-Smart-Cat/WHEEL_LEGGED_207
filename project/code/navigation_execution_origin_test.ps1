$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$trackingPath = Join-Path $projectRoot "code/navigation_tracking.c"
$tracking = Get-Content -LiteralPath $trackingPath -Raw

$executionStart = $tracking.IndexOf('else if (navi_ctrl.navi_mode_driver == 1 && last_mode_driver != 1)')
if ($executionStart -lt 0) {
    throw "Execution-mode transition block was not found."
}

$executionEnd = $tracking.IndexOf('else if (navi_ctrl.navi_mode_driver == 2)', $executionStart)
if ($executionEnd -lt 0) {
    throw "Execution-mode transition block end was not found."
}

$executionBlock = $tracking.Substring($executionStart, $executionEnd - $executionStart)

if ($executionBlock -notmatch 'Navi_Data_Set_Origin\(1\)') {
    throw "Execution start must reset the yaw origin with Navi_Data_Set_Origin(1)."
}

if ($executionBlock -match 'Navi_Data_Set_Origin\(0\)') {
    throw "Execution start still preserves the old yaw origin."
}

Write-Host "navigation execution yaw-origin reset checks passed"
