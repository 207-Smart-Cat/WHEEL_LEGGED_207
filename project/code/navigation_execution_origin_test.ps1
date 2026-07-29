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

if ($executionBlock -notmatch 'Navi_Yaw_Calibration_Start\(NAVI_YAW_CAL_CONTEXT_NAV_START\)') {
    throw "Execution start must begin a fresh yaw calibration."
}

if ($executionBlock -match 'Navi_Data_Set_Origin\([01]\)') {
    throw "Execution transition must not reset pose before yaw calibration completes."
}

$completionStart = $tracking.IndexOf('if (navi_start_cal_pending)')
if ($completionStart -lt 0) {
    throw "Yaw-calibration completion block was not found."
}

$completionEnd = $tracking.IndexOf('//            //', $completionStart)
if ($completionEnd -lt 0) {
    throw "Yaw-calibration completion block end was not found."
}

$completionBlock = $tracking.Substring($completionStart, $completionEnd - $completionStart)
if ($completionBlock -notmatch 'Navi_Yaw_Calibration_Consume_Done\(NAVI_YAW_CAL_CONTEXT_NAV_START\)') {
    throw "Navigation must wait for the fresh yaw calibration result."
}

if ($completionBlock -notmatch 'Navi_Data_Set_Origin\(0\)') {
    throw "Navigation must reset pose while preserving the freshly calibrated yaw origin."
}

if ($completionBlock -match 'Navi_Data_Set_Origin\(1\)') {
    throw "Navigation must not replace the fresh calibration with the rolling yaw history."
}

Write-Host "navigation fresh yaw-calibration checks passed"
