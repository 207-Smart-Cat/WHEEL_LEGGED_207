$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$controlPath = Join-Path $repoRoot 'project/code/control.c'
$actionPath = Join-Path $repoRoot 'project/code/navigation_action.c'
$tuningPath = Join-Path $repoRoot 'project/code/course3_tuning.h'

$control = Get-Content -LiteralPath $controlPath -Raw
$action = Get-Content -LiteralPath $actionPath -Raw
$tuning = Get-Content -LiteralPath $tuningPath -Raw

if ($tuning -notmatch '#define\s+COURSE3_VISION_CAL_SPEED\s+\(50\.0f\)') {
    throw 'COURSE3_VISION_CAL_SPEED must be defined in course3_tuning.h.'
}

if ($control -match 'target_velocity\s*=\s*50\.0f\s*;') {
    throw 'control.c still hard-codes vision calibration speed as 50.0f.'
}

if ($control -notmatch 'target_velocity\s*=\s*COURSE3_VISION_CAL_SPEED\s*;') {
    throw 'control.c does not use COURSE3_VISION_CAL_SPEED for vision calibration speed.'
}

if ($action -notmatch 'anti_stall_saved' -or
    $action -notmatch 'Runtime_Is_Module_Enabled\(RUNTIME_MODULE_ANTI_STALL\)' -or
    $action -notmatch 'Runtime_Set_Module_Enabled\(RUNTIME_MODULE_ANTI_STALL,\s*1U\)' -or
    $action -notmatch 'Runtime_Set_Module_Enabled\(RUNTIME_MODULE_ANTI_STALL,\s*course3_aux_segment\.saved_anti_stall_enabled\)') {
    throw 'Bump auxiliary segment does not save, enable, and restore Anti Stall Assist.'
}

Write-Output 'course3 bump assist and vision calibration speed test passed'
