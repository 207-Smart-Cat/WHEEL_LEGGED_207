$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$controlPath = Join-Path $repoRoot 'project/code/control.c'
$actionPath = Join-Path $repoRoot 'project/code/navigation_action.c'
$tuningPath = Join-Path $repoRoot 'project/code/course3_tuning.h'

$control = Get-Content -LiteralPath $controlPath -Raw
$action = Get-Content -LiteralPath $actionPath -Raw
$tuning = Get-Content -LiteralPath $tuningPath -Raw

if ($tuning -notmatch '#define\s+COURSE3_VISION_CAL_SPEED\s+\([0-9]+(?:\.[0-9]+)?f\)') {
    throw 'COURSE3_VISION_CAL_SPEED must be defined in course3_tuning.h.'
}

if ($tuning -notmatch '#define\s+VISION_MENU_CAL_SPEED\s+\(100\.0f\)') {
    throw 'VISION_MENU_CAL_SPEED must keep the manual Vision page at the configured test speed.'
}

if ($control -match 'target_velocity\s*=\s*50\.0f\s*;') {
    throw 'control.c still hard-codes vision calibration speed as 50.0f.'
}

if ($control -notmatch 'target_velocity\s*=\s*COURSE3_VISION_CAL_SPEED\s*;') {
    if ($control -notmatch 'manual_cal_enabled\s*\?\s*VISION_MENU_CAL_SPEED\s*:\s*COURSE3_VISION_CAL_SPEED') {
        throw 'Menu and course3 vision calibration speeds are not selected independently.'
    }
}

if ($control -match 'VisionAlignCal_UpdateContinuous\s*\(') {
    throw 'Course3 visual guidance must not collect or use calibrated yaw batches.'
}

if ($action -notmatch 'course3_vision_segment_get_map_target_yaw\s*\(entry_idx,\s*end_idx\)' -or
    $action -notmatch 'navi_get_two_points_azimuth\s*\(' -or
    $action -match 'course3_vision_segment\.latest_yaw') {
    throw 'Bridge/ramp action yaw must come from the P2-to-P3 inertial map direction.'
}

if ($action -notmatch 'anti_stall_saved' -or
    $action -notmatch 'Runtime_Is_Module_Enabled\(RUNTIME_MODULE_ANTI_STALL\)' -or
    $action -notmatch 'Runtime_Set_Module_Enabled\(RUNTIME_MODULE_ANTI_STALL,\s*1U\)' -or
    $action -notmatch 'Runtime_Set_Module_Enabled\(RUNTIME_MODULE_ANTI_STALL,\s*course3_aux_segment\.saved_anti_stall_enabled\)') {
    throw 'Bump auxiliary segment does not save, enable, and restore Anti Stall Assist.'
}

Write-Output 'course3 bump assist and vision calibration speed test passed'
