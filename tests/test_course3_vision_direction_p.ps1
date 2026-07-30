$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$tuning = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/course3_tuning.h') -Raw
$control = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/control.c') -Raw
$vision = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/vision_control.c') -Raw
$calibration = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/vision_align_calibration.c') -Raw
$camera = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/camera_assist.c') -Raw
$display = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/camera_test_display.c') -Raw
$main = Get-Content -LiteralPath (Join-Path $repoRoot 'project/user/main_cm7_0.c') -Raw

$requiredMacros = @(
    'COURSE3_VISION_YAW_RATE_SIGN',
    'COURSE3_VISION_DIRECTION_P',
    'COURSE3_VISION_DIRECTION_I',
    'COURSE3_VISION_DIRECTION_D',
    'COURSE3_VISION_CONTROL_DT_S',
    'COURSE3_VISION_I_ERROR_MIN_DEG',
    'COURSE3_VISION_I_ERROR_MAX_DEG',
    'COURSE3_VISION_I_YAW_RATE_MAX_DPS',
    'COURSE3_VISION_I_OUTPUT_LIMIT',
    'COURSE3_VISION_TURN_PWM_LIMIT',
    'VISION_MAX_ANGLE_OFFSET_DEG'
)

foreach ($macro in $requiredMacros) {
    if ($tuning -notmatch "#define\s+$macro\s+\(") {
        throw "$macro is not defined in course3_tuning.h."
    }
}

if ($tuning -match 'VISION_FILTER_(OLD|NEW)' -or $vision -match 'VISION_FILTER_(OLD|NEW)') {
    throw 'The second Vision angle filter still exists.'
}

if ($camera -notmatch 'vision_control_update\(&camera_vision_control,\s*camera_assist_status\.lane_error_px\)') {
    throw 'Vision angle mapping does not use the filtered lane-center error.'
}

if ($camera -match 'vision_control_update\(&camera_vision_control,\s*raw_lane_error_px\)') {
    throw 'Raw lane error is still used as the control input.'
}

if ($vision -notmatch 'state->filtered_offset_deg\s*=\s*state->raw_offset_deg' -or
    $vision -notmatch 'return\s+state->raw_offset_deg\s*;') {
    throw 'Vision angle output is not a direct bounded pixel mapping.'
}

if ($control -notmatch 'COURSE3_VISION_YAW_RATE_SIGN\s*\*\s*raw_yaw_rate') {
    throw 'Vision yaw-rate sign is not controlled by the tuning macro.'
}

if ($control -match 'COURSE3_VISION_DIRECTION_I\s*\*\s*g_turn_yaw_integral') {
    throw 'Vision still reuses the ordinary navigation integral.'
}

if ($control -notmatch 'vision_turn_control_update\(&g_vision_turn_control') {
    throw 'The independent conditional Vision PID is not connected.'
}

if ($main -notmatch 'pit_ms_init\(PIT_Balance,\s*1\)') {
    throw 'The configured balance-control period is not 1 ms.'
}

if ($tuning -notmatch '#define\s+COURSE3_VISION_CONTROL_DT_S\s+\(0\.001f\)') {
    throw 'Vision PID dt does not match the configured 1 ms balance period.'
}

if ($tuning -notmatch '#define\s+COURSE3_VISION_I_ERROR_MAX_DEG\s+\(5\.00f\)') {
    throw 'Vision integral error upper bound is not 5 degrees.'
}

if ($tuning -notmatch '#define\s+VISION_ALIGN_SAMPLE_COUNT_TARGET\s+\(14U\)' -or
    $tuning -notmatch '#define\s+VISION_ALIGN_COMPLETE_TIMEOUT_MS\s+\(800U\)' -or
    $tuning -match 'VISION_ALIGN_SIDE_SAMPLE_TARGET') {
    throw 'Vision alignment completion parameters are not consecutive 14 frames with an 800 ms timeout.'
}

if ($calibration -match 'VISION_ALIGN_SIDE_SAMPLE_TARGET' -or
    $calibration -notmatch 'if\s*\(!within_sample\)[\s\S]*?cal->sample_count\s*=\s*0U' -or
    $calibration -notmatch 'cal->elapsed_ms\s*>=\s*VISION_ALIGN_COMPLETE_TIMEOUT_MS') {
    throw 'Vision alignment completion logic still depends on side quotas or lacks consecutive/timeout handling.'
}

if (([regex]::Matches($display, 'Lim L:%\+5\.0f R:%\+5\.0f')).Count -ne 2 -or
    ([regex]::Matches($display, 'COURSE3_VISION_TURN_PWM_LIMIT,\s*-COURSE3_VISION_TURN_PWM_LIMIT')).Count -ne 2 -or
    $display -match 'Trn L:') {
    throw 'Vision screen does not show the configured positive and negative turn limits.'
}

$resetCount = ([regex]::Matches($control, 'Vision_Turn_Reset\(\)')).Count
if ($resetCount -lt 8) {
    throw "Vision reset coverage is incomplete: found $resetCount reset calls."
}

Write-Output 'course3 Vision PID and single-filter structure test passed'
