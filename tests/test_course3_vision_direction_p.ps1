$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$tuningPath = Join-Path $repoRoot 'project/code/course3_tuning.h'
$controlPath = Join-Path $repoRoot 'project/code/control.c'

$tuning = Get-Content -LiteralPath $tuningPath -Raw
$control = Get-Content -LiteralPath $controlPath -Raw

if ($tuning -notmatch '#define\s+COURSE3_VISION_DIRECTION_P\s+\([0-9]+(?:\.[0-9]+)?f\)') {
    throw 'COURSE3_VISION_DIRECTION_P is not defined in course3_tuning.h.'
}

if ($tuning -notmatch '#define\s+COURSE3_VISION_DIRECTION_I\s+\([0-9]+(?:\.[0-9]+)?f\)') {
    throw 'COURSE3_VISION_DIRECTION_I is not defined in course3_tuning.h.'
}

if ($tuning -notmatch '#define\s+COURSE3_VISION_DIRECTION_D\s+\([0-9]+(?:\.[0-9]+)?f\)') {
    throw 'COURSE3_VISION_DIRECTION_D is not defined in course3_tuning.h.'
}

if ($tuning -notmatch '#define\s+COURSE3_VISION_TURN_PWM_LIMIT\s+\([0-9]+(?:\.[0-9]+)?f\)') {
    throw 'COURSE3_VISION_TURN_PWM_LIMIT is not defined in course3_tuning.h.'
}

if ($control -match 'vision_control_pd_output\(yaw_error,\s*yaw_rate,\s*COURSE3_VISION_DIRECTION_P,\s*Direction_d,\s*2200\.0f\)') {
    throw 'Vision PD control still hard-codes the PWM output limit.'
}

if ($control -match 'vision_control_pd_output\(yaw_error,\s*yaw_rate,\s*COURSE3_VISION_DIRECTION_P,\s*Direction_d,\s*COURSE3_VISION_TURN_PWM_LIMIT\)') {
    throw 'Vision direction control still uses global Direction_d.'
}

if ($control -notmatch 'COURSE3_VISION_DIRECTION_P\s*\*\s*yaw_error' -or
    $control -notmatch 'COURSE3_VISION_DIRECTION_I\s*\*\s*g_turn_yaw_integral' -or
    $control -notmatch 'vision_yaw_rate\s*=\s*-yaw_rate' -or
    $control -notmatch 'COURSE3_VISION_DIRECTION_D\s*\*\s*vision_yaw_rate') {
    throw 'Vision direction control does not use the dedicated vision PID constants.'
}

if ($control -notmatch 'target_angle\s*=\s*vision_wrap_angle\(IMU_data\.filter_result\.yaw\s*\+\s*core_b_cmd\.vision_angle_offset_deg\)') {
    throw 'Vision calibration does not apply the pixel correction relative to the current yaw.'
}

if ($control -match 'g_vision_reference_yaw') {
    throw 'Vision calibration still uses a fixed entry yaw, which permits a non-centered visual equilibrium.'
}

Write-Output 'course3 vision Direction_P tuning test passed'
