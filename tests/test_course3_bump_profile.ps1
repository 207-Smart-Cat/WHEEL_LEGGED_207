$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$logic = Get-Content (Join-Path $repoRoot 'project/code/bump_mode_logic.h') -Raw
$control = Get-Content (Join-Path $repoRoot 'project/code/control.c') -Raw
$action = Get-Content (Join-Path $repoRoot 'project/code/navigation_action.c') -Raw
$data = Get-Content (Join-Path $repoRoot 'project/code/navigation_data_handling.c') -Raw
$ipc = Get-Content (Join-Path $repoRoot 'project/code/ipc_shared_data.c') -Raw
$screen = [IO.File]::ReadAllText(
    (Join-Path $repoRoot 'project/code/screen_display.c'),
    [Text.Encoding]::GetEncoding(936))
$param = Get-Content (Join-Path $repoRoot 'project/code/param.h') -Raw
$core0Project = Get-Content (Join-Path $repoRoot 'project/iar/project_config/cyt4bb7_cm_7_0.ewp') -Raw
$core1Project = Get-Content (Join-Path $repoRoot 'project/iar/project_config/cyt4bb7_cm_7_1.ewp') -Raw

foreach ($expected in @(
    'BUMP_TARGET_SPEED_DEFAULT\s+\(314\.0f\)',
    'BUMP_ASSIST_PWM_LIMIT\s+\(6000\)',
    'BUMP_ACTIVE_DIRECTION_P\s+\(95\.0f\)',
    'BUMP_ACTIVE_INTEGRAL_GAIN\s+\(8\.0f\)',
    'BUMP_ACTIVE_PWM_GAIN\s+\(1\.5f\)',
    'BUMP_DISTANCE_COMPENSATION_M\s+\(1\.0f\)')) {
    if ($logic -notmatch $expected) { throw "Missing BUMP profile: $expected" }
}

if ($control -notmatch 'anti_stall_pwm_limit\s*=\s*4000\.0f' -or
    $control -notmatch 'anti_stall_pwm_limit\s*=\s*\(float\)BUMP_ASSIST_PWM_LIMIT' -or
    $control -notmatch 'BumpMode_ReverseAssistUpdate') {
    throw 'BUMP assist limit scoping or reverse assist is missing.'
}

if ($action -notmatch 'course3_segment_get_map_target_yaw\(start_idx,\s*end_idx\)' -or
    $action -notmatch 'Navi_Course3_Bump_Meter_Begin\([\s\S]*?BUMP_DISTANCE_COMPENSATION_M' -or
    $action -notmatch 'target_angle\s*=\s*course3_aux_segment\.target_yaw' -or
    $action -notmatch 'Navi_Data_Set_Position\(point_map\[completed_end_idx\]\.x') {
    throw 'Fixed-yaw BUMP waypoint travel or endpoint pose alignment is missing.'
}

if ($data -notmatch 'Course3TravelMeter_Update\(&course3_bump_meter' -or
    $data -notmatch 'robot_pose\.x\s*=\s*x;[\s\S]*?robot_pose\.y\s*=\s*y;') {
    throw 'BUMP travel meter or pose alignment implementation is missing.'
}

if ($screen -notmatch 'if\s*\(run_enabled\)[\s\S]*?Runtime_Set_Module_Enabled\(RUNTIME_MODULE_ANTI_STALL,\s*1U\)' -or
    $screen -notmatch 'Runtime_Set_Module_Enabled\(RUNTIME_MODULE_ANTI_STALL,[\s\S]*?ui_bump_saved_assist_enabled\)') {
    throw 'BUMP screen Run ON must enable assist and restore its prior state on exit.'
}

if ($ipc -notmatch '!BumpMode_LoadRecord\(&record,\s*&config\)[\s\S]*?config\.pwm_gain\s*=\s*BUMP_ACTIVE_PWM_GAIN' -or
    $screen -match 'ui_bump_gain\s*=\s*BUMP_ACTIVE_PWM_GAIN;[\s\S]*?IPC_Bump_Set_Config') {
    throw 'Saved manual BUMP settings must survive leaving and re-entering the page.'
}

if ($param -notmatch '#define\s+MAX_DUTY\s+\(70\)' -or
    $core0Project -notmatch 'bump_mode_logic\.c' -or
    $core1Project -notmatch 'bump_mode_logic\.c') {
    throw 'Motor final limit or IAR project membership is incorrect.'
}

Write-Output 'course3 fixed-yaw BUMP profile test passed'
