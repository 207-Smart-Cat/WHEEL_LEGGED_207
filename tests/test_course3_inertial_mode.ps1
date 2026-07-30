$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$runtime = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/runtime_status.h') -Raw
$logic = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/course3_bridge_logic.c') -Raw
$tracking = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/navigation_tracking.c') -Raw
$action = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/navigation_action.c') -Raw
$remote = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/remote.c') -Raw
$screen = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/screen_display.c') -Raw

if ($runtime -notmatch 'VEHICLE_MODE_COURSE_3_INERTIAL\s*=\s*4') {
    throw 'The inertial Course3 vehicle mode is missing or has the wrong value.'
}

if ($logic -notmatch 'Course3Segment_PointCountForMode' -or
    $logic -notmatch 'if\s*\(!Course3Mode_UsesVision\(vehicle_mode\)\)\s*\{\s*return 2U;') {
    throw 'Mode-aware two-point segment interpretation is missing.'
}

if ($tracking -notmatch 'Course3Segment_ExpectedActionForMode' -or
    $tracking -notmatch 'Course3Mode_IsCourse3\(Runtime_Get_Vehicle_Mode\(\)\)') {
    throw 'Recording or tracking was not extended to both Course3 modes.'
}

if ($action -notmatch 'course3_inertial_segment_begin' -or
    $action -notmatch 'course3_segment_get_map_target_yaw\(start_idx, end_idx\)' -or
    $action -notmatch 'Runtime_Get_Vehicle_Mode\(\)\s*==\s*VEHICLE_MODE_COURSE_3_INERTIAL') {
    throw 'The direct inertial obstacle action entry is missing.'
}

if ($remote -notmatch 'Course3Mode_IsCourse3\(mode\)' -or
    $screen -notmatch 'UI_TEXT_T_MODE_3_INERTIAL' -or
    $screen -notmatch 'Course3Segment_ExpectedActionForMode') {
    throw 'Remote recording or screen integration for inertial Course3 is incomplete.'
}

if ($action -match 'VEHICLE_MODE_COURSE_3_INERTIAL[\s\S]{0,400}FSM_COURSE3_TRACK_ALIGN') {
    throw 'The inertial Course3 branch must not enter visual track alignment.'
}

Write-Output 'course3 inertial mode structural test passed'
