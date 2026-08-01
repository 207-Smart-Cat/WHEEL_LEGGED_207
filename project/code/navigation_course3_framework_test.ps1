$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$encoding = [Text.Encoding]::GetEncoding(936)
$remote = [IO.File]::ReadAllText((Join-Path $projectRoot "code/remote.c"), $encoding)
$action = [IO.File]::ReadAllText((Join-Path $projectRoot "code/navigation_action.c"), $encoding)
$bridge = [IO.File]::ReadAllText((Join-Path $projectRoot "code/course3_bridge_logic.c"), $encoding)
$tracking = [IO.File]::ReadAllText((Join-Path $projectRoot "code/navigation_tracking.c"), $encoding)
$courseSpeed = [IO.File]::ReadAllText((Join-Path $projectRoot "code/navigation_course_speed.c"), $encoding)

if ($remote -notmatch 'VEHICLE_MODE_COURSE_2') {
    throw "Course 2 mapping branch was not found."
}

if ($remote -notmatch 'return WP_TYPE_MINE_SWEEP;') {
    throw "Course 2 mid switch must retain mine-sweep recording."
}

if ($remote -notmatch 'Course3Mode_IsCourse3') {
    throw "Course 3 mapping branch was not found."
}

if ($remote -notmatch 'Course3Remote_SelectSpecialType') {
    throw "Course 3 special-point selector was not found."
}

if ($bridge -notmatch 'COURSE3_CH4_LOW_MID_BOUNDARY' -or
    $bridge -notmatch 'COURSE3_CH4_MID_HIGH_BOUNDARY') {
    throw "Course 3 selector thresholds were not found."
}

if ($action -notmatch 'Course3Segment_ShouldQueueAction') {
    throw "Course 3 action queue integration was not found."
}

if ($tracking -notmatch 'Navi_CourseSpeed_Get_Approach') {
    throw "Course 3 approach policy integration was not found."
}

if ($courseSpeed -notmatch '0\.50f' -or $courseSpeed -notmatch '100\.0f') {
    throw "Course 3 p2 approach values were not found."
}

Write-Host "course 3 waypoint framework checks passed"
