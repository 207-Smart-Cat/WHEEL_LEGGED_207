$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$remote = Get-Content -LiteralPath (Join-Path $projectRoot "code/remote.c") -Raw
$action = Get-Content -LiteralPath (Join-Path $projectRoot "code/navigation_action.c") -Raw

if ($remote -notmatch 'VEHICLE_MODE_COURSE_2') {
    throw "Course 2 mapping branch was not found."
}

if ($remote -notmatch 'return WP_TYPE_MINE_SWEEP;') {
    throw "Course 2 mid switch must retain mine-sweep recording."
}

if ($remote -notmatch 'VEHICLE_MODE_COURSE_3') {
    throw "Course 3 mapping branch was not found."
}

if ($remote -notmatch 'ch4 < REMOTE_CH4_MID_THRESHOLD') {
    throw "Course 3 low/mid threshold was not found."
}

if ($remote -notmatch 'ch4 < REMOTE_CH4_HIGH_THRESHOLD') {
    throw "Course 3 mid/high threshold was not found."
}

if ($remote -notmatch 'return WP_TYPE_BRIDGE;') {
    throw "Course 3 mid switch must record bridge points."
}

if ($action -notmatch 'mode != VEHICLE_MODE_COURSE_3') {
    throw "Course 3 jumps must not enter the action sequence."
}

if ($action -notmatch 'point_map\[curr_idx\]\.type == WP_TYPE_NORMAL') {
    throw "Course 3 normal-point action handoff was not found."
}

Write-Host "course 3 waypoint framework checks passed"
