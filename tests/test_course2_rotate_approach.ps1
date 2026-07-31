$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$action = Get-Content -LiteralPath (Join-Path $root 'project/code/navigation_action.c') -Raw
$tracking = [IO.File]::ReadAllText(
    (Join-Path $root 'project/code/navigation_tracking.c'),
    [Text.Encoding]::GetEncoding(936))
$header = [IO.File]::ReadAllText(
    (Join-Path $root 'project/code/navigation_tracking.h'),
    [Text.Encoding]::GetEncoding(936))

function Require([bool]$Condition, [string]$Message)
{
    if (-not $Condition) { throw $Message }
}

Require ($header -match 'NAVI_COURSE3_APPROACH_DISTANCE\s+0\.50f') `
    'The shared special-point approach distance must remain 0.50 m.'
Require ($header -match 'NAVI_COURSE3_APPROACH_SPEED\s+100\.0f') `
    'The shared special-point approach speed must remain 100.'
Require ($tracking -match 'mode\s*==\s*VEHICLE_MODE_COURSE_2[\s\S]*?target->type\s*==\s*WP_TYPE_MINE_SWEEP') `
    'Course 2 rotate points do not enter the shared approach state.'
Require ($tracking -match 'course2_rotate[\s\S]*?distance\s*>\s*NAVI_COURSE3_APPROACH_DISTANCE') `
    'Course 2 rotate-point approach does not use the 0.50 m threshold.'
Require ($tracking -notmatch 'case\s+WP_TYPE_MINE_SWEEP:\s*return\s+DISTANCE_THRESHOLD\s*\*\s*0\.5f') `
    'Course 2 rotate points still use the old 5 cm reach radius.'

Require ($action -match '#define\s+MINE_ROTATE_BASE_DEG\s+720\.0f') `
    'The rotate action does not enforce two full turns.'
Require ($action -match '360\.0f\s*:\s*MINE_ROTATE_BASE_DEG') `
    'The mine rotate target is not fixed at 720 degrees.'
Require ($action -notmatch 'mine_rotate_exit_delta_deg|mine_get_exit_delta_deg|mine_wrap_positive_deg') `
    'Next-waypoint exit-angle compensation is still present.'
Require ($action -match 'rotated_progress\s*=\s*\(rotated_deg\s*>\s*0\.0f\)') `
    'Rotation in the wrong direction can still count toward completion.'

Require ($header -match 'NAVI_NORMAL_MAX_VELOCITY\s+300\.0f') `
    'Course 1/2 normal waypoint speed cap must remain 300.'
Require ($tracking -match 'mode\s*!=\s*VEHICLE_MODE_COURSE_1\s*&&\s*mode\s*!=\s*VEHICLE_MODE_COURSE_2') `
    'Course 1 and Course 2 do not share the high-speed segment resolver.'

Write-Output 'course2 rotate approach and fixed two-turn test passed'
