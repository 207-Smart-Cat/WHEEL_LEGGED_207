$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$tracking = [IO.File]::ReadAllText(
    (Join-Path $root 'project/code/navigation_tracking.c'),
    [Text.Encoding]::GetEncoding(936))
$header = [IO.File]::ReadAllText(
    (Join-Path $root 'project/code/navigation_tracking.h'),
    [Text.Encoding]::GetEncoding(936))
$action = Get-Content -LiteralPath (Join-Path $root 'project/code/navigation_action.c') -Raw

function Require([bool]$Condition, [string]$Message)
{
    if (-not $Condition) { throw $Message }
}

Require ($tracking -match 'uint8_t\s+meter_active;') `
    'The visual approach does not own a dedicated travel-meter state.'
Require ($tracking -match 'visual_approach[\s\S]*?Navi_Course3_Calibration_Meter_Begin\(NAVI_COURSE3_APPROACH_DISTANCE\)') `
    'The visual approach does not start a 0.50 m travel meter.'
Require ($tracking -match 'Navi_Course3_Vision_Approach_Is_Complete[\s\S]*?Navi_Course3_Calibration_Meter_Is_Complete') `
    'Visual calibration cannot be triggered from approach-meter completion.'
Require ($header -match 'Navi_Course3_Vision_Approach_Is_Complete\(uint16_t target_idx\)') `
    'The visual-approach completion interface is not exported.'

Require ($tracking -match 'entry_idx\s*=\s*\(uint16_t\)\(target_idx\s*\+\s*1U\)') `
    'The visual approach does not select the second waypoint.'
Require ($tracking -match 'navi_get_two_points_azimuth\(robot_pose\.x,\s*robot_pose\.y,\s*point_map\[entry_idx\]\.x') `
    'The visual approach heading is not calculated toward the second waypoint.'
Require ($tracking -match 'course3_approach_active[\s\S]*?navi_course3_apply_visual_approach_heading\(curr_idx') `
    'The second-waypoint heading is not connected to normal tracking control.'
Require ($tracking -match 'course3_approach_active\s*\|\|\s*is_action_busy') `
    'The first waypoint can still be advanced by the ordinary 10 cm reach path.'

Require ($action -match 'Navi_Course3_Vision_Approach_Is_Complete\(target_idx\)') `
    'Bridge/ramp visual calibration still waits for the first waypoint reach event.'
Require ($action -match 'calibration_distance\s*=\s*navi_get_two_points_distance\([\s\S]*?point_map\[target_idx\]\.x[\s\S]*?point_map\[entry_idx\]\.x') `
    'The original first-to-second waypoint visual-calibration distance was not preserved.'
Require ($action -match 'upcoming_type\s*==\s*WP_TYPE_JUMP\s*&&\s*navi_isreach_target_point\(target_idx\)') `
    'The existing Course3 jump-point reach trigger was changed unexpectedly.'

Write-Output 'course3 visual 0.50 m approach and second-point heading test passed'
