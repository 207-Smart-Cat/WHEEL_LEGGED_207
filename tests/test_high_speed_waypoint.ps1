$ErrorActionPreference = 'Stop'

function Assert-Contains([string]$Text, [string]$Needle, [string]$Message) {
    if (-not $Text.Contains($Needle)) {
        throw $Message
    }
}

$root = Split-Path -Parent $PSScriptRoot
$enc = [Text.Encoding]::GetEncoding(936)
$header = [IO.File]::ReadAllText((Join-Path $root 'project/code/navigation_tracking.h'), $enc)
$remote = [IO.File]::ReadAllText((Join-Path $root 'project/code/remote.c'), $enc)
$tracking = [IO.File]::ReadAllText((Join-Path $root 'project/code/navigation_tracking.c'), $enc)
$screen = [IO.File]::ReadAllText((Join-Path $root 'project/code/screen_display.c'), $enc)

Assert-Contains $header 'WP_TYPE_HIGH_SPEED = 9' 'High-speed waypoint enum is missing.'
Assert-Contains $header 'NAVI_HIGH_SPEED_MAX_VELOCITY          500.0f' '500 speed limit is missing.'
Assert-Contains $header 'NAVI_HIGH_SPEED_EXIT_DISTANCE_M       3.0f' '3 m exit distance is missing.'
Assert-Contains $remote 'mode == VEHICLE_MODE_COURSE_1 || mode == VEHICLE_MODE_COURSE_2' 'Course 1/2 mapping is missing.'
Assert-Contains $remote 'return WP_TYPE_HIGH_SPEED;' 'CH4 high position does not record a high-speed point.'
Assert-Contains $tracking 'point_map[target_idx - 1U].type != WP_TYPE_HIGH_SPEED' 'High-speed segment does not require two adjacent high-speed points.'
Assert-Contains $tracking 'point_map[target_idx + 1U].type != WP_TYPE_HIGH_SPEED' 'High-speed exit detection is missing.'
Assert-Contains $tracking 'distance / NAVI_HIGH_SPEED_EXIT_DISTANCE_M' 'Exit deceleration formula is missing.'
Assert-Contains $tracking 'point_map[target_idx - 1U].type != WP_TYPE_HIGH_SPEED ||' 'Course 1 entry boundary must not advance early.'
Assert-Contains $screen 'UI_TEXT_T_TYPE_HIGH_SPEED' 'High-speed waypoint screen text is missing.'

$normal = 300.0
$high = 500.0
$exitDistance = 3.0
function Resolve-Limit([double]$Distance) {
    $ratio = [Math]::Max(0.0, [Math]::Min(1.0, $Distance / $exitDistance))
    return $normal + ($high - $normal) * $ratio
}

if ([Math]::Abs((Resolve-Limit 3.0) - 500.0) -gt 0.001) { throw '3 m speed limit must be 500.' }
if ([Math]::Abs((Resolve-Limit 1.5) - 400.0) -gt 0.001) { throw '1.5 m speed limit must be 400.' }
if ([Math]::Abs((Resolve-Limit 0.0) - 300.0) -gt 0.001) { throw 'Exit-point speed limit must be normal speed.' }

function Resolve-Turn-Limit([double]$ErrorDeg, [double]$StraightLimit) {
    $absTurn = [Math]::Abs($ErrorDeg)
    $cornerEntry = [Math]::Min($StraightLimit, 300.0)
    if ($StraightLimit -lt 100.0) { return $StraightLimit }
    if ($absTurn -le 5.0) { return $StraightLimit }
    if ($absTurn -lt 20.0 -and $StraightLimit -gt $cornerEntry) {
        return $StraightLimit - ($absTurn - 5.0) * (($StraightLimit - $cornerEntry) / 15.0)
    }
    if ($absTurn -le 20.0) { return $cornerEntry }
    if ($absTurn -ge 90.0) { return 100.0 }
    return $cornerEntry - ($absTurn - 20.0) * (($cornerEntry - 100.0) / 70.0)
}

if ([Math]::Abs((Resolve-Turn-Limit 5.0 500.0) - 500.0) -gt 0.001) { throw 'Aligned high-speed segment must allow 500.' }
if ([Math]::Abs((Resolve-Turn-Limit 12.5 500.0) - 400.0) -gt 0.001) { throw 'High-speed alignment ramp is incorrect.' }
if ([Math]::Abs((Resolve-Turn-Limit 20.0 500.0) - 300.0) -gt 0.001) { throw '20 degree turn limit must return to normal speed.' }
if ([Math]::Abs((Resolve-Turn-Limit 90.0 500.0) - 100.0) -gt 0.001) { throw 'Large-turn limit must remain 100.' }

Write-Output 'PASS: high-speed waypoint mapping and speed profile'
