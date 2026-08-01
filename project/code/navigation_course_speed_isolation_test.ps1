$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$encoding = [Text.Encoding]::GetEncoding(936)

function Read-ProjectText([string]$name) {
    return [IO.File]::ReadAllText((Join-Path $root $name), $encoding)
}

function Require-Text([string]$text, [string]$needle, [string]$message) {
    if (-not $text.Contains($needle)) {
        throw $message
    }
}

function Reject-Text([string]$text, [string]$needle, [string]$message) {
    if ($text.Contains($needle)) {
        throw $message
    }
}

$profileHeader = Read-ProjectText "navigation_course_speed.h"
$profileSource = Read-ProjectText "navigation_course_speed.c"
$trackingHeader = Read-ProjectText "navigation_tracking.h"
$trackingSource = Read-ProjectText "navigation_tracking.c"
$smoothHeader = Read-ProjectText "navigation_smooth_logic.h"
$bridgeSource = Read-ProjectText "course3_bridge_logic.c"

Require-Text $profileHeader "NaviCourseSpeedProfile_t" "Missing per-course speed profile type."
Require-Text $profileSource "VEHICLE_MODE_COURSE_3_INERTIAL" "Course 3 inertial mode is not mapped."
Require-Text $profileSource "300.0f" "Course 3 p2 speed values are missing."
Require-Text $profileSource "12.0f" "Course 3 p2 speed step is missing."
Require-Text $profileSource "out->trigger_distance_m = 1.0f" "Course 2 mine distance is missing."
Require-Text $profileSource "out->approach_speed = 120.0f" "Course 2 mine speed is missing."
Require-Text $profileSource "out->trigger_distance_m = 0.50f" "Course 3 p2 approach distance is missing."
Require-Text $profileSource "out->approach_speed = 100.0f" "Course 3 p2 approach speed is missing."
Require-Text $trackingSource "navi_get_course_speed_profile" "Tracking does not select a course profile."
Require-Text $trackingSource "profile->high_speed_max_velocity" "High-speed limit is still shared."
Require-Text $trackingSource "smooth_zone_speed_limit" "Smooth limit is still shared."
Require-Text $trackingSource "profile->min_running_speed" "Minimum running speed is still shared."
Require-Text $trackingSource "profile->speed_max_step" "Speed step is still shared."
Require-Text $trackingSource "Navi_CourseSpeed_Get_Approach" "Tracking does not select an approach policy."
Require-Text $bridgeSource "trigger_distance_m" "Course 3 segment logic still owns the approach distance."

Reject-Text $trackingHeader "DEFAULT_TRACKING_VELOCITY" "Old fixed tracking macro remains."
Reject-Text $trackingHeader "NAVI_NORMAL_MAX_VELOCITY" "Old normal speed macro remains."
Reject-Text $trackingHeader "NAVI_HIGH_SPEED_MAX_VELOCITY" "Old high-speed macro remains."
Reject-Text $trackingHeader "NAVI_COURSE12_MIN_RUNNING_SPEED" "Old minimum speed macro remains."
Reject-Text $trackingHeader "NAVI_COURSE3_APPROACH_DISTANCE" "Old shared approach distance remains."
Reject-Text $trackingHeader "NAVI_COURSE3_APPROACH_SPEED" "Old shared approach speed remains."
Reject-Text $trackingHeader "NAVI_COURSE2_MINE_APPROACH_SPEED" "Old mine approach macro remains."
Reject-Text $smoothHeader "NAVI_SMOOTH_ZONE_SPEED_LIMIT" "Old smooth speed macro remains."

Write-Output "navigation course speed isolation checks passed"
