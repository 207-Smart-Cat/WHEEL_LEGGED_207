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

$zoneHeader = Read-ProjectText "navigation_speed_zone.h"
$zoneSource = Read-ProjectText "navigation_speed_zone.c"
$trackingHeader = Read-ProjectText "navigation_tracking.h"
$trackingSource = Read-ProjectText "navigation_tracking.c"
$ipcHeader = Read-ProjectText "ipc_shared_data.h"
$ipcSource = Read-ProjectText "ipc_shared_data.c"

Require-Text $zoneHeader "NaviMapSpeedZoneProfile_t" "Missing per-map zone profile."
Require-Text $zoneSource "navi_speed_zone_profiles[NAV_EXEC_GROUP_COUNT]" "Map groups are not indexed directly."
Require-Text $zoneSource "UI map 11, Course 3 preset group 10" "Map 11 preset entry is missing."
Require-Text $zoneSource "current_target_idx > start_point_idx" "Passage start must be exclusive."
Require-Text $zoneSource "current_target_idx <= end_point_idx" "Passage end must be inclusive."
Require-Text $zoneSource "for (idx = 0U; idx < navi_speed_zone_selected_profile->zone_count; idx++)" "A map cannot hold multiple zones."
Require-Text $zoneSource "profile->zone_count == 0U" "Empty map profiles are not supported."
Require-Text $zoneSource "navi_speed_zone_profiles_overlap" "Zone overlap validation is missing."
Require-Text $zoneSource "navi_speed_zone_type_is_allowed" "Action-point validation is missing."
Require-Text $ipcHeader "IPC_Nav_Get_Active_Group" "Active map group getter is missing."
Require-Text $ipcSource "return g_nav_record_active_group" "The selected Flash group is not exposed to Core0."
Require-Text $trackingSource "Navi_SpeedZone_Select_Profile(IPC_Nav_Get_Active_Group()" "Tracking does not bind the loaded map group."
Require-Text $trackingSource "zone_speed = Navi_SpeedZone_Get_Speed(target_idx)" "Tracking does not query map speed zones."
Require-Text $trackingSource "profile->high_speed_max_velocity" "Zone speed is not capped by the current course."
Require-Text $trackingSource "Navi_SpeedZone_Is_Final_Target_Hold" "Final-zone hold integration is missing."

Reject-Text $zoneSource "NAVI_COURSE_SLOT_COUNT][" "Speed zones must not be indexed by course and map."
Reject-Text $trackingHeader "NAVI_RETURN_HIGH_SPEED_AFTER_LAST_N_POINTS" "Legacy last-N sprint macro remains."
Reject-Text $trackingSource "navi_is_course12_return_high_speed_segment" "Legacy return sprint function remains."
Reject-Text $trackingSource "navi_is_high_speed_segment" "Legacy type-based acceleration remains."

function Passage-Speed([int]$start, [int]$end, [double]$speed, [int]$target) {
    if ($start -ge $end -or $speed -lt 0.0) {
        return -1.0
    }
    if ($target -gt $start -and $target -le $end) {
        return $speed
    }
    return -1.0
}

if ((Passage-Speed 2 4 950 2) -ge 0.0) { throw "Zone activated while driving to start point." }
if ((Passage-Speed 2 4 950 3) -ne 950) { throw "Zone did not activate after start point." }
if ((Passage-Speed 2 4 950 4) -ne 950) { throw "Zone did not include end point." }
if ((Passage-Speed 2 4 950 5) -ge 0.0) { throw "Zone remained active after end point." }

Write-Output "navigation map speed-zone checks passed"
