$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$encoding = [Text.Encoding]::GetEncoding(936)
$header = [IO.File]::ReadAllText((Join-Path $root 'project/code/ipc_shared_data.h'), $encoding)
$ipc = [IO.File]::ReadAllText((Join-Path $root 'project/code/ipc_shared_data.c'), $encoding)
$screen = [IO.File]::ReadAllText((Join-Path $root 'project/code/screen_display.c'), $encoding)

function Require([bool]$condition, [string]$message)
{
    if (-not $condition) { throw $message }
}

Require ($header -match '#define\s+NAV_GROUP_COUNT\s+\(10U\)') `
    'Flash-backed group count changed from 10.'
Require ($header -match '#define\s+NAV_COURSE3_PRESET_GROUP\s+\(NAV_GROUP_COUNT\)') `
    'Preset group is not isolated at index 10.'
Require ($header -match '#define\s+NAV_EXEC_GROUP_COUNT\s+\(NAV_GROUP_COUNT \+ 1U\)') `
    'Execution group count does not include the preset.'
Require ($header -match '#define\s+NAV_COURSE3_PRESET_POINT_COUNT\s+\(20U\)') `
    'Preset point count is not 20.'
Require ($ipc -match 'NAV_GROUP_COUNT \* NAV_STORE_GROUP_PAGES\) == 80U') `
    'Flash page layout is no longer fixed to pages 0..79.'

$blockMatch = [regex]::Match(
    $ipc,
    'g_course3_preset_points\[NAV_COURSE3_PRESET_POINT_COUNT\]\s*=\s*\{(?<body>[\s\S]*?)\n\};')
Require $blockMatch.Success 'Preset point table was not found.'

$pointPattern = '\{\s*(?<x>-?\d+\.\d+)f,\s*(?<y>-?\d+\.\d+)f,\s*(?<yaw>-?\d+\.\d+)f,\s*(?<action>[A-Z0-9_]+),\s*(?<type>WP_TYPE_[A-Z_]+),\s*1U\s*\}'
$matches = [regex]::Matches($blockMatch.Groups['body'].Value, $pointPattern)
Require ($matches.Count -eq 20) "Expected 20 preset points, found $($matches.Count)."

$expected = @(
    @( 0.00,  0.00,    0.0, '0U',                                   'WP_TYPE_HOME'),
    @( 0.37,  0.00,    2.1, '0U',                                   'WP_TYPE_NORMAL'),
    @( 1.00, -0.92,  -63.7, '0U',                                   'WP_TYPE_NORMAL'),
    @( 2.61, -0.92,    5.4, '0U',                                   'WP_TYPE_NORMAL'),
    @( 3.36, -0.91,    0.0, 'NAVI_VISION_SEGMENT_ACTION_CALIBRATE', 'WP_TYPE_BRIDGE'),
    @( 4.06, -0.90,   -0.2, 'NAVI_VISION_SEGMENT_ACTION_ENTRY',     'WP_TYPE_BRIDGE'),
    @( 5.18, -0.88,   -3.0, 'NAVI_VISION_SEGMENT_ACTION_END',       'WP_TYPE_BRIDGE'),
    @( 5.75, -0.88,   -0.2, '0U',                                   'WP_TYPE_NORMAL'),
    @( 6.60, -1.73,  -37.9, '0U',                                   'WP_TYPE_NORMAL'),
    @( 8.33, -1.76,   12.9, '0U',                                   'WP_TYPE_NORMAL'),
    @(10.92, -0.74,   20.7, '0U',                                   'WP_TYPE_NORMAL'),
    @(10.93,  0.80,   98.8, '0U',                                   'WP_TYPE_NORMAL'),
    @( 6.01,  0.96,  175.8, '0U',                                   'WP_TYPE_NORMAL'),
    @( 4.80,  0.90,  178.1, '0U',                                   'WP_TYPE_NORMAL'),
    @( 3.98,  0.95,  174.2, '0U',                                   'WP_TYPE_NORMAL'),
    @( 3.10, -0.15, -127.4, '0U',                                   'WP_TYPE_NORMAL'),
    @( 2.48, -0.15,  179.4, 'NAVI_VISION_SEGMENT_ACTION_CALIBRATE', 'WP_TYPE_STAIR_RAMP'),
    @( 2.12, -0.15, -176.5, 'NAVI_VISION_SEGMENT_ACTION_ENTRY',     'WP_TYPE_STAIR_RAMP'),
    @( 0.90, -0.15,  -97.0, 'NAVI_VISION_SEGMENT_ACTION_END',       'WP_TYPE_STAIR_RAMP'),
    @(-0.43, -0.03,  169.3, '0U',                                   'WP_TYPE_NORMAL')
)

for ($i = 0; $i -lt $expected.Count; $i++)
{
    $actual = $matches[$i].Groups
    $want = $expected[$i]
    Require ([Math]::Abs([double]$actual['x'].Value - $want[0]) -lt 0.0001) "Point $($i + 1) X mismatch."
    Require ([Math]::Abs([double]$actual['y'].Value - $want[1]) -lt 0.0001) "Point $($i + 1) Y mismatch."
    Require ([Math]::Abs([double]$actual['yaw'].Value - $want[2]) -lt 0.0001) "Point $($i + 1) yaw mismatch."
    Require ($actual['action'].Value -eq $want[3]) "Point $($i + 1) action mismatch."
    Require ($actual['type'].Value -eq $want[4]) "Point $($i + 1) type mismatch."
}

Require ($screen -match 'ui_group_action == UI_GROUP_ACTION_EXECUTE[\s\S]*?Runtime_Get_Vehicle_Mode\(\) == VEHICLE_MODE_COURSE_3[\s\S]*?NAV_EXEC_GROUP_COUNT') `
    'Preset group is not restricted to visual Course 3 execution selection.'
Require ($screen -match 'group == NAV_COURSE3_PRESET_GROUP[\s\S]*?"PRESET"') `
    'Preset group is not identified in the execution list.'
Require ($ipc -match 'group == NAV_COURSE3_PRESET_GROUP[\s\S]*?intent != NAV_GROUP_INTENT_EXECUTE[\s\S]*?VEHICLE_MODE_COURSE_3') `
    'Core1 preset loading is not restricted to visual Course 3 execution.'
Require ($ipc -match 'g_nav_load_group == NAV_COURSE3_PRESET_GROUP[\s\S]*?g_course3_preset_points') `
    'Preset chunks do not use the code-backed point table.'

Write-Output 'course3 code-backed preset group test passed'
