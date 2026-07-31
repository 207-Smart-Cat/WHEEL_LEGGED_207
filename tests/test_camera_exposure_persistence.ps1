$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$assist = Get-Content -LiteralPath (Join-Path $root 'project/code/camera_assist.c') -Raw
$assistHeader = Get-Content -LiteralPath (Join-Path $root 'project/code/camera_assist.h') -Raw
$display = Get-Content -LiteralPath (Join-Path $root 'project/code/camera_test_display.c') -Raw
$tuning = Get-Content -LiteralPath (Join-Path $root 'project/code/course3_tuning.h') -Raw
$main = [IO.File]::ReadAllText(
    (Join-Path $root 'project/user/main_cm7_1.c'),
    [Text.Encoding]::GetEncoding(936))
$screen = [IO.File]::ReadAllText(
    (Join-Path $root 'project/code/screen_display.c'),
    [Text.Encoding]::GetEncoding(936))

foreach ($required in @(
    'CAMERA_CONFIG_PAGE_A            (93U)',
    'CAMERA_CONFIG_PAGE_B            (94U)',
    'CAMERA_CONFIG_MAGIC',
    'camera_config_checksum',
    'CameraAssist_RequestExposureSave',
    'CameraAssist_FlushExposureConfig',
    'CameraAssist_ConfigTask'
)) {
    if (-not $assist.Contains($required)) {
        throw "Missing exposure persistence element: $required"
    }
}

if ($assistHeader -notmatch 'CAMERA_ASSIST_EXPOSURE_MIN\s+\(0U\)' -or
    $assistHeader -notmatch 'CAMERA_ASSIST_EXPOSURE_FINE_THRESHOLD\s+\(40U\)' -or
    $assistHeader -notmatch 'CAMERA_ASSIST_EXPOSURE_MEDIUM_THRESHOLD\s+\(100U\)' -or
    $assistHeader -notmatch 'CAMERA_ASSIST_EXPOSURE_MAX\s+\(1000U\)') {
    throw 'Exposure bounds are not centralized in camera_assist.h.'
}

if ($display -notmatch 'CameraAssist_RequestExposureSave\(\)' -or
    $display -notmatch 'EXP:%4u %-5s' -or
    $display -notmatch 'CAMERA_TEST_EXPOSURE_FINE_STEP\s+\(1U\)' -or
    $display -notmatch 'CAMERA_TEST_EXPOSURE_MEDIUM_STEP\s+\(5U\)' -or
    $display -notmatch 'exposure < CAMERA_ASSIST_EXPOSURE_FINE_THRESHOLD') {
    throw 'Vision display does not show and save the adjusted exposure.'
}

if ($main -notmatch 'CameraAssist_ConfigTask\(\)') {
    throw 'Core1 main loop does not service delayed exposure saves.'
}

if ($screen -notmatch 'CameraAssist_FlushExposureConfig\(\)' -or
    $screen -notmatch 'P_TARGET_VELOCITY,\s*0\.0f') {
    throw 'Vision entry/exit handling is incomplete.'
}

if ($tuning -notmatch '#define\s+VISION_MENU_CAL_SPEED\s+\(120\.0f\)' -or
    $tuning -notmatch '#define\s+COURSE3_VISION_CAL_SPEED\s+\(120\.0f\)') {
    throw 'Manual and Course3 Vision guidance must both use target speed 120.'
}

function Resolve-Exposure([int]$Exposure, [bool]$Increase)
{
    if ($Increase) {
        if ($Exposure -lt 40) { return $Exposure + 1 }
        if ($Exposure -lt 100) {
            if ($Exposure -le 95) { return $Exposure + 5 }
            return 100
        }
        if ($Exposure -le 975) { return $Exposure + 25 }
        return 1000
    }

    if ($Exposure -gt 100) {
        if ($Exposure -ge 125) { return $Exposure - 25 }
        return 100
    }
    if ($Exposure -gt 40) {
        if ($Exposure -ge 45) { return $Exposure - 5 }
        return 40
    }
    if ($Exposure -gt 0) { return $Exposure - 1 }
    return 0
}

$exposureCases = @(
    @(0, $false, 0),
    @(1, $false, 0),
    @(39, $true, 40),
    @(40, $false, 39),
    @(40, $true, 45),
    @(50, $false, 45),
    @(95, $true, 100),
    @(100, $false, 95),
    @(100, $true, 125),
    @(110, $false, 100),
    @(125, $false, 100),
    @(975, $true, 1000),
    @(1000, $true, 1000)
)
foreach ($case in $exposureCases) {
    $actual = Resolve-Exposure $case[0] $case[1]
    if ($actual -ne $case[2]) {
        throw "Exposure step case failed: $($case[0]), increase=$($case[1]), expected=$($case[2]), actual=$actual"
    }
}

Write-Output 'camera exposure persistence and Vision stop test passed'
