$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$source = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/navigation_data_handling.c') -Raw

$meterPattern = 'if\s*\(course3_calibration_meter_active\)\s*\{[^}]*Course3TravelMeter_Update'
$bridgePattern = 'if\s*\(course3_bridge_odometry_active\)\s*\{[^}]*Course3BridgeOdometry_Update'

if ($source -notmatch $meterPattern) {
    throw 'Course3 calibration meter update is not independently gated.'
}

if ($source -notmatch $bridgePattern) {
    throw 'Course3 bridge odometry update is not independently gated.'
}

if ($source -match 'if\s*\(course3_bridge_odometry_active\)\s*if\s*\(course3_calibration_meter_active\)') {
    throw 'Bridge odometry and calibration meter conditions are incorrectly nested.'
}

Write-Output 'navigation pose update gating test passed'
