$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$remote = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/remote.c') -Raw
$tracking = Get-Content -LiteralPath (Join-Path $repoRoot 'project/code/navigation_tracking.c') -Raw

if ($remote -notmatch 'Navi_Record_Get_Open_Segment_Type\s*\(\)' -or
    $remote -notmatch 'return\s+\(WayPoint_Type\)open_type\s*;') {
    throw 'Course3 CH6 recording does not lock subsequent points to the open segment type.'
}

if ($remote -notmatch 'Remote_CheckRecordTrigger\s*\(\(uint8\)\(!normal_recorded\)\)') {
    throw 'Course3 CH6 recording is still gated by CH5 remote drive enable.'
}

if ($tracking -match 'segment start/end distance must be at least 1 cm') {
    throw 'Near-distance segment recording is still rejected before the point is stored.'
}

if ($tracking -notmatch 'adjacent segment points are closer than 1 cm; execution will reject this map') {
    throw 'Near-distance segment points must retain an execution-safety warning.'
}

Write-Output 'course3 CH6 segment recording test passed'
