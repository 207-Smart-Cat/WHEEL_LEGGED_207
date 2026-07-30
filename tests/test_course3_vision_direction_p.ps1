$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$tuningPath = Join-Path $repoRoot 'project/code/course3_tuning.h'
$controlPath = Join-Path $repoRoot 'project/code/control.c'

$tuning = Get-Content -LiteralPath $tuningPath -Raw
$control = Get-Content -LiteralPath $controlPath -Raw

if ($tuning -notmatch '#define\s+COURSE3_VISION_DIRECTION_P\s+\(50\.0f\)') {
    throw 'COURSE3_VISION_DIRECTION_P is not defined in course3_tuning.h with the expected default.'
}

if ($control -notmatch 'vision_control_pd_output\(yaw_error,\s*yaw_rate,\s*COURSE3_VISION_DIRECTION_P,\s*Direction_d,\s*2200\.0f\)') {
    throw 'Vision PD control does not use COURSE3_VISION_DIRECTION_P.'
}

Write-Output 'course3 vision Direction_P tuning test passed'
