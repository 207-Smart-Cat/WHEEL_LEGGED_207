$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$runtimeHeader = Join-Path $PSScriptRoot 'triple_jump_runtime.h'
$runtimeSource = Join-Path $PSScriptRoot 'triple_jump_runtime.c'
$ipcHeader = Join-Path $PSScriptRoot 'ipc_shared_data.h'
$ipcSource = Join-Path $PSScriptRoot 'ipc_shared_data.c'
$isrSource = Join-Path $root 'project/user/cm7_0_isr.c'
$navSource = Join-Path $PSScriptRoot 'navigation_action.c'
$navHeader = Join-Path $PSScriptRoot 'navigation_action.h'
$cm70Project = Join-Path $root 'project/iar/project_config/cyt4bb7_cm_7_0.ewp'
$cm71Project = Join-Path $root 'project/iar/project_config/cyt4bb7_cm_7_1.ewp'

foreach ($path in @($runtimeHeader, $runtimeSource)) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Missing triple-jump runtime file: $path"
    }
}

$runtime = Get-Content -LiteralPath $runtimeSource -Raw
$ipcH = Get-Content -LiteralPath $ipcHeader -Raw
$ipcC = Get-Content -LiteralPath $ipcSource -Raw
$isr = Get-Content -LiteralPath $isrSource -Raw
$nav = (Get-Content -LiteralPath $navSource -Raw) + (Get-Content -LiteralPath $navHeader -Raw)
$projects = (Get-Content -LiteralPath $cm70Project -Raw) + (Get-Content -LiteralPath $cm71Project -Raw)

foreach ($token in @('triple_jump_start_seq', 'triple_jump_stop_seq',
                      'triple_jump_x1_m', 'triple_jump_x2_m',
                      'triple_jump_x3_m', 'triple_jump_speed')) {
    if ($ipcH -notmatch $token) { throw "IPC field missing: $token" }
}
foreach ($token in @('IPC_Request_Triple_Jump_Start',
                      'IPC_Request_Triple_Jump_Stop',
                      'IPC_Consume_Triple_Jump_Start_Core0',
                      'IPC_Consume_Triple_Jump_Stop_Core0')) {
    if ($ipcC -notmatch $token) { throw "IPC API missing: $token" }
}
foreach ($token in @('TripleJump_Start', 'TripleJump_Update5ms',
                      'JumpAction_Start', 'JumpAction_Task5ms',
                      'RUNTIME_MODULE_NAVIGATION',
                      'motor_value.receive_left_speed_data',
                      'motor_value.receive_right_speed_data',
                      'target_velocity', 'target_angle')) {
    if ($runtime -notmatch [regex]::Escape($token)) {
        throw "Runtime integration missing: $token"
    }
}
if ($isr -notmatch 'TripleJumpRuntime_Task5ms') {
    throw 'PIT_CH14 does not schedule TripleJumpRuntime_Task5ms'
}
if ($nav -match 'TripleJumpRuntime|triple_jump_start_seq') {
    throw 'Standalone triple jump leaked into navigation_action files'
}
foreach ($file in @('triple_jump_runtime.c', 'triple_jump.c',
                     'jump_action_profile.c', 'landing_detector.c')) {
    if ($projects -notmatch [regex]::Escape($file)) {
        throw "IAR project entry missing: $file"
    }
}

Write-Host 'Triple-jump integration checks passed.'
