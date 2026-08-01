$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$screen = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'screen_display.c') -Raw
$cm70 = Get-Content -LiteralPath (Join-Path $root 'project/iar/project_config/cyt4bb7_cm_7_0.ewp') -Raw
$cm71 = Get-Content -LiteralPath (Join-Path $root 'project/iar/project_config/cyt4bb7_cm_7_1.ewp') -Raw

foreach ($token in @('UI_SCREEN_JUMP', 'ui_draw_jump', 'ui_handle_jump',
                      'IPC_Request_Triple_Jump_Start',
                      'IPC_Request_Triple_Jump_Stop',
                      'TripleJumpConfig_Load', 'TripleJumpConfig_Save',
                      'core_a_status.triple_jump_landings',
                      'core_a_status.triple_jump_distance_m')) {
    if ($screen -notmatch [regex]::Escape($token)) {
        throw "Jump UI integration missing: $token"
    }
}
if ($screen -match 'IPC_Request_Nav_Jump\(\)') {
    throw 'Jump menu still invokes the legacy navigation jump request'
}
foreach ($project in @($cm70, $cm71)) {
    foreach ($file in @('triple_jump_config.c', 'triple_jump_config.h')) {
        if ($project -notmatch [regex]::Escape($file)) {
            throw "IAR project entry missing: $file"
        }
    }
}

Write-Host 'Triple-jump UI and Flash integration checks passed.'
