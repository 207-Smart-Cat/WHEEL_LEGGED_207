$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$mainPath = Join-Path $projectRoot "user/main_cm7_0.c"
$navHeaderPath = Join-Path $projectRoot "code/navigation_data_handling.h"
$navSourcePath = Join-Path $projectRoot "code/navigation_data_handling.c"

$main = Get-Content -LiteralPath $mainPath -Raw
$navHeader = Get-Content -LiteralPath $navHeaderPath -Raw
$navSource = Get-Content -LiteralPath $navSourcePath -Raw

if ($main -match 'wheel_speed,L:%d,R:%d') {
    throw "Wheel-speed debug print is still active."
}

if ($main -match 'IMU_ACC_RAW_VOFA_TEST_MODE' -or
    $main -match 'imu_acc_raw_vofa_test_loop' -or
    $main -match 'avg_x|avg_y|avg_z') {
    throw "Acceleration debug print path is still present."
}

if ($main -notmatch 'printf\("%\.3f,%\.3f,%\.3f\\r\\n",\s*robot_pose\.radius,\s*\(float\)robot_pose\.x,\s*\(float\)robot_pose\.y\);') {
    throw "Serial debug output must be comma-separated radius,x,y."
}

if ($main -notmatch 'radius_print_div\s*>=\s*5') {
    throw "Radius/X/Y debug output is not paced at the odometry update period."
}

if ($main -notmatch 'pit_ms_init\(PIT_Navigation,\s*5\)') {
    throw "Navigation PIT period must be 5ms."
}

if ($navHeader -notmatch 'float\s+radius;') {
    throw "RobotState_t does not expose the odometry radius."
}

if ($navHeader -notmatch '#define\s+NAVI_ARC_MIN_YAWRATE_RADPS\s+0\.10f') {
    throw "Arc odometry yaw-rate threshold must be a named 0.10f rad/s constant."
}

if ($navHeader -notmatch '#define\s+ENCODER_DT\s+0\.005f') {
    throw "ENCODER_DT must match the 5ms navigation period."
}

if ($navHeader -notmatch '#define\s+YAW_HISTORY_LEN\s+600') {
    throw "Yaw history length must preserve the 3s averaging window at 5ms."
}

if ($navSource -notmatch 'robot_pose\.radius\s*=\s*999\.0f;' -or
    $navSource -notmatch 'float\s+radius\s*=\s*opt_v\s*/\s*opt_w;' -or
    $navSource -notmatch 'robot_pose\.radius\s*=\s*radius;' -or
    $navSource.IndexOf('robot_pose.radius = radius;') -gt $navSource.IndexOf('dx = radius *') -or
    $navSource -notmatch 'fabsf\(opt_w\)\s*<\s*NAVI_ARC_MIN_YAWRATE_RADPS' -or
    $navSource -match 'fabsf\(opt_w\)\s*<\s*1e-3f') {
    throw "Navigation EKF does not expose the same radius value used by arc odometry."
}

Write-Host "navigation radius,x,y debug output checks passed"
