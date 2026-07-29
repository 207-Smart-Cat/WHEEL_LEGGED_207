# Current Parameters

This file records the current PID parameters, PWM parameters, odometry parameters, and major thresholds used by the car firmware.

Notes:
- Values with `_init` are default/reset values registered in `project/code/param.c`. Runtime variables may be changed by IPC/WiFi/flash parameter handling.
- Macro and local `const` values are compile-time values unless the code is changed and rebuilt.

## PID And Filter Defaults

Source: `project/code/param.c`

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `Speed_p_init` | `0.025f` | Speed loop P |
| `Speed_i_init` | `0.0f` | Speed loop I |
| `Speed_d_init` | `0.0f` | Speed loop D |
| `Angle_p_init` | `12.3999f` | Balance angle loop P |
| `Angle_i_init` | `0.15f` | Balance angle loop I |
| `Angle_d_init` | `0.0f` | Balance angle loop D |
| `Gyro_p_init` | `15.0f` | Gyro loop P |
| `Gyro_i_init` | `0.0f` | Gyro loop I |
| `Gyro_d_init` | `0.0f` | Gyro loop D |
| `Air_roll_p_init` | `3.0f` | Air roll control P |
| `Air_roll_i_init` | `0.03f` | Air roll control I |
| `Air_roll_d_init` | `0.0015f` | Air roll control D |
| `Direction_p_init` | `15.00f` | Yaw/direction control P |
| `Direction_i_init` | `0.012f` | Yaw/direction control I |
| `Direction_d_init` | `0.875f` | Yaw/direction control D |
| `Leg_Kp_init` | `0.0f` | Leg-height PID P |
| `Leg_Ki_init` | `0.0f` | Leg-height PID I |
| `Leg_Kd_init` | `0.0f` | Leg-height PID D |
| `Q_yaw_init` | `0.001f` | IMU yaw Kalman process noise |
| `Q_pr_init` | `0.003f` | IMU pitch/roll Kalman process noise |
| `Q_bias_init` | `0.001f` | IMU bias process noise |
| `R_yaw_init` | `0.05f` | IMU yaw measurement noise |
| `R_pr_init` | `0.05f` | IMU pitch/roll measurement noise |

## Navigation EKF Defaults

Source: `project/code/param.c`, `project/code/navigation_data_handling.h`

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `Nav_q_v_init` / `NAV_Q_V` | `2.0f` | Linear velocity process noise |
| `Nav_q_w_init` / `NAV_Q_W` | `0.0099f` | Angular velocity process noise |
| `Nav_q_bias_ax_init` / `NAV_Q_BIAS_AX` | `0.015f` | Acceleration bias process noise |
| `Nav_q_bias_w_init` / `NAV_Q_BIAS_W` | `0.001f` | Gyro bias process noise |
| `Nav_r_v_normal_init` / `NAV_R_V_NORMAL` | `0.01f` | Encoder velocity measurement noise, normal |
| `Nav_r_v_slip_init` / `NAV_R_V_SLIP` | `0.01f` | Encoder velocity measurement noise, slip |
| `Nav_r_w_normal_init` / `NAV_R_W_NORMAL` | `0.002f` | Encoder angular-rate measurement noise, normal |
| `Nav_r_w_slip_init` / `NAV_R_W_SLIP` | `0.002f` | Encoder angular-rate measurement noise, slip |
| `Nav_r_gyro_init` / `NAV_R_GYRO` | `0.01f` | IMU gyro measurement noise |
| `SLIP_THRESHOLD` | `2.0f` | Slip detection threshold, m/s^2 |
| `STATIC_V_THRESHOLD` | `0.01f` | Static velocity threshold |

## Target And Runtime Defaults

Source: `project/code/param.c`

| Parameter | Current runtime value | Default/reset value |
| --- | ---: | ---: |
| `target_velocity` | `0` | `Target_Velocity_init = 0.0f` |
| `target_angle` | `180.0f` | `Target_Angle_init = 0.0f` |
| `target_motor_Stand` | `0` | `Target_Motor_Stand_init = 4.0f` |
| `x_current` | `0` | `X_Current_init = 0.0f` |
| `y_current` | `0.04` | `Y_Current_init = 0.04f` |
| `servo_alpha` | `0.55f` | Fixed const |
| `mag_offset_x` | `-0.080f` | `Mag_offset_x_init = 10.0f` |
| `mag_offset_y` | `0.040f` | `Mag_offset_y_init = 0.01f` |
| `mag_scale_x` | `1.0f` | `Mag_scale_x_init = 10.0f` |
| `mag_scale_y` | `1.0499f` | `Mag_scale_y_init = 0.01f` |

## Motor PWM And Servo Parameters

Sources: `project/code/param.h`, `project/code/small_driver_uart_control.c`, `project/code/engine.h`, `project/code/engine.c`, `project/code/jump_control.c`

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `MAX_DUTY` | `70` | Motor PWM max percent; comment maps to max duty `7000` |
| `MOTOR_DUTY_MAX_ABS` | `MAX_DUTY * (PWM_DUTY_MAX / 100)` | Final absolute motor duty limit |
| `MOTOR_STARTUP_DUTY_STEP` | `1` | Motor startup ramp step |
| `MOTOR_ZERO_WAIT_MIN_MS` | `5500U` | Min wait before zero-calibration speed request |
| `MOTOR_ZERO_WAIT_TIMEOUT_MS` | `8000U` | Zero-calibration timeout |
| `MOTOR_ZERO_SETTLE_MS` | `500U` | Zero-calibration settle time |
| `FREQ` | `50` | Servo PWM frequency |
| Servo logical min/max | `250` / `1230` | Normal servo logical PWM clamp in `engine.c` |
| `SERVO_TEST_MODE` | `0` | Servo test disabled |
| `SERVO_TEST_CHANNEL` | `2` | Servo test channel |
| `SERVO_TEST_DUTY` | `750` | Servo test fixed duty |
| `SERVO_TEST_CENTER_DUTY` | `750` | Servo test center duty |
| `SERVO_TEST_MIN_DUTY` | `350` | Servo test min duty |
| `SERVO_TEST_MAX_DUTY` | `1150` | Servo test max duty |
| `SERVO_TEST_STEP_TICKS` | `50` | Servo test auto step period |
| `JUMP_PREPARE_PWM` | `420` | Jump prepare servo PWM |
| `JUMP_BURST_PWM` | `1300` | Jump takeoff servo PWM |
| `JUMP_RECOVER_PWM` | `420` | Jump recover servo PWM |
| `JUMP_SERVO_SUM` | `1500` | Symmetric jump servo sum |
| `JUMP_SERVO_MIN_PWM` | `270` | Jump servo min PWM |
| `JUMP_SERVO_MAX_PWM` | `1300` | Jump servo max PWM |

## Leg Geometry And Leg Control

Sources: `project/code/param.h`, `project/code/param.c`, `project/code/control.c`

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `MIN_X` / `MAX_X` | `-0.05` / `0.05` | Foot X command clamp |
| `MIN_Y` / `MAX_Y` | `0.025` / `0.14` | Foot Y command clamp |
| `MIN_LEG_LENGTH` / `MAX_LEG_LENGTH` | `0.04` / `0.1` | Leg length range |
| `L1`, `L2`, `L3`, `L4`, `L5` | `0.06`, `0.09`, `0.09`, `0.06`, `0.038` | Five-bar linkage lengths |
| `leg_x_gain` | `1.6f` | Runtime speed-tilt to leg-X gain |
| `leg_x_limit` | `0.018f` | Runtime leg-X offset limit |
| `leg_x_min_step` | `0.0008f` | Runtime minimum leg-X step |
| `leg_x_step_limit` | `0.0012f` | Runtime leg-X step limit |
| `Leg_X_Gain_init` | `5.0f` | Default leg-X gain |
| `Leg_X_Limit_init` | `0.020f` | Default leg-X limit |
| `Leg_X_Min_Step_init` | `0.0008f` | Default leg-X minimum step |
| `Leg_X_Step_Limit_init` | `0.0012f` | Default leg-X step limit |
| `HOLD_ENTER_PITCH` | `0.8 deg` | Enter hold mode pitch threshold |
| `HOLD_EXIT_PITCH` | `1.8 deg` | Exit hold mode pitch threshold |
| `HOLD_ENTER_ROLL` | `1.2 deg` | Enter hold mode roll threshold |
| `HOLD_EXIT_ROLL` | `2.5 deg` | Exit hold mode roll threshold |
| `HOLD_ENTER_RATE` | `12.0 deg/s` | Enter hold mode rate threshold |
| `HOLD_EXIT_RATE` | `20.0 deg/s` | Exit hold mode rate threshold |
| `LEG_SOFTZONE` | `1.5 deg` | Leg balancing soft zone |
| `LEG_HARDZONE` | `7.0 deg` | Leg balancing hard zone |
| `X_ACTIVEZONE` | `2.0 deg` | Leg X active zone |
| `LEG_GAIN_SCALE` | `6.0f` | Leg balancing gain scale |
| `LEG_MIN_DIFF` / `LEG_MAX_DIFF` | `0.0015f` / `0.020f` | Leg height difference clamp |
| `LEG_STEP_LIMIT` | `0.0009f` | Leg difference step limit |
| `X_STEP_LIMIT` | `0.0008f` | Leg X command step limit |
| `Y_STEP_LIMIT` | `0.0006f` | Leg Y command step limit |
| `X_HOLD_BAND` / `Y_HOLD_BAND` | `0.0008f` / `0.0006f` | Hold deadband |

## Odometry And Navigation Thresholds

Sources: `project/code/navigation_data_handling.h`, `project/code/navigation_tracking.h`

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `NAVI_USE_LOCAL_FRAME` | `1` | Use local coordinate frame |
| `WHEEL_DIAMETER` | `0.045F` | Wheel diameter, m |
| `WHEEL_DISRANCE` | `0.190f` | Wheel distance/base width, m |
| `ENCODER_DT` | `0.005f` | Odometry sample time, 5 ms |
| `YAW_HISTORY_LEN` | `600` | 600 samples, about 3000 ms at 5 ms |
| `NAVI_ARC_MIN_YAWRATE_RADPS` | `0.10f` | Use arc model only above this yaw-rate threshold; otherwise use midpoint integration |
| `LOOK_AHEAD_DIST` | `0.6f` | Look-ahead distance |
| `NAVI_POINT_MAX` | `500` | Max recorded waypoints |
| `DISTANCE_THRESHOLD` | `0.10f` | Waypoint arrival threshold |
| `INTERPOLATION_STEP` | `1.0f` | Path interpolation step |
| `ENABLE_PATH_INTERPOLATION` | `0` | Path interpolation disabled |
| `USE_HOST_TARGET_VELOCITY` | `2` | Dynamic PID-planned tracking speed |
| `DEFAULT_TRACKING_VELOCITY` | `300.0f` | Default tracking speed |
| `Navi_Speed_Kp_init` | `220.0f` | Navigation speed planner P |
| `Navi_Speed_Ki_init` | `0.0f` | Navigation speed planner I |
| `Navi_Speed_Kd_init` | `20.0f` | Navigation speed planner D |
| `Navi_Speed_Max_init` | `300.0f` | Navigation speed max |
| `Navi_Speed_MaxStep_init` | `12.0f` | Navigation speed max step |

## Turn, Anti-Stall, And Misc Control Limits

Source: `project/code/control.c`

| Parameter | Value | Meaning |
| --- | ---: | --- |
| Yaw deadband | `1.5f deg` | `Turn()` sets yaw error to zero inside this band |
| Turn integral output limit | `450.0f` | Clamp for `Direction_i * yaw_integral` |
| Turn integral state limit | `15000.0f` | Clamp for yaw integral state |
| Turn output limit | `+/-2200.0f` | Final turn PWM clamp |
| `ANTI_STALL_TARGET_MIN` | `40.0f` | Anti-stall active only above this target speed |
| `ANTI_STALL_ERROR_START` | `60.0f` | Speed error threshold to accumulate assist |
| `ANTI_STALL_RECOVER_ERROR` | `20.0f` | Recovery speed error threshold |
| `ANTI_STALL_RECOVER_RATIO` | `0.85f` | Recovery ratio threshold |
| `ANTI_STALL_INTEGRAL_LIMIT` | `50000.0f` | Anti-stall integral clamp |
| `ANTI_STALL_PWM_GAIN` | `0.04f` | Anti-stall integral-to-PWM gain |
| `ANTI_STALL_PWM_LIMIT` | `4000.0f` | Anti-stall assist PWM limit |
| Airborne accel threshold | `0.5f` | `fabs(accel_z - 1) > 0.5` |
| Output jump limiter | `4.0f` | Local output change limit in `limit_jumps()` |

## Jump And Special Action Parameters

Sources: `project/code/jump_control.c`, `project/code/navigation_action.h`, `project/code/param.c`

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `jump_burst_pwm` | `1200.0f` | Runtime jump burst PWM parameter |
| `jump_burst_ms` | `130.0f` | Runtime jump burst duration parameter |
| `jump_air_retract_y` | `0.03f` | Runtime air retract Y parameter |
| `jump_buffer_y` | `0.05f` | Runtime jump buffer Y parameter |
| `jump_landing_max_ms` | `600.0f` | Runtime landing timeout parameter |
| `Jump_Burst_Pwm_init` | `1200.0f` | Default jump burst PWM |
| `Jump_Burst_Ms_init` | `130.0f` | Default jump burst duration |
| `Jump_Air_Retract_Y_init` | `0.03f` | Default air retract Y |
| `Jump_Buffer_Y_init` | `0.05f` | Default jump buffer Y |
| `Jump_Landing_Max_Ms_init` | `600.0f` | Default landing timeout |
| `JUMP_PREPARE_MS` | `260U` | Jump prepare duration |
| `JUMP_BURST_MS` | `180U` | Jump takeoff duration |
| `JUMP_AIR_RETRACT_MS` | `40U` | Air retract duration |
| `JUMP_RECOVER_MS` | `50U` | Recover duration |
| `JUMP_END_MS` | `50U` | End buffer duration |
| `JUMP_LANDING_MAX_MS` | `600U` | Hard landing timeout |
| `JUMP_LAND_ACCEL_G` | `1.0f` | Landing acceleration threshold |
| `JUMP_AIR_RETRACT_X` / `Y` | `-0.00f` / `0.015f` | Fixed air retract command |
| `JUMP_EXE_BUFFER_X` / `Y` | `+0.00f` / `0.035f` | Fixed execution buffer command |
| `JUMP_RECOVER_X` / `Y` | `0.00f` / `0.03f` | Fixed recover command |
| `NAVI_JUMP_ACTION_MODE` | `2U` | Triple-jump mode |
| `NAVI_JUMP_POSE_UPDATE_MODE` | `1U` | Use normal automatic pose update during jump |
| `NAVI_JUMP_FIXED_FORWARD_M` | `0.30f` | Fixed pose compensation forward distance, only mode 2 |
| `NAVI_JUMP_FIXED_RIGHT_M` | `0.00f` | Fixed pose compensation right distance, only mode 2 |
| `NAVI_JUMP_RUNUP_SPEED` | `350.0f` | Jump run-up speed |
| `NAVI_TRIPLE_JUMP_RAMP_DOWN_MS` | `1500U` | Triple jump ramp-down duration |
| `NAVI_TRIPLE_JUMP_TURN_BACK_MS` | `1300U` | Triple jump turn-back duration |
| `NAVI_TRIPLE_JUMP_RAMP_UP_MS` | `1700U` | Triple jump ramp-up duration |
| `NAVI_TRIPLE_JUMP_STAIR_DOWN_MS` | `1400U` | Triple jump stair-down duration |
| `NAVI_TRIPLE_JUMP_STAIR_SPEED` | `250.0f` | Triple jump stair speed |

# 导航定向与科目三前瞻

- 首次记录HOME点和每次打点导航发车前，车辆必须保持静止；程序会重新采集2秒IMU yaw并用圆均值建立本次参考方向。倒计时期间目标速度强制为0。
- 科目三普通航点段使用线段投影加0.40 m前瞻点计算目标方向。航点距离、10 cm到达判定及原速度规划保持不变。
