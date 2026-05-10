#include <stdio.h>
#include "pid.h"
#include "param.h"
#include "imu.h"

// ========================================================
// 1. 运行时参数定义 (开机后会被 IPC 立刻覆盖)
//不要在这里改初始值，在下面的2改
// ========================================================
float target_velocity = 0;
float target_angle = 180.0f;
float target_motor_Stand = 0;
float x_current = 0, y_current = 0.04;
float leg_x_gain = 1.6f, leg_x_limit = 0.018f, leg_x_min_step = 0.0008f, leg_x_step_limit = 0.0012f;
float jump_burst_pwm = 1200.0f, jump_burst_ms = 130.0f, jump_air_retract_y = 0.03f, jump_buffer_y = 0.05f, jump_landing_max_ms = 600.0f;
float mag_offset_x = -0.080f, mag_offset_y = 0.040f;
float mag_scale_x = 1.0f, mag_scale_y = 1.0499f;

pid_param_t motor_speed, motor_Stand, motor_direction, motor_gyro, air_roll_pid, motor_leg_pid;
const float servo_alpha = 0.55f; //一阶平滑，越大越灵敏

float Speed_p = 0, Speed_i = 0, Speed_d = 0;
float Angle_p = 0, Angle_i = 0, Angle_d = 0;
float Gyro_p  = 0, Gyro_i  = 0, Gyro_d  = 0;
float Air_roll_p = 0, Air_roll_i = 0, Air_roll_d = 0;
float Direction_p = 0, Direction_i = 0, Direction_d = 0;
float leg_Kp = 0, leg_Ki = 0, leg_Kd = 0;

// ========================================================
// 2. 【新增】系统全量默认初始值字典 (带有 _init 后缀)
//在这里改初始值
// ========================================================
const float Q_yaw_init = 0.001f, Q_pr_init = 0.003f, Q_bias_init = 0.001f;
const float R_yaw_init = 0.05f, R_pr_init = 0.05f;

const float Speed_p_init = 0.025f,  Speed_i_init = 0.0002f, Speed_d_init = 0.0f;
const float Angle_p_init = 12.3999f, Angle_i_init = 0.15f,  Angle_d_init = 0.15f;
const float Gyro_p_init  = 15.0f,   Gyro_i_init  = 0.0f,   Gyro_d_init  = 0.0f;

const float Target_Velocity_init = 0.0f, Target_Angle_init = 180.0f, Target_Motor_Stand_init = 0.0f;

const float Leg_Kp_init = 0.0f,    Leg_Ki_init = 0.0f,    Leg_Kd_init = 0.0f;
const float X_Current_init = 0.0f, Y_Current_init = 0.05f;
const float Leg_X_Gain_init = 1.6f, Leg_X_Limit_init = 0.018f, Leg_X_Min_Step_init = 0.0008f, Leg_X_Step_Limit_init = 0.0012f;
const float Jump_Burst_Pwm_init = 1200.0f, Jump_Burst_Ms_init = 130.0f, Jump_Air_Retract_Y_init = 0.03f, Jump_Buffer_Y_init = 0.05f, Jump_Landing_Max_Ms_init = 600.0f;

const float Air_roll_p_init = 45.0f, Air_roll_i_init = 0.0f, Air_roll_d_init = 0.0f;
const float Direction_p_init = 14.93f, Direction_i_init = 0.012f, Direction_d_init = 0.875f;

// 导航参数初始值
const float Nav_q_v_init = 0.01f, Nav_q_w_init = 0.01f;
const float Nav_q_bias_ax_init = 0.002f, Nav_q_bias_w_init = 0.0f;
const float Nav_r_v_normal_init = 0.01f, Nav_r_v_slip_init = 10.0f;
const float Nav_r_w_normal_init = 0.0f, Nav_r_w_slip_init = 0.08f;
const float Nav_r_gyro_init = 0.01f;

// 磁力计参数初始值
const float Mag_offset_x_init = -0.080f, Mag_offset_y_init = 0.040f;
const float Mag_scale_x_init = 1.0f, Mag_scale_y_init = 1.0499f;
