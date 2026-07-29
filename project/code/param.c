#include <stdio.h>
#include "pid.h"
#include "param.h"
#include "imu.h"

// ========================================================
// Runtime values. IPC and Flash may overwrite these after startup.
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
const float servo_alpha = 0.55f; // Servo low-pass filter coefficient.

float Speed_p = 0, Speed_i = 0, Speed_d = 0;
float Angle_p = 0, Angle_i = 0, Angle_d = 0;
float Gyro_p  = 0, Gyro_i  = 0, Gyro_d  = 0;
float Air_roll_p = 0, Air_roll_i = 0, Air_roll_d = 0;
float Direction_p = 0, Direction_i = 0, Direction_d = 0;
float leg_Kp = 0, leg_Ki = 0, leg_Kd = 0;
float navi_speed_kp = 0, navi_speed_ki = 0, navi_speed_kd = 0;
float navi_speed_max = 0, navi_speed_max_step = 0;

// ========================================================
// Default values used to initialize the control system.
const float Q_yaw_init = 0.001f, Q_pr_init = 0.003f, Q_bias_init = 0.001f;
const float R_yaw_init = 0.05f, R_pr_init = 0.05f;

const float Speed_p_init = 0.025f,  Speed_i_init = 0.0f, Speed_d_init = 0.0f;
const float Angle_p_init = 12.3999f, Angle_i_init = 0.15f,  Angle_d_init = 0.0f;
const float Gyro_p_init  = 15.0f,   Gyro_i_init  = 0.0f,   Gyro_d_init  = 0.0f;

const float Target_Velocity_init = 0.0f, Target_Angle_init = 0.0f, Target_Motor_Stand_init = 4.0f;

const float Leg_Kp_init = 0.0f,    Leg_Ki_init = 0.0f,    Leg_Kd_init = 0.0f;
const float X_Current_init = 0.0f, Y_Current_init = 0.04f;
const float Leg_X_Gain_init = 5.0f, Leg_X_Limit_init = 0.020f, Leg_X_Min_Step_init = 0.0008f, Leg_X_Step_Limit_init = 0.0012f;
const float Jump_Burst_Pwm_init = 1200.0f, Jump_Burst_Ms_init = 130.0f, Jump_Air_Retract_Y_init = 0.03f, Jump_Buffer_Y_init = 0.05f, Jump_Landing_Max_Ms_init = 600.0f;

const float Air_roll_p_init = 3.0f, Air_roll_i_init = 0.03f, Air_roll_d_init = 0.0015f;
const float Direction_p_init = 15.0f, Direction_i_init = 0.012f, Direction_d_init = 0.875f;

// Navigation estimator defaults.
const float Nav_q_v_init = 2.0f, Nav_q_w_init = 0.0099f;
const float Nav_q_bias_ax_init = 0.015f, Nav_q_bias_w_init = 0.001f;
const float Nav_r_v_normal_init = 0.01f, Nav_r_v_slip_init = 0.01f;
const float Nav_r_w_normal_init = 0.002f, Nav_r_w_slip_init = 0.002f;
const float Nav_r_gyro_init = 0.01f;

// Navigation mode and command defaults.
const float Navi_Mode_Driver_init = 0.0f;
const float Navi_Mode_Map_init = 0.0f;
const float Navi_Trigger_Record_init = 0.0f;
const float Navi_Print_Pose_En_init = 0.0f;
const float Navi_Print_Pose_Period_init = 3000.0f;
const float Navi_Wifi_Cmd_init = 0.0f;
const float Navi_Wifi_Remote_Type_init = 0.0f;
const float Navi_Wifi_In_Action_init = 0.0f;

// Navigation speed-controller defaults.
const float Navi_Speed_Kp_init = 220.0f;
const float Navi_Speed_Ki_init = 0.0f;
const float Navi_Speed_Kd_init = 20.0f;
const float Navi_Speed_Max_init = 300.0f;
const float Navi_Speed_MaxStep_init = 12.0f;

// Magnetometer calibration defaults.
const float Mag_offset_x_init = 10.0f, Mag_offset_y_init = 0.01f;
const float Mag_scale_x_init = 10.0f, Mag_scale_y_init = 0.01f;
