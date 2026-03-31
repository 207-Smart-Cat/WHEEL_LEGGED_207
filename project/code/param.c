#include <stdio.h>
#include "pid.h"
#include "param.h"
#include "imu.h"
#include <stdio.h>
#include "pid.h"
#include "param.h"
#include "imu.h"

// ========================================================
// 1. 运行时参数定义 (开机后会被 IPC 立刻覆盖)
//不要在这里改初始值，在下面的2改
// ========================================================
float target_velocity = 0;    
float target_angle = 0;       
float target_motor_Stand = 0; 
float x_current = 0, y_current = 0.4; 

pid_param_t motor_speed, motor_Stand, motor_direction, motor_gyro, air_roll_pid, motor_leg_pid;
const float servo_alpha = 0.3f; //一阶平滑，越大越灵敏

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

const float Speed_p_init = 0.06f,  Speed_i_init = 0.0f,   Speed_d_init = 0.0220f;
const float Angle_p_init = 6.0f,   Angle_i_init = 0.0f,   Angle_d_init = 0.4f;
const float Gyro_p_init  = 5.0f,   Gyro_i_init  = 0.5f,   Gyro_d_init  = 0.27f;

const float Target_Velocity_init = 0.0f, Target_Angle_init = 0.0f, Target_Motor_Stand_init = 0.0f;

const float Leg_Kp_init = 0.015f,  Leg_Ki_init = 0.0f,    Leg_Kd_init = -0.007f;
const float X_Current_init = 0.0f, Y_Current_init = 0.04f;

const float Air_roll_p_init = 45.0f, Air_roll_i_init = 0.0f, Air_roll_d_init = 2.0f;
const float Direction_p_init = 0.0099f, Direction_i_init = 0.015f, Direction_d_init = 0.001f;

// 导航参数初始值
const float Nav_q_v_init = 0.01f, Nav_q_w_init = 0.01f;
const float Nav_q_bias_ax_init = 0.002f, Nav_q_bias_w_init = 0.002f;
const float Nav_r_v_normal_init = 0.01f, Nav_r_v_slip_init = 10.0f;
const float Nav_r_w_normal_init = 0.01f, Nav_r_w_slip_init = 10.0f;
const float Nav_r_gyro_init = 0.01f;

// 磁力计参数初始值
const float Mag_offset_x_init = -0.080f, Mag_offset_y_init = 0.040f;
const float Mag_scale_x_init = 1.0f, Mag_scale_y_init = 1.05f;