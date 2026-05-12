#ifndef PARAM_H
#define PARAM_H

#include "pid.h"

#define MIN_X -0.05
#define MAX_X 0.05
#define MIN_Y 0.025
#define MAX_Y 0.14
#define L1 0.06
#define L2 0.09
#define L3 0.09
#define L4 0.06
#define L5 0.038
#ifndef PI
#define PI 3.141592653589793
#endif
#define MAX_DUTY (35)       // motor PWM max = 3500
#define MIN_LEG_LENGTH 0.04 // 最小腿部长度
#define MAX_LEG_LENGTH 0.1  // 最大腿部长度

#define RPITCH_ROLL 0.05f //测量噪声协方差 (增大这个值。这会让滤波器在更新角度时，更少地参考当前有毛刺的加速度计读数)0.05
#define QPITCH_ROLL 0.05f //过程噪声协方差 (减小这个值。这会告诉滤波器更相信陀螺仪的历史惯性积分，使得输出轨迹变得非常平稳)0.05
extern const float servo_alpha;

// ========================================================
// 1. 运行时参数变量声明 (会被 IPC 覆盖实时更新)
// ========================================================
extern float target_velocity;    // 目标速度
extern float target_angle;       // 目标角度
extern float target_motor_Stand; // 目标电机角度
extern float x_current, y_current;
extern float leg_x_gain, leg_x_limit, leg_x_min_step, leg_x_step_limit;
extern float jump_burst_pwm, jump_burst_ms, jump_air_retract_y, jump_buffer_y, jump_landing_max_ms;

extern float Speed_p, Speed_i, Speed_d;
extern float Angle_p, Angle_i, Angle_d;
extern float Gyro_p, Gyro_i, Gyro_d;
extern float Air_roll_p, Air_roll_i, Air_roll_d;
extern float Direction_p, Direction_i, Direction_d;
extern float leg_Kp, leg_Ki, leg_Kd;
extern float mag_offset_x, mag_offset_y, mag_scale_x, mag_scale_y;
extern float wifi_cmd_trigger, wifi_remote_type, wifi_in_action;
extern float vofa_trigger_record, vofa_mode_driver, vofa_mode_map;
extern float vofa_print_pose_en, vofa_print_pose_period;

// ========================================================
// 2. 【新增】系统默认初始值常量声明 (带有 _init 后缀)
// ========================================================
extern const float Q_yaw_init, Q_pr_init, Q_bias_init, R_yaw_init, R_pr_init;
extern const float Speed_p_init, Speed_i_init, Speed_d_init;
extern const float Angle_p_init, Angle_i_init, Angle_d_init;
extern const float Gyro_p_init, Gyro_i_init, Gyro_d_init;
extern const float Target_Velocity_init, Target_Angle_init, Target_Motor_Stand_init;
extern const float Leg_Kp_init, Leg_Ki_init, Leg_Kd_init;
extern const float X_Current_init, Y_Current_init;
extern const float Leg_X_Gain_init, Leg_X_Limit_init, Leg_X_Min_Step_init, Leg_X_Step_Limit_init;
extern const float Jump_Burst_Pwm_init, Jump_Burst_Ms_init, Jump_Air_Retract_Y_init, Jump_Buffer_Y_init, Jump_Landing_Max_Ms_init;
extern const float Air_roll_p_init, Air_roll_i_init, Air_roll_d_init;
extern const float Direction_p_init, Direction_i_init, Direction_d_init;

// 导航与磁力计初始值
extern const float Nav_q_v_init, Nav_q_w_init, Nav_q_bias_ax_init, Nav_q_bias_w_init;
extern const float Nav_r_v_normal_init, Nav_r_v_slip_init, Nav_r_w_normal_init, Nav_r_w_slip_init, Nav_r_gyro_init;
extern const float Navi_Mode_Driver_init, Navi_Mode_Map_init, Navi_Trigger_Record_init;
extern const float Navi_Print_Pose_En_init, Navi_Print_Pose_Period_init;
extern const float Navi_Wifi_Cmd_init, Navi_Wifi_Remote_Type_init, Navi_Wifi_In_Action_init;
extern const float Mag_offset_x_init, Mag_offset_y_init, Mag_scale_x_init, Mag_scale_y_init;

#endif // PARAM_H
