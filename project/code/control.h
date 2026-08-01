#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "imu.h"
#include "engine.h"
#include "small_driver_uart_control.h"
// #include "jump_control.h"
// #include "image.h"

extern float target_velocity;             // 目标速度值0————400
extern float now_velocity;                // 实际速度值
extern float Encoder_Left, Encoder_Right; // 左右电机编码器值
extern float target_motor_Stand;          // 机械中值
extern float Turn_Pwm;                    // 转向PWM值
extern int stop_flash;                    // 完赛标志位
extern int bridge_high;                   // 单边桥高低标志位
extern int speed_up;                      // 加速标志位
extern pid_param_t motor_direction;       // 方向PID参数
//******************
extern signed short int Motor_Left, Motor_Right;
extern float out_speed_l, out_speed_r, out_angle_l, out_angle_r, out_gyro_l, out_gyro_r;
extern float Turn_Pwm, leg_error;
extern float leg_dbg_speed_tilt, leg_dbg_x_offset, leg_dbg_x_target, leg_dbg_x_cmd, leg_dbg_tick;
extern float leg_dbg_x_gain_used, leg_dbg_x_limit_used, leg_dbg_x_step_used, leg_dbg_x_limit_hit;
extern float anti_stall_dbg_enabled, anti_stall_dbg_integral, anti_stall_dbg_pwm, anti_stall_dbg_clear_reason;
extern float anti_stall_pwm_gain;
extern volatile uint8 bump_control_mode_active, bump_control_run_enabled;
extern volatile float bump_control_target_speed;
extern float vision_dbg_yaw_error, vision_dbg_raw_yaw_rate, vision_dbg_yaw_rate;
extern float vision_dbg_p_output, vision_dbg_i_output, vision_dbg_d_output, vision_dbg_integral;
//******************
void Balance_init(void);
void balance_control(void);
void leg_control(float *x, float *y);
float leg_velocity(float leg_Kp, int Velocity_Pwm);
void adjust_pid_based_on_leg_height(float *current_leg_height);
bool is_airborne();
void Height_PID_Switch(bool high_mode);
void pid_high_init(void);
void pid_low_init(void);
float leg_sensor_filter(float new_val, bool is_first_run);
float max(float a, float b);
float min(float a, float b);
void Turn_Reset(void);
uint8 Vision_Align_Cal_Get_State(void);
uint8 Vision_Align_Cal_Get_Stable_Count(void);
uint8 Vision_Align_Cal_Get_Sample_Count(void);
uint8 Vision_Align_Cal_Get_Left_Sample_Count(void);
uint8 Vision_Align_Cal_Get_Right_Sample_Count(void);
uint8 Vision_Align_Cal_Result_Valid(void);
float Vision_Align_Cal_Get_Result_Yaw(void);
void Vision_Align_Cal_Reset(void);

#endif /* CODE_ENGINE_H_ */
