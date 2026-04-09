#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "kalman_rm.h"
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
//******************
void Balance_init(void);
void balance_control(void);
void leg_control(float *x, float *y);
float leg_velocity(float leg_Kp, int Velocity_Pwm);
void adjust_pid_based_on_leg_height(float *current_leg_height);
bool is_airborne();
void pid_high_init();
float leg_sensor_filter(float new_val, bool is_first_run);
float max(float a, float b);
float min(float a, float b);

#endif /* CODE_ENGINE_H_ */
