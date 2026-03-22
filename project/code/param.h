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
#define PI 3.141592653589793
#define MAX_DUTY (90)       // 电机最大PWM=85%
#define MIN_LEG_LENGTH 0.04 // 最小腿部长度
#define MAX_LEG_LENGTH 0.1  // 最大腿部长度

#define RPITCH_ROLL 0.05f //测量噪声协方差 (增大这个值。这会让滤波器在更新角度时，更少地参考当前有毛刺的加速度计读数)0.05
#define QPITCH_ROLL 0.05f //过程噪声协方差 (减小这个值。这会告诉滤波器更相信陀螺仪的历史惯性积分，使得输出轨迹变得非常平稳)0.05
extern const float servo_alpha;

extern float target_velocity;    // 目标速度
extern float target_angle;       // 目标角度
extern float target_motor_Stand; // 目标电机角度
extern float x_current, y_current;


extern pid_param_t motor_speed;     // 速度PID参数---------------------速度环
extern pid_param_t motor_Stand;     // 电机角度PID参数---------------------角度环
extern pid_param_t motor_direction; // 方向PID参数------------------------方向调整
extern pid_param_t motor_gyro;      // 陀螺仪PID参数------------------------角速度环
extern pid_param_t air_roll_pid;    // 空中控制器参数

extern float Speed_p,Speed_i,Speed_d;
extern float Angle_p,Angle_i,Angle_d;
extern float Gyro_p,Gyro_i,Gyro_d;
extern float Air_roll_p,Air_roll_i,Air_roll_d;
extern float Direction_p,Direction_i,Direction_d;
#endif // PARAMS_H