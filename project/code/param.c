#include <stdio.h>
#include "pid.h"
#include "param.h"
#include "imu.h"
float target_velocity = 0;  // 目标速度
float target_angle = 0;       // 目标角度（遥控专用））
float target_motor_Stand =9 ; // 目标电机角度

float x_current, y_current = 0.05; // 腿高零点

pid_param_t motor_speed;     // 速度PID参数---------------------速度环
pid_param_t motor_Stand;     // 电机角度PID参数---------------------角度环
pid_param_t motor_direction; // 方向PID参数------------------------方向调整
pid_param_t motor_gyro;      // 陀螺仪PID参数------------------------角速度环
pid_param_t air_roll_pid;    // 空中控制器参数

const float servo_alpha = 0.3f;

float Speed_p = 0.06, Speed_i = 0, Speed_d = 0.0220; // 速度环
//              0.06              0          0.022             
float Angle_p = 6, Angle_i = 0, Angle_d = 0.4; // 角度环
//             6                         0.4                
float Gyro_p = 5, Gyro_i = 0.5, Gyro_d = 0.27; // 角速度环
//             5                         0.27       
float Air_roll_p = 45, Air_roll_i = 0, Air_roll_d = 2; // 空中控制器

float Direction_p = 0.044, Direction_i = 0.00086, Direction_d = 0.85; // 方向调整

