#ifndef CODE_ENGINE_H_
#define CODE_ENGINE_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include"control.h"


#define PWM_1              (TCPWM_CH13_P00_3)//左2 900向上 +
#define PWM_2              (TCPWM_CH12_P05_3)//左1 改到 P05_3 测试
#define PWM_3              (TCPWM_CH11_P01_1)//右1 900向上 -
#define PWM_4              (TCPWM_CH10_P05_1)//右2 900向下 -
#define FREQ               (50)  //与后期250-1250占空对应，不得更改

// 舵机测试模式
// 0: 关闭测试
// 1: 固定输出指定通道，隔离硬件/映射问题
// 2: 四通道自动测试，依次输出中位/下限/上限
#define SERVO_TEST_MODE    (0)
#define SERVO_TEST_CHANNEL (2)    // 1~4 对应 PWM_1~PWM_4
#define SERVO_TEST_DUTY    (750)  // 直接作用到物理 PWM 输出
#define SERVO_TEST_CENTER_DUTY (750)
#define SERVO_TEST_MIN_DUTY    (350)
#define SERVO_TEST_MAX_DUTY    (1150)
#define SERVO_TEST_STEP_TICKS  (50)   // 舵机控制 20ms 一次时，约 1 秒切换一步

//初始化使用
void engine_init(int pwm1,int pwm2);
uint32 auu(uint32 c);

//主循环使用
void engine_maintain(int pwm1,int pwm2);
void engine_left_maintain(int pwm1,int pwm2);
void engine_right_maintain(int pwm1,int pwm2);
void engine_Stand_change(uint32 left, uint32 right, pid_param_t * pid1, pid_param_t * pid2);

//一次性单独使用
void engine_jump(void);

#endif /* CODE_ENGINE_H_ */



