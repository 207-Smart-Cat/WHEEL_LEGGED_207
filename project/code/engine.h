#ifndef CODE_ENGINE_H_
#define CODE_ENGINE_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include"control.h"

//#define PWM_1              (ATOM0_CH4_P02_4)//左2 900向上 +
//#define PWM_2              (ATOM2_CH3_P33_7)//左1 900向下 +
//#define PWM_3              (ATOM3_CH2_P33_6)//右1 900向上 -
//#define PWM_4              (ATOM0_CH5_P02_5)//右2 900向下 -
#define PWM_1              (TCPWM_CH13_P00_3)//左2 900向上 +
#define PWM_2              (TCPWM_CH12_P01_0)//左1 900向下 +
#define PWM_3              (TCPWM_CH11_P01_1)//右1 900向上 -
#define PWM_4              (TCPWM_CH11_P05_2)//右2 900向下 -
#define FREQ               (50)
//           ??（行进方向）
//左1(PWM2-ch2)       右1(PWM3-ch3)
//          九轴
//           364
//左2(PWM1-ch1)       右2(PWM4-ch4)
//       （驱动板悬挂）

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
