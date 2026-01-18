#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

#define MAX_DUTY            (10 )                                               // 最大 MAX_DUTY% 占空比

int8 duty = 0;
bool dir = true;

int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);      // 时钟配置及系统初始化<务必保留>
    
    debug_init();                       // 调试串口初始化
    // 此处编写用户代码 例如外设初始化代码等
    
    small_driver_uart_init(SMALL_DRIVER_1);
    
    small_driver_uart_init(SMALL_DRIVER_2);
    /*  */
    
    // 此处编写用户代码 例如外设初始化代码等
    for(;;)
    {
        // 此处编写需要循环执行的代码

        
        small_driver_set_duty(SMALL_DRIVER_1, duty * (PWM_DUTY_MAX / 100), -duty * (PWM_DUTY_MAX / 100));   // 计算占空比输出
        
        small_driver_set_duty(SMALL_DRIVER_2, duty * (PWM_DUTY_MAX / 100), -duty * (PWM_DUTY_MAX / 100));   // 计算占空比输出

        if(dir)                                                                 // 根据方向判断计数方向 本例程仅作参考
        {
            duty ++;                                                            // 正向计数
            
            if(duty >= MAX_DUTY)                                                // 达到最大值
            {
                dir = false;                                                    // 变更计数方向
            }
        }
        else
        {
            duty --;                                                            // 反向计数
            
            if(duty <= -MAX_DUTY)                                               // 达到最小值
            {
                dir = true;                                                     // 变更计数方向
            }
        }

        printf("motor_1_left_speed:%d, motor_1_right_speed:%d\r\n", motor_value_1.receive_left_speed_data, motor_value_1.receive_right_speed_data);
        
        printf("motor_2_left_speed:%d, motor_2_right_speed:%d\r\n", motor_value_2.receive_left_speed_data, motor_value_2.receive_right_speed_data);

        system_delay_ms(50);


        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
