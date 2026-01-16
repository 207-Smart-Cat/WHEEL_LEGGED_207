
#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);                                              // 时钟配置及系统初始化<务必保留>
    
    debug_init();                                                               // 调试串口初始化
    // 此处编写用户代码 例如外设初始化代码等
    interrupt_global_disable();							// 关闭全局中断
    
    driver_gpio_init();                                                         // 板载普通GPIO功能初始化（按键、LED）
    
    driver_adc_init();                                                          // 电池检测初始化

    motor_flash_init();                                                         // FLASH初始化 读取零点、旋转方向、极对数数据  
    
    motor_control_init();                                                       // 双电机控制初始化    
    
    interrupt_global_enable(0);							// 开启全局中断
    // 此处编写用户代码 例如外设初始化代码等
    for(;;)
    {
        // 此处编写需要循环执行的代码
        
        driver_adc_loop();                                                      // 驱动 ADC 循环检测函数
        
        driver_gpio_loop();                                                     // 驱动 GPIO 循环检测函数
        
        driver_cmd_loop();                                                      // 驱动 控制指令 循环响应函数

        system_delay_ms(DRIVER_RESPONSE_CYCLE);                                 // 主循环延时
        
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
