#include "zf_common_headfile.h"
#include "imu.h"
#define LED1                    (P19_0)
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等

    imu_init (LED1);
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
imu_attitude();
        printf("%.5f,%.5f,%.5f\n",IMU_data.filter_result.pitch,IMU_data.filter_result.yaw,IMU_data.filter_result.roll);
        system_delay_ms(10);
      
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
