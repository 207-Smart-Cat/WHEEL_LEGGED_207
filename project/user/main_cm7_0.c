#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"
#include "imu.h"
#include "engine.h"
#include "control.h"
#include "wifi.h"
#include "remote.h"
#include "screen_display.h"
#include "ipc_shared_data.h"
// **************************** 核间通信区域 ****************************
// 在 CM7_0 和 CM7_1 中都需要加入这段代码

// 将 Core A 的状态数据放在 0x28001000
#pragma location = 0x28001000
__no_init CoreA_Status_t core_a_status; 

// 将 Core B 的指令数据放在 0x28001200 (往后偏移512字节，预留充足空间)
#pragma location = 0x28001200
__no_init CoreB_Command_t core_b_cmd;


// **************************** 宏定义区域 ****************************
// 中断
#define PIT_IMU (PIT_CH0)
#define PIT_Balance (PIT_CH10)
#define PIT_Remote (PIT_CH12)
#define PIT_Engine (PIT_CH13)
#define LED1 (P19_0)
// **************************** 全局变量区域 ****************************

// 电机+舵机（运行）
int duty = 1;
int stop = 0;
extern float pitch1, roll1, yaw1;
float v_buchang;
/*腿部姿态设置*/
extern float x_current, y_current;
// 用于舵机（x，y）位置的调整,务必记住应该给一个符合区间的初始值，否则舵机将不在转动
int stop_flash = 0; // 完赛标志位

int Bridge_position = 1;     // 可能是过单边桥时候需要的（腿部自适应）
int yanshi_biaozhiwei = 100; // 可能是过单边桥时候需要的（腿部自适应模式）
int change_speed = 0;

// **************************** 封装调试部分函数区域 ****************************


// ================= 主函数 =================
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_init();                  // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等

    interrupt_global_disable(); // 初始化外设之前先关闭中断
    //=================================GPIO初始化=======================
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化 LED1 输出 默认高电平 推挽输出模式
    //=================================IMU初始化=======================
    imu_init(LED1);
    pit_ms_init(PIT_IMU, 5);
     //========================遥控器控制初始化==========================
    Remote_Init();
    pit_ms_init(PIT_Remote, 10);//10ms更新一次目标速度
    //=================================平衡动作初始化========================
    Balance_init(); // 初始化平衡控制（设置Kalman滤波的各个参数）
    pit_ms_init(PIT_Balance, 10);
    small_driver_uart_init();           //驱动板通信初始化
    //=================================舵机初始化======================
    pit_ms_init(PIT_Engine, 30); //舵机初始化
     //=================================双核通信初始化======================
    IPC_Init_Shared_Memory();


    interrupt_global_enable(0); // 在初始化后使能中断

    
    system_delay_ms(1000);

    while (true)
    {
        // 此处编写需要循环执行的代码

        
        system_delay_ms(1);
    }
}