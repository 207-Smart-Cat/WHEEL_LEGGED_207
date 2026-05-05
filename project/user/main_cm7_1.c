#include "zf_common_headfile.h"
#include "ipc_shared_data.h"
#include "wifi.h"
#include "screen_display.h"
#include "vofa_protocol.h"
// **************************** 核间通信区域 ****************************
// 将 Core A 的状态数据放在 0x28001000
#pragma location = IPC_CORE_A_SHARED_ADDR
__no_init CoreA_Status_t core_a_status;

// 将 Core B 的指令数据放在 0x28001200 (往后偏移512字节，预留充足空间)
#pragma location = IPC_CORE_B_SHARED_ADDR
__no_init CoreB_Command_t core_b_cmd;

// **************************** 宏定义区域 ****************************
// 中断
#define PIT_IPC (PIT_CH15)
#define PIT_WiFi (PIT_CH16)
#define PIT_IPS (PIT_CH17)

// **************************** 全局变量区域 ****************************
int count=0;

// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化

    // 此处编写用户代码 例如外设初始化代码等
    interrupt_global_disable(); // 初始化外设之前先关闭中断
    flash_init();
    //=================================屏幕初始化============================
    screen_display_init();
    pit_ms_init(PIT_IPS, 50);
//    // =================================WIFI模块初始化======================
     wifi_init();
     IPC_Update_Wifi_Status_From_CoreB(wifi_is_connected);
     pit_ms_init(PIT_WiFi, 20);
    //=================================共享缓存以及Flash部分模块初始化===============
    pit_ms_init(PIT_IPC, 5);
    IPC_Load_Params_From_Flash();
    //=================================串口调参初始化======================
    VOFA_UART_Init();


    interrupt_global_enable(0);// 在初始化后使能中断
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
       screen_display_process();               //屏幕显示
        wifi_process_loop();                    //wifi接收数据解析
        wifi_report_task();
        wifi_health_check_task();
        wifi_auto_reconnect_task();
        IPC_Update_Wifi_Status_From_CoreB(wifi_is_connected);
        VOFA_UART_Process();

        system_delay_ms(1);
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************


