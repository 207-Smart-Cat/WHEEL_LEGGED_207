#include "app_headfile.h"
// **************************** 核间通信区域 ****************************
// 将 Core A 的状态数据放在 0x28001000
#pragma location = IPC_CORE_A_SHARED_ADDR
__no_init CoreA_Status_t core_a_status;

// 将 Core A 的日志邮箱放在 0x28001400
#pragma location = IPC_LOG_SHARED_ADDR
__no_init IpcLogBox_t ipc_log_box;

// 将 Core B 的指令数据放在 0x28001800 (扩大日志区，避免日志与参数区重叠)
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
    clock_init(SYSTEM_CLOCK_250M);     // 时钟配置及系统初始化<务必保留>
    debug_info_init();                 // 调试串口信息初始化

    interrupt_global_disable();        // 初始化外设之前先关闭中断

    // ============================= 基础外设初始化 =============================
    flash_init();
    NavStore_Init();
    Runtime_Status_Init();

    screen_display_init();
    screen_boot_begin();
    screen_boot_show_status(NULL, NULL);
    screen_boot_show_status("Core1", "START");
    screen_boot_show_status("Flash", "OK");
    screen_boot_show_status("Screen", "OK");
    screen_boot_show_status("Runtime", "OK");
    CameraAssist_Init();
    screen_boot_show_status("Camera", camera_assist_status.ready ? "OK" : "OFF");
    pit_ms_init(PIT_IPS, 50);

    // ============================= 串口调参与参数初始化 =============================
    VOFA_UART_Init();
    screen_boot_show_status("VOFA", "OK");

    pit_ms_init(PIT_IPC, 5);
    screen_boot_show_status("IPC", "OK");
    IPC_Load_Params_From_Flash();
    screen_boot_show_status("Params", "OK");

    // ============================= WiFi 初始化 =============================
    screen_boot_show_status("WiFi", "TRY");
    wifi_init_with_skip(1);
    IPC_Update_Wifi_Status_From_CoreB(wifi_control_is_ready());
    screen_boot_show_status("WiFi", wifi_get_boot_state_text());
    pit_ms_init(PIT_WiFi, 20);

    screen_boot_show_done(wifi_is_connected, wifi_get_boot_state() == WIFI_BOOT_STATE_SKIPPED);

    interrupt_global_enable(0);        // 初始化后使能中断

    while(true)
    {
        static uint8_t ipc_coreb_update_div = 0;

        screen_display_process();
        NavStore_Task();
        wifi_process_loop();
        wifi_report_task();
        wifi_health_check_task();
        wifi_auto_reconnect_task();
        ipc_coreb_update_div++;
        if (ipc_coreb_update_div >= 10)
        {
            ipc_coreb_update_div = 0;
            IPC_Update_Wifi_Status_From_CoreB(wifi_control_is_ready());
            IPC_Flush_Log_To_CoreB();
        }
        VOFA_UART_Process();

        system_delay_ms(1);
    }
}
// **************************** 代码区域 ****************************


