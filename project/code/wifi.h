#ifndef __WIFI_H__
#define __WIFI_H__

#include "zf_common_headfile.h" // 确保包含逐飞库的基础头文件


// ==========================================
// ?? 通信协议模式一键切换开关
// 0: 使用 TCP 模式 (1对1可靠传输，适合发遥控指令)
// 1: 使用 UDP 模式 (全局广播模式，适合多台手机同时看波形)
// ==========================================
#define WIFI_PROTOCOL_MODE  1   // <---- 以后你只需要修改这个数字！


// ------ 下面的代码会自动根据上面的开关进行配置，无需手动修改 ------
#if (WIFI_PROTOCOL_MODE == 0)
    #define WIFI_PROTOCOL_STR   "TCP"
    #define WIFI_TARGET_IP      "192.168.230.136" // 【填入你实验室电脑/特定手机的真实IP】
#elif (WIFI_PROTOCOL_MODE == 1)
    #define WIFI_PROTOCOL_STR   "UDP"
    #define WIFI_TARGET_IP      "255.255.255.255" // 全局广播IP，无需修改
#endif

// 通用端口与WIFI设置
#define WIFI_TARGET_PORT    "8086"
#define WIFI_LOCAL_PORT     "6666"
#define WIFI_SSID_TEST          "test207"               //注意zf_device_wifi_spi.h里面也有相关定义，这里已经把那里 的定义注释了，使用本文件的定义
#define WIFI_PASSWORD_TEST      "12345678"

// --- WIFI 工作模式枚举定义 ---
typedef enum {
    WIFI_MODE_SILENT = 0,   // 模式0：静默待机（不发数据，省算力）
    WIFI_MODE_WAVE   = 1,   // 模式1：波形回传（发给虚拟示波器）
    WIFI_MODE_IMAGE  = 2,   // 模式2：图像回传（发给图传上位机）
    WIFI_MODE_LOG    = 3    // 模式3：日志模式（发文本调试信息）
} wifi_mode_t;

// --- 外部变量声明 ---
// 使用 extern 暴露这个变量，这样你在 main 函数里也能随时强制修改它
extern wifi_mode_t current_wifi_mode;

// --- 函数声明 ---
void wifi_init(void);           
void wifi_process_loop(void);   // 负责接收指令（包含切换模式的指令）
void wifi_report_task(void);    // 负责根据当前模式向外发送数据 
#endif
