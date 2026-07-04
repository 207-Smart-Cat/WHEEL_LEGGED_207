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
#define WIFI_TARGET_PORT        "8086"///
#define WIFI_LOCAL_PORT         "6666"
//#define WIFI_SSID_TEST          "test207"
#define WIFI_SSID_TEST          "zhangtao"
#define WIFI_PASSWORD_TEST      "12345678"

// --- WIFI 工作模式枚举定义 ---
typedef enum {
    WIFI_MODE_SILENT = 0,   // 模式0：静默待机（不发数据，省算力）
    WIFI_MODE_WAVE   = 1,   // 模式1：波形回传（发给虚拟示波器）
    WIFI_MODE_IMAGE  = 2,   // 模式2：图像回传（发给图传上位机）
    WIFI_MODE_LOG    = 3    // 模式3：日志模式（发文本调试信息）
} wifi_mode_t;
#define WIFI_WAVE_MAX_SELECTED 6
#define WIFI_WAVE_EMPTY_SLOT   0xFF

typedef enum {
    WIFI_WAVE_VAR_ROLL = 0,
    WIFI_WAVE_VAR_PITCH,
    WIFI_WAVE_VAR_YAW,
    WIFI_WAVE_VAR_LEFT_SPEED,
    WIFI_WAVE_VAR_RIGHT_SPEED,
    WIFI_WAVE_VAR_LEFT_PWM,
    WIFI_WAVE_VAR_RIGHT_PWM,
    WIFI_WAVE_VAR_SPD_OUT_L,
    WIFI_WAVE_VAR_SPD_OUT_R,
    WIFI_WAVE_VAR_ANG_OUT_L,
    WIFI_WAVE_VAR_ANG_OUT_R,
    WIFI_WAVE_VAR_GYR_OUT_L,
    WIFI_WAVE_VAR_GYR_OUT_R,
    WIFI_WAVE_VAR_TURN_OUT,
    WIFI_WAVE_VAR_LEG_OUT,
    WIFI_WAVE_VAR_LEG_SPEED_TILT,
    WIFI_WAVE_VAR_LEG_X_OFFSET,
    WIFI_WAVE_VAR_LEG_X_TARGET,
    WIFI_WAVE_VAR_LEG_TICK,
    WIFI_WAVE_VAR_BATTERY,
    WIFI_WAVE_VAR_NAV_X,
    WIFI_WAVE_VAR_NAV_Y,
    WIFI_WAVE_VAR_NAV_V,
    WIFI_WAVE_VAR_NAV_W,
    WIFI_WAVE_VAR_NAV_YAW,
    WIFI_WAVE_VAR_NAV_VALID,
    WIFI_WAVE_VAR_LEG_X_GAIN_USED,
    WIFI_WAVE_VAR_LEG_X_LIMIT_USED,
    WIFI_WAVE_VAR_LEG_X_STEP_USED,
    WIFI_WAVE_VAR_LEG_X_LIMIT_HIT,
    WIFI_WAVE_VAR_MOTOR_ZERO_STATE,
    WIFI_WAVE_VAR_MOTOR_ZERO_ELAPSED,
    WIFI_WAVE_VAR_MOTOR_ZERO_RX,
    WIFI_WAVE_VAR_MOTOR_ZERO_SPEED,
    WIFI_WAVE_VAR_MOTOR_ZERO_START,
    WIFI_WAVE_VAR_MOTOR_ZERO_TX,
    WIFI_WAVE_VAR_MOTOR_ZERO_TASK,
    WIFI_WAVE_VAR_MOTOR_ZERO_RX_COUNT,
    WIFI_WAVE_VAR_TARGET_VELOCITY,
    WIFI_WAVE_VAR_TARGET_ANGLE,
    WIFI_WAVE_VAR_TARGET_STAND,
    WIFI_WAVE_VAR_X_CURRENT,
    WIFI_WAVE_VAR_Y_CURRENT,
    WIFI_WAVE_VAR_COUNT
} wifi_wave_var_t;


typedef enum {
    WIFI_BOOT_STATE_NONE = 0,
    WIFI_BOOT_STATE_CONNECTED,
    WIFI_BOOT_STATE_SKIPPED,
    WIFI_BOOT_STATE_FAILED
} wifi_boot_state_t;
// --- 外部变量声明 ---
extern wifi_mode_t current_wifi_mode;
extern volatile uint8 WIFI_Send_flag;
extern uint8 wifi_is_connected; // 供外部查询 WiFi 是否在线
extern uint8 wifi_wave_selected_count;
extern uint8 wifi_wave_selected[WIFI_WAVE_MAX_SELECTED];

// --- API 函数声明 ---
const char *wifi_wave_var_name(wifi_wave_var_t id);
float wifi_wave_get_value(wifi_wave_var_t id);
uint8 wifi_wave_is_selected(uint8 id);
uint8 wifi_wave_toggle_selected(uint8 id);
void wifi_wave_enter_mode(void);
void wifi_wave_send_var_map(void);
uint8 wifi_wave_set_selected_ids(const uint8 *ids, uint8 count);
void wifi_init(void);
uint8 wifi_init_with_skip(uint8 allow_skip);
void wifi_process_loop(void);   // 负责接收指令（包含切换模式的指令）
void wifi_report_task(void);    // 负责根据当前模式向外发送数据
void wifi_health_check_task(void); // 静默模式下的低频在线检测
void wifi_auto_reconnect_task(void); // 【新增】断线自动重连状态机
void wifi_request_reconnect(void);
uint8 wifi_control_is_ready(void);
wifi_boot_state_t wifi_get_boot_state(void);
const char *wifi_get_boot_state_text(void);
uint8 WIFI_Send_Buffer_Checked(const uint8 *data, uint32 len, uint8 flush_now);
void LOG_Printf(const char *format, ...);

#endif

