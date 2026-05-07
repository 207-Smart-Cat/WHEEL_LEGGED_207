#ifndef _RUNTIME_STATUS_H_
#define _RUNTIME_STATUS_H_

#include "zf_common_headfile.h"

// 运行时可控制的模块。枚举顺序会作为 module_enable_mask 里的 bit 位序号使用。
typedef enum
{
    RUNTIME_MODULE_MOTOR = 0,        // 电机最终输出开关。关闭后电机占空比会被强制置 0。
    RUNTIME_MODULE_BALANCE,          // 平衡控制计算开关。关闭后平衡环输出会被清零。
    RUNTIME_MODULE_SERVO,            // 腿部舵机控制开关。关闭后舵机保持固定 x/y 腿部位置。
    RUNTIME_MODULE_REMOTE,           // 遥控接管开关。关闭后忽略遥控器给出的目标速度和目标航向。
    RUNTIME_MODULE_NAVIGATION,       // 导航控制预留开关。目前只保存状态，还没有接入导航逻辑。
    RUNTIME_MODULE_DEBUG_OUTPUT,     // 调试输出预留开关。目前只保存状态。
    RUNTIME_MODULE_COUNT
} runtime_module_t;

// 运行状态原因码，用来说明某个模块为什么没有动作或当前处于什么状态。
typedef enum
{
    RUNTIME_REASON_NORMAL = 0,       // 正常运行或没有被门控拦截。
    RUNTIME_REASON_WIFI_OFF,         // WiFi 未连接，电机输出被安全锁住。
    RUNTIME_REASON_MOTOR_OFF,        // 电机输出模块被手动关闭。
    RUNTIME_REASON_BALANCE_OFF,      // 平衡控制模块被手动关闭。
    RUNTIME_REASON_SERVO_OFF,        // 舵机控制模块被手动关闭。
    RUNTIME_REASON_REMOTE_OFF,       // 遥控接管模块被手动关闭。
    RUNTIME_REASON_REMOTE_LOST,      // 遥控器未连接或 SBUS 失联。
    RUNTIME_REASON_REMOTE_STANDBY,   // 遥控器已连接，但 CH5 未进入接管位置。
    RUNTIME_REASON_SERVO_FIXED,      // 舵机处于固定腿长/固定位置模式。
    RUNTIME_REASON_STARTUP_RAMP,     // 电机处于启动斜坡限幅阶段。
    RUNTIME_REASON_COUNT
} runtime_reason_t;

// 当前核心本地保存的一份运行状态快照。Core0/Core1 都通过 core_b_cmd 做同步。
typedef struct
{
    uint32 module_enable_mask;       // 每个 bit 对应一个 runtime_module_t。1 表示开启，0 表示关闭。
    uint8 wifi_connected;            // Core1 上报的 WiFi 当前连接状态。
    uint8 vehicle_mode;              // 开机设置界面选择的小车模式。目前只显示和保存，控制逻辑预留。
    uint8 motor_reason;              // 电机当前运行原因码。
    uint8 balance_reason;            // 平衡控制当前运行原因码。
    uint8 servo_reason;              // 舵机控制当前运行原因码。
    uint8 remote_reason;             // 遥控接管当前运行原因码。
} runtime_status_t;

// 将模块枚举值转换成 module_enable_mask 里的 bit。
#define RUNTIME_MODULE_BIT(module)       (1UL << (uint32)(module))

// 上电后的安全默认开关：允许手动调车所需模块，导航和调试输出默认关闭。
#define RUNTIME_DEFAULT_MODULE_MASK      (RUNTIME_MODULE_BIT(RUNTIME_MODULE_MOTOR) | \
                                          RUNTIME_MODULE_BIT(RUNTIME_MODULE_BALANCE) | \
                                          RUNTIME_MODULE_BIT(RUNTIME_MODULE_SERVO) | \
                                          RUNTIME_MODULE_BIT(RUNTIME_MODULE_REMOTE))

// 当前核心的运行状态缓存。外部文件不要直接改它，统一走 runtime_status.c 里的接口。
extern volatile runtime_status_t g_runtime_status;

/*
  函数名称：Runtime_Status_Init
  变量：无
  返回：无
  作用：将当前核心本地运行状态缓存恢复为安全默认值。共享内存初始化由 IPC_Init_Shared_Memory() 负责。
*/
void Runtime_Status_Init(void);

/*
  函数名称：Runtime_Sync_From_IPC
  变量：无
  返回：无
  作用：从 core_b_cmd 共享内存同步运行状态到 g_runtime_status；如果 IPC 未初始化，则先写入默认状态。
*/
void Runtime_Sync_From_IPC(void);

/*
  函数名称：Runtime_Is_Module_Enabled
  变量：module：需要查询的运行模块
  返回：1 表示模块开启，0 表示模块关闭或参数非法
  作用：查询某个运行模块是否开启，查询前会先从 IPC 同步最新状态。
*/
uint8 Runtime_Is_Module_Enabled(runtime_module_t module);

/*
  函数名称：Runtime_Set_Module_Enabled
  变量：module：需要设置的运行模块；enable：1 开启，0 关闭
  返回：无
  作用：开启或关闭某个运行模块，并把新的开关状态写回 IPC 共享内存。
*/
void Runtime_Set_Module_Enabled(runtime_module_t module, uint8 enable);

/*
  函数名称：Runtime_Toggle_Module
  变量：module：需要翻转开关状态的运行模块
  返回：无
  作用：翻转某个运行模块的开关状态，屏幕设置界面按确认键时会调用这个函数。
*/
void Runtime_Toggle_Module(runtime_module_t module);

/*
  函数名称：Runtime_Set_Wifi_Connected
  变量：connected：1 表示 WiFi 已连接，0 表示 WiFi 未连接
  返回：无
  作用：更新 WiFi 连接状态到 IPC，Core1 的 WiFi 状态更新链路会调用这个函数。
*/
void Runtime_Set_Wifi_Connected(uint8 connected);

/*
  函数名称：Runtime_Get_Wifi_Connected
  变量：无
  返回：1 表示 WiFi 已连接，0 表示 WiFi 未连接
  作用：通过运行状态缓存读取 WiFi 连接状态，Core0 的电机安全门控会用到它。
*/
uint8 Runtime_Get_Wifi_Connected(void);

/*
  函数名称：Runtime_Set_Vehicle_Mode
  变量：mode：当前选择的小车模式编号
  返回：无
  作用：保存当前选择的小车模式到 IPC。第一版只记录和显示，不直接改变控制逻辑。
*/
void Runtime_Set_Vehicle_Mode(uint8 mode);

/*
  函数名称：Runtime_Get_Vehicle_Mode
  变量：无
  返回：当前选择的小车模式编号
  作用：从 IPC 读取当前选择的小车模式，开机设置界面首页会用它显示当前模式。
*/
uint8 Runtime_Get_Vehicle_Mode(void);

/*
  函数名称：Runtime_Set_Motor_Reason
  变量：reason：电机运行原因码
  返回：无
  作用：记录电机当前为什么正常输出或为什么被门控拦截。
*/
void Runtime_Set_Motor_Reason(runtime_reason_t reason);

/*
  函数名称：Runtime_Set_Balance_Reason
  变量：reason：平衡控制运行原因码
  返回：无
  作用：记录平衡控制当前为什么正常运行或为什么被关闭。
*/
void Runtime_Set_Balance_Reason(runtime_reason_t reason);

/*
  函数名称：Runtime_Set_Servo_Reason
  变量：reason：舵机控制运行原因码
  返回：无
  作用：记录舵机当前为什么正常运行、固定位置或被关闭。
*/
void Runtime_Set_Servo_Reason(runtime_reason_t reason);

/*
  函数名称：Runtime_Set_Remote_Reason
  变量：reason：遥控接管运行原因码
  返回：无
  作用：记录遥控当前为什么生效或为什么没有接管。
*/
void Runtime_Set_Remote_Reason(runtime_reason_t reason);

/*
  函数名称：Runtime_Reason_Name
  变量：reason：运行原因码
  返回：原因码对应的字符串
  作用：将运行原因码转换成可打印的文本，供 WiFi/VOFA 日志输出使用。
*/
const char *Runtime_Reason_Name(runtime_reason_t reason);

#endif