#include "runtime_status.h"
#include "ipc_shared_data.h"

// 当前核心本地缓存的一份运行状态。跨核共享时以 core_b_cmd 里的字段为准。
volatile runtime_status_t g_runtime_status = {
    RUNTIME_DEFAULT_MODULE_MASK,
    0,
    0,
    RUNTIME_REASON_NORMAL,
    RUNTIME_REASON_NORMAL,
    RUNTIME_REASON_NORMAL,
    RUNTIME_REASON_NORMAL
};

/*
  函数名称：runtime_sanitize_mask
  变量：mask：待检查的模块开关 bitmask
  返回：清除无效 bit 后的模块开关 bitmask
  作用：清除 module_enable_mask 里无效的 bit，避免 IPC 数据异常时开启未定义模块。
*/
static uint32 runtime_sanitize_mask(uint32 mask)
{
    uint32 valid_mask = (1UL << (uint32)RUNTIME_MODULE_COUNT) - 1UL;
    return mask & valid_mask;
}

/*
  函数名称：runtime_sanitize_reason
  变量：reason：待检查的运行原因码
  返回：合法的运行原因码
  作用：防止非法原因码进入运行状态结构体。
*/
static runtime_reason_t runtime_sanitize_reason(runtime_reason_t reason)
{
    if (reason >= RUNTIME_REASON_COUNT)
    {
        return RUNTIME_REASON_NORMAL;
    }
    return reason;
}

/*
  函数名称：Runtime_Status_Init
  变量：无
  返回：无
  作用：只恢复当前核心本地缓存的默认值，上电时共享内存的初始化由 IPC_Init_Shared_Memory() 完成。
*/
void Runtime_Status_Init(void)
{
    g_runtime_status.module_enable_mask = RUNTIME_DEFAULT_MODULE_MASK;
    g_runtime_status.wifi_connected = 0;
    g_runtime_status.vehicle_mode = 0;
    g_runtime_status.motor_reason = RUNTIME_REASON_NORMAL;
    g_runtime_status.balance_reason = RUNTIME_REASON_NORMAL;
    g_runtime_status.servo_reason = RUNTIME_REASON_NORMAL;
    g_runtime_status.remote_reason = RUNTIME_REASON_NORMAL;
}

/*
  函数名称：Runtime_Sync_From_IPC
  变量：无
  返回：无
  作用：从 Core1/Core0 共享指令区同步运行状态；如果共享内存还没有有效运行状态，就先写入安全默认值。
*/
void Runtime_Sync_From_IPC(void)
{
    uint8 motor_reason = g_runtime_status.motor_reason;
    uint8 balance_reason = g_runtime_status.balance_reason;
    uint8 servo_reason = g_runtime_status.servo_reason;
    uint8 remote_reason = g_runtime_status.remote_reason;

    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));

    if (core_b_cmd.runtime_status_valid == 0)
    {
        core_b_cmd.runtime_module_enable_mask = RUNTIME_DEFAULT_MODULE_MASK;
        core_b_cmd.vehicle_mode = 0;
        core_b_cmd.runtime_status_valid = 1;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }

    g_runtime_status.module_enable_mask = runtime_sanitize_mask(core_b_cmd.runtime_module_enable_mask);
    g_runtime_status.wifi_connected = core_b_cmd.wifi_connected;
    g_runtime_status.vehicle_mode = core_b_cmd.vehicle_mode;
    g_runtime_status.motor_reason = motor_reason;
    g_runtime_status.balance_reason = balance_reason;
    g_runtime_status.servo_reason = servo_reason;
    g_runtime_status.remote_reason = remote_reason;
}

/*
  函数名称：Runtime_Is_Module_Enabled
  变量：module：需要判断的运行模块
  返回：1 表示模块开启，0 表示模块关闭或参数非法
  作用：判断某个模块当前是否开启，电机、平衡、舵机、遥控等入口会用这个函数做运行门控。
*/
uint8 Runtime_Is_Module_Enabled(runtime_module_t module)
{
    if (module >= RUNTIME_MODULE_COUNT)
    {
        return 0;
    }

    Runtime_Sync_From_IPC();
    return (g_runtime_status.module_enable_mask & RUNTIME_MODULE_BIT(module)) ? 1 : 0;
}

/*
  函数名称：Runtime_Set_Module_Enabled
  变量：module：需要设置的运行模块；enable：1 开启，0 关闭
  返回：无
  作用：设置某个模块的开关状态，并把新的 mask 发布到共享内存；屏幕设置界面和后续 WiFi/VOFA 指令都可以复用。
*/
void Runtime_Set_Module_Enabled(runtime_module_t module, uint8 enable)
{
    if (module >= RUNTIME_MODULE_COUNT)
    {
        return;
    }

    Runtime_Sync_From_IPC();
    if (enable)
    {
        g_runtime_status.module_enable_mask |= RUNTIME_MODULE_BIT(module);
    }
    else
    {
        g_runtime_status.module_enable_mask &= ~RUNTIME_MODULE_BIT(module);
    }

    core_b_cmd.runtime_module_enable_mask = runtime_sanitize_mask(g_runtime_status.module_enable_mask);
    core_b_cmd.runtime_status_valid = 1;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
}

/*
  函数名称：Runtime_Toggle_Module
  变量：module：需要翻转开关状态的运行模块
  返回：无
  作用：翻转某个模块的开关状态，屏幕模块开关页面选中一项后按确认键会调用这个函数。
*/
void Runtime_Toggle_Module(runtime_module_t module)
{
    Runtime_Set_Module_Enabled(module, Runtime_Is_Module_Enabled(module) ? 0 : 1);
}

/*
  函数名称：Runtime_Set_Wifi_Connected
  变量：connected：1 表示 WiFi 已连接，0 表示 WiFi 未连接
  返回：无
  作用：更新 WiFi 连接状态，同时保留其他运行状态字段不变；Core0 在允许电机输出前会读取这个状态。
*/
void Runtime_Set_Wifi_Connected(uint8 connected)
{
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    g_runtime_status.wifi_connected = connected ? 1 : 0;
    core_b_cmd.wifi_connected = g_runtime_status.wifi_connected;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
}

/*
  函数名称：Runtime_Get_Wifi_Connected
  变量：无
  返回：1 表示 WiFi 已连接，0 表示 WiFi 未连接
  作用：返回共享内存里的 WiFi 连接状态，避免电机安全逻辑直接依赖 core_b_cmd 字段。
*/
uint8 Runtime_Get_Wifi_Connected(void)
{
    Runtime_Sync_From_IPC();
    return g_runtime_status.wifi_connected;
}

/*
  函数名称：Runtime_Set_Vehicle_Mode
  变量：mode：当前选择的小车模式编号
  返回：无
  作用：保存开机设置界面选择的小车模式；当前只负责记录选择，不在这里改变导航或控制策略。
*/
void Runtime_Set_Vehicle_Mode(uint8 mode)
{
    g_runtime_status.vehicle_mode = mode;
    core_b_cmd.vehicle_mode = mode;
    core_b_cmd.runtime_status_valid = 1;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
}

/*
  函数名称：Runtime_Get_Vehicle_Mode
  变量：无
  返回：当前选择的小车模式编号
  作用：读取开机设置界面选择的小车模式，设置界面首页用它显示当前模式。
*/
uint8 Runtime_Get_Vehicle_Mode(void)
{
    Runtime_Sync_From_IPC();
    return g_runtime_status.vehicle_mode;
}

/*
  函数名称：Runtime_Set_Motor_Reason
  变量：reason：电机运行原因码
  返回：无
  作用：记录电机当前为什么正常输出或为什么被门控拦截。
*/
void Runtime_Set_Motor_Reason(runtime_reason_t reason)
{
    g_runtime_status.motor_reason = (uint8)runtime_sanitize_reason(reason);
}

/*
  函数名称：Runtime_Set_Balance_Reason
  变量：reason：平衡控制运行原因码
  返回：无
  作用：记录平衡控制当前为什么正常运行或为什么被关闭。
*/
void Runtime_Set_Balance_Reason(runtime_reason_t reason)
{
    g_runtime_status.balance_reason = (uint8)runtime_sanitize_reason(reason);
}

/*
  函数名称：Runtime_Set_Servo_Reason
  变量：reason：舵机控制运行原因码
  返回：无
  作用：记录舵机当前为什么正常运行、固定位置或被关闭。
*/
void Runtime_Set_Servo_Reason(runtime_reason_t reason)
{
    g_runtime_status.servo_reason = (uint8)runtime_sanitize_reason(reason);
}

/*
  函数名称：Runtime_Set_Remote_Reason
  变量：reason：遥控接管运行原因码
  返回：无
  作用：记录遥控当前为什么生效或为什么没有接管。
*/
void Runtime_Set_Remote_Reason(runtime_reason_t reason)
{
    g_runtime_status.remote_reason = (uint8)runtime_sanitize_reason(reason);
}

/*
  函数名称：Runtime_Reason_Name
  变量：reason：运行原因码
  返回：原因码对应的字符串
  作用：将运行原因码转换成可打印的文本，供 WiFi/VOFA 日志输出使用。
*/
const char *Runtime_Reason_Name(runtime_reason_t reason)
{
    switch (runtime_sanitize_reason(reason))
    {
        case RUNTIME_REASON_NORMAL:         return "NORMAL";
        case RUNTIME_REASON_WIFI_OFF:       return "WIFI_OFF";
        case RUNTIME_REASON_MOTOR_OFF:      return "MOTOR_OFF";
        case RUNTIME_REASON_BALANCE_OFF:    return "BALANCE_OFF";
        case RUNTIME_REASON_SERVO_OFF:      return "SERVO_OFF";
        case RUNTIME_REASON_REMOTE_OFF:     return "REMOTE_OFF";
        case RUNTIME_REASON_REMOTE_LOST:    return "REMOTE_LOST";
        case RUNTIME_REASON_REMOTE_STANDBY: return "REMOTE_STANDBY";
        case RUNTIME_REASON_SERVO_FIXED:    return "SERVO_FIXED";
        case RUNTIME_REASON_STARTUP_RAMP:   return "STARTUP_RAMP";
        default:                            return "UNKNOWN";
    }
}