#include "zf_common_headfile.h"
#include "ipc_shared_data.h"
#include "imu.h"
#include "control.h"
#include "param.h"
#include "navigation_data_handling.h"
#include "navigation_tracking.h"
#include "imu.h"
#include "wifi.h"
#include "battery_monitor.h"
#include "remote.h"
#include "vofa_protocol.h"
#include "runtime_status.h"
#include "small_driver_uart_control.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
extern void navi_record_fill_preview(uint16 start, IpcNavRecordPreviewPoint_t *out, uint16 max_count, uint16 *actual_start, uint16 *actual_count);
// --- 1. 绝对地址内存分配 ---
#pragma location = IPC_CORE_A_SHARED_ADDR
__no_init CoreA_Status_t core_a_status;

#pragma location = IPC_LOG_SHARED_ADDR
__no_init IpcLogBox_t ipc_log_box;

#pragma location = IPC_CORE_B_SHARED_ADDR
__no_init CoreB_Command_t core_b_cmd;

// --- 2. 初始化函数 ---
#include <string.h>  // 必须包含这个头文件才能使用 memset

// 【核心映射表】把底层全局变量的地址，按枚举顺序放进这个指针数组里
float* const param_map[PARAM_COUNT] = {
#define PARAM_ITEM(id, runtime_var, init_val, display_name) &runtime_var,
#include "param_registry.def"
#undef PARAM_ITEM
};

const char *const g_param_names[PARAM_COUNT] = {
#define PARAM_ITEM(id, runtime_var, init_val, display_name) display_name,
#include "param_registry.def"
#undef PARAM_ITEM
};
uint64_t IPC_Get_All_Param_Mask(void)
{
    if (PARAM_COUNT >= 64)
    {
        return 0xFFFFFFFFFFFFFFFFULL;
    }
    return ((1ULL << PARAM_COUNT) - 1ULL);
}

void IPC_Request_Param_Update(ParamID_e id, float value)
{
    if (id >= PARAM_COUNT)
    {
        return;
    }

    __disable_irq();
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    core_b_cmd.params[id] = value;
    core_b_cmd.update_mask |= (1ULL << id);
    core_b_cmd.param_update_flag = 1;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    __enable_irq();
}

void IPC_Request_All_Params_Update(void)
{
    __disable_irq();
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    core_b_cmd.update_mask = IPC_Get_All_Param_Mask();
    core_b_cmd.param_update_flag = 1;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    __enable_irq();
}

void IPC_Request_Motor_Zero_Calibration(void)
{
    __disable_irq();
    core_b_cmd.motor_zero_request = 1;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    __enable_irq();
}

void IPC_Request_Nav_Jump(void)
{
    __disable_irq();
    core_b_cmd.nav_jump_request = 1;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    __enable_irq();
}

uint8 IPC_Consume_Motor_Zero_Request_Core0(void)
{
    uint8 request = 0;

    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    if (core_b_cmd.motor_zero_request)
    {
        request = 1;
        core_b_cmd.motor_zero_request = 0;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }

    return request;
}

uint8 IPC_Consume_Nav_Jump_Request_Core0(void)
{
    uint8 request = 0;

    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    if (core_b_cmd.nav_jump_request)
    {
        request = 1;
        core_b_cmd.nav_jump_request = 0;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }

    return request;
}

void IPC_Set_Nav_Record_Preview_Start(uint16 start)
{
    __disable_irq();
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    core_b_cmd.nav_record_preview_start = start;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    __enable_irq();
}
void IPC_Update_Motor_Zero_State_From_Core0(uint8 state)
{
    core_a_status.motor_zero_state = state;
    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
}
// --- 初始化函数 ---
void IPC_Init_Shared_Memory(void) {
    memset(&core_a_status, 0, sizeof(CoreA_Status_t));
    memset(&ipc_log_box, 0, sizeof(IpcLogBox_t));
    memset(&core_b_cmd, 0, sizeof(CoreB_Command_t));

    // 【完美替换】直接从 param.c 读取基准初始配置
#define PARAM_ITEM(id, runtime_var, init_val, display_name) core_b_cmd.params[id] = init_val;
#include "param_registry.def"
#undef PARAM_ITEM

    core_b_cmd.update_mask = IPC_Get_All_Param_Mask();
    core_b_cmd.param_update_flag = 1;
    core_b_cmd.runtime_module_enable_mask = RUNTIME_DEFAULT_MODULE_MASK;
    core_b_cmd.vehicle_mode = 0;
    core_b_cmd.runtime_status_valid = 1;

    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
    SCB_CleanInvalidateDCache_by_Addr(&ipc_log_box, sizeof(ipc_log_box));
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
}

// --- 3. Core A 专属：将全局变量打包进共享内存，并刷入 SRAM ---
void IPC_Push_Status_From_CoreA(void) {
    // 1. 打包高频运动状态与调试输出
#define STATUS_ITEM(type, name, source_expr) core_a_status.name = (source_expr);
#include "status_registry.def"
#undef STATUS_ITEM

    // 2. 【核心新增】打包当前真正在使用的底层参数 (Ground Truth)
    for(int i = 0; i < PARAM_COUNT; i++) {
            core_a_status.act_params[i] = *(param_map[i]);
        }

    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    navi_record_fill_preview(core_b_cmd.nav_record_preview_start,
                             core_a_status.navi_record_preview,
                             IPC_NAV_RECORD_PREVIEW_ROWS,
                             &core_a_status.navi_record_preview_start,
                             &core_a_status.navi_record_preview_count);

    // 3. 更新时间戳防卡死
    core_a_status.heartbeat++;
    core_a_status.motor_reason = g_runtime_status.motor_reason;
    core_a_status.balance_reason = g_runtime_status.balance_reason;
    core_a_status.servo_reason = g_runtime_status.servo_reason;
    core_a_status.remote_reason = g_runtime_status.remote_reason;

    // 4. 将写好的 Cache 刷进真正的 SRAM 中
    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
}

// --- Core A -> Core B 日志队列：Core0 写入，Core1 主循环转发到现有 LOG_Printf ---
void IPC_LOG_Printf(const char *format, ...)
{
    char log_buf[IPC_LOG_TEXT_SIZE];
    uint32 i;
    uint32 write_seq;
    uint32 read_seq;
    uint32 slot;

    va_list args;
    va_start(args, format);
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    va_end(args);
    log_buf[IPC_LOG_TEXT_SIZE - 1U] = '\0';

    __disable_irq();
    SCB_CleanInvalidateDCache_by_Addr(&ipc_log_box, sizeof(ipc_log_box));

    write_seq = ipc_log_box.write_seq;
    read_seq = ipc_log_box.read_seq;
    if ((write_seq - read_seq) >= IPC_LOG_SLOT_COUNT)
    {
        ipc_log_box.read_seq = write_seq - IPC_LOG_SLOT_COUNT + 1U;
        ipc_log_box.dropped_count++;
    }

    slot = write_seq % IPC_LOG_SLOT_COUNT;
    for (i = 0; i < IPC_LOG_TEXT_SIZE; i++)
    {
        ipc_log_box.text[slot][i] = log_buf[i];
        if (log_buf[i] == '\0')
        {
            break;
        }
    }
    if (i >= IPC_LOG_TEXT_SIZE)
    {
        ipc_log_box.text[slot][IPC_LOG_TEXT_SIZE - 1U] = '\0';
    }

    ipc_log_box.write_seq = write_seq + 1U;
    SCB_CleanInvalidateDCache_by_Addr(&ipc_log_box, sizeof(ipc_log_box));
    __enable_irq();
}

void IPC_Flush_Log_To_CoreB(void)
{
    char log_buf[IPC_LOG_TEXT_SIZE];
    uint32 i;
    uint32 read_seq;
    uint32 write_seq;
    uint32 slot;

    SCB_CleanInvalidateDCache_by_Addr(&ipc_log_box, sizeof(ipc_log_box));
    read_seq = ipc_log_box.read_seq;
    write_seq = ipc_log_box.write_seq;
    if (read_seq == write_seq)
    {
        return;
    }

    slot = read_seq % IPC_LOG_SLOT_COUNT;
    for (i = 0; i < IPC_LOG_TEXT_SIZE; i++)
    {
        log_buf[i] = ipc_log_box.text[slot][i];
        if (log_buf[i] == '\0')
        {
            break;
        }
    }
    log_buf[IPC_LOG_TEXT_SIZE - 1U] = '\0';

    ipc_log_box.read_seq = read_seq + 1U;
    SCB_CleanInvalidateDCache_by_Addr(&ipc_log_box, sizeof(ipc_log_box));

    LOG_Printf("%s", log_buf);
}
// --- 4. Core B 专属：从 SRAM 拉取最新数据到结构体 ---
void IPC_Pull_Status_To_CoreB(void) {
    // 让 Core B 的 Cache 失效，强制从真正的 SRAM 读取最新数据
    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
    // 读取完后，Core B 就可以直接在程序里使用 core_a_status.yaw 等变量了
}


/**
 * @brief Core A (Core 0) 专属：从 SRAM 拉取 Core B 发来的调参指令并应用
 * @note  建议放在 Core 0 的 5ms 定时器中断最开始执行
 */
// Core A 定时器调用的精准更新函数
void IPC_Update_Wifi_Status_From_CoreB(uint8 connected) {
    Runtime_Set_Wifi_Connected(connected);
}

uint8 IPC_CoreB_Wifi_Is_Connected(void) {
    return g_runtime_status.wifi_connected;
}
void IPC_Check_And_Apply_Params_To_Core0(void) {
    Runtime_Sync_From_IPC();
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));

    if(core_b_cmd.param_update_flag == 1) {
        for(int i = 0; i < PARAM_COUNT; i++) {
            // 【关键修改：把 1 改成 1ULL】
            if(core_b_cmd.update_mask & (1ULL << i)) {
                *(param_map[i]) = core_b_cmd.params[i];
            }
        }
        core_b_cmd.update_mask = 0;
        core_b_cmd.param_update_flag = 0;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }
}



// ====================================================================
// 逐飞库 Flash 参数存储配置 (仅本文件内部使用)
// ====================================================================
#define PARAM_FLASH_SECTION  (0)  // 选用靠后的扇区
#define PARAM_FLASH_PAGE     (95)
#define PARAM_FLASH_LEGACY_COUNT_BEFORE_JUMP (45)
#define PARAM_FLASH_LEGACY_COUNT_BEFORE_NAVI_CTRL (50)

// ====================================================================
// 功能 1：真正操作硬件，把参数写入 Flash (Core B 调用)
// ====================================================================
void IPC_Save_Params_To_Flash(void) {
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    __disable_irq();
    flash_buffer_clear();

    flash_union_buffer[0].uint32_type = 0x55AA55AA;

    // 一个循环搞定所有参数的写入
    for(int i = 0; i < PARAM_COUNT; i++) {
        flash_union_buffer[i + 1].float_type = core_b_cmd.params[i];
    }

    // 帧尾自动放在参数之后
    flash_union_buffer[PARAM_COUNT + 1].uint32_type = 0x11223344;

    if(flash_check(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE)) {
        flash_erase_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);
    }
    // 数据总长度：1(头) + PARAM_COUNT(数据) + 1(尾)
    flash_write_page_from_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, PARAM_COUNT + 2);
    __enable_irq();
}

// ====================================================================
// 功能 2：单片机上电时，从硬件 Flash 读取参数 (Core B 调用)
// ====================================================================
void IPC_Load_Params_From_Flash(void) {
    uint16 load_count = 0;
    flash_read_page_to_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, PARAM_COUNT + 2);

    if(flash_union_buffer[0].uint32_type == 0x55AA55AA &&
       flash_union_buffer[PARAM_COUNT + 1].uint32_type == 0x11223344)
    {
        load_count = PARAM_COUNT;
    }
    else if(flash_union_buffer[0].uint32_type == 0x55AA55AA &&
            flash_union_buffer[PARAM_FLASH_LEGACY_COUNT_BEFORE_NAVI_CTRL + 1].uint32_type == 0x11223344)
    {
        load_count = PARAM_FLASH_LEGACY_COUNT_BEFORE_NAVI_CTRL;
    }
    else if(flash_union_buffer[0].uint32_type == 0x55AA55AA &&
            flash_union_buffer[PARAM_FLASH_LEGACY_COUNT_BEFORE_JUMP + 1].uint32_type == 0x11223344)
    {
        load_count = PARAM_FLASH_LEGACY_COUNT_BEFORE_JUMP;
    }

    if(load_count > 0)
    {
        VOFA_Set_Param_Rx_Source(VOFA_PARAM_RX_SRC_FLASH);
        LOG_Printf("\r\n[SYS] Valid parameters found in Flash! Loading to Core A...\r\n");
        if(load_count != PARAM_COUNT)
        {
            LOG_Printf("[SYS] Legacy Flash parameter count detected. New params use defaults.\r\n");
        }

        // 先填默认值，再用 Flash 中已有参数覆盖，兼容旧参数表。
#define PARAM_ITEM(id, runtime_var, init_val, display_name) core_b_cmd.params[id] = init_val;
#include "param_registry.def"
#undef PARAM_ITEM
        for(int i = 0; i < load_count; i++) {
            core_b_cmd.params[i] = flash_union_buffer[i + 1].float_type;
        }

// ==========================================================
        // 专治编译器不服的宏：把浮点数安全拆解为 符号、整数、四位小数
        #define F_S(f) ((f) < 0 ? "-" : "")
        #define F_I(f) (int)((f) < 0 ? -(f) : (f))
        #define F_D(f) (int)((((f) < 0 ? -(f) : (f)) - (int)((f) < 0 ? -(f) : (f))) * 10000)
        #define F_D5(f) (int)((((f) < 0 ? -(f) : (f)) - (int)((f) < 0 ? -(f) : (f))) * 100000)
        // ==========================================================

        LOG_Printf("\r\n============= FLASH LOAD RESULT =============\r\n");
        LOG_Printf(" [Filter] Q_yaw: %s%d.%04d, Q_pr: %s%d.%04d, Q_bias: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_Q_YAW]), F_I(core_b_cmd.params[P_Q_YAW]), F_D(core_b_cmd.params[P_Q_YAW]),
               F_S(core_b_cmd.params[P_Q_PR]), F_I(core_b_cmd.params[P_Q_PR]), F_D(core_b_cmd.params[P_Q_PR]),
               F_S(core_b_cmd.params[P_Q_BIAS]), F_I(core_b_cmd.params[P_Q_BIAS]), F_D(core_b_cmd.params[P_Q_BIAS]));

        LOG_Printf(" [Filter] R_yaw: %s%d.%04d, R_pr: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_R_YAW]), F_I(core_b_cmd.params[P_R_YAW]), F_D(core_b_cmd.params[P_R_YAW]),
               F_S(core_b_cmd.params[P_R_PR]), F_I(core_b_cmd.params[P_R_PR]), F_D(core_b_cmd.params[P_R_PR]));

        LOG_Printf("---------------------------------------------\r\n");

        LOG_Printf(" [Speed]  P: %s%d.%04d, I: %s%d.%04d, D: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_SPEED_P]), F_I(core_b_cmd.params[P_SPEED_P]), F_D(core_b_cmd.params[P_SPEED_P]),
               F_S(core_b_cmd.params[P_SPEED_I]), F_I(core_b_cmd.params[P_SPEED_I]), F_D(core_b_cmd.params[P_SPEED_I]),
               F_S(core_b_cmd.params[P_SPEED_D]), F_I(core_b_cmd.params[P_SPEED_D]), F_D(core_b_cmd.params[P_SPEED_D]));

        LOG_Printf(" [Angle]  P: %s%d.%04d, I: %s%d.%04d, D: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_ANGLE_P]), F_I(core_b_cmd.params[P_ANGLE_P]), F_D(core_b_cmd.params[P_ANGLE_P]),
               F_S(core_b_cmd.params[P_ANGLE_I]), F_I(core_b_cmd.params[P_ANGLE_I]), F_D(core_b_cmd.params[P_ANGLE_I]),
               F_S(core_b_cmd.params[P_ANGLE_D]), F_I(core_b_cmd.params[P_ANGLE_D]), F_D(core_b_cmd.params[P_ANGLE_D]));

        LOG_Printf(" [Gyro]   P: %s%d.%04d, I: %s%d.%04d, D: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_GYRO_P]), F_I(core_b_cmd.params[P_GYRO_P]), F_D(core_b_cmd.params[P_GYRO_P]),
               F_S(core_b_cmd.params[P_GYRO_I]), F_I(core_b_cmd.params[P_GYRO_I]), F_D(core_b_cmd.params[P_GYRO_I]),
               F_S(core_b_cmd.params[P_GYRO_D]), F_I(core_b_cmd.params[P_GYRO_D]), F_D(core_b_cmd.params[P_GYRO_D]));

        LOG_Printf("---------------------------------------------\r\n");

        LOG_Printf(" [Target] Vel: %s%d.%04d, Ang: %s%d.%04d, Stand: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_TARGET_VELOCITY]), F_I(core_b_cmd.params[P_TARGET_VELOCITY]), F_D(core_b_cmd.params[P_TARGET_VELOCITY]),
               F_S(core_b_cmd.params[P_TARGET_ANGLE]), F_I(core_b_cmd.params[P_TARGET_ANGLE]), F_D(core_b_cmd.params[P_TARGET_ANGLE]),
               F_S(core_b_cmd.params[P_TARGET_MOTOR_STAND]), F_I(core_b_cmd.params[P_TARGET_MOTOR_STAND]), F_D(core_b_cmd.params[P_TARGET_MOTOR_STAND]));

        LOG_Printf("---------------------------------------------\r\n");

        LOG_Printf(" [ Leg ]  Kp: %s%d.%04d, Ki: %s%d.%04d, Kd: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_LEG_KP]), F_I(core_b_cmd.params[P_LEG_KP]), F_D(core_b_cmd.params[P_LEG_KP]),
               F_S(core_b_cmd.params[P_LEG_KI]), F_I(core_b_cmd.params[P_LEG_KI]), F_D(core_b_cmd.params[P_LEG_KI]),
               F_S(core_b_cmd.params[P_LEG_KD]), F_I(core_b_cmd.params[P_LEG_KD]), F_D(core_b_cmd.params[P_LEG_KD]));

        LOG_Printf(" [ Pos ]  X: %s%d.%04d, Y: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_X_CURRENT]), F_I(core_b_cmd.params[P_X_CURRENT]), F_D(core_b_cmd.params[P_X_CURRENT]),
               F_S(core_b_cmd.params[P_Y_CURRENT]), F_I(core_b_cmd.params[P_Y_CURRENT]), F_D(core_b_cmd.params[P_Y_CURRENT]));
        LOG_Printf(" [Leg X]  G: %s%d.%04d, L: %s%d.%04d, Mn: %s%d.%04d, St: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_LEG_X_GAIN]), F_I(core_b_cmd.params[P_LEG_X_GAIN]), F_D(core_b_cmd.params[P_LEG_X_GAIN]),
               F_S(core_b_cmd.params[P_LEG_X_LIMIT]), F_I(core_b_cmd.params[P_LEG_X_LIMIT]), F_D(core_b_cmd.params[P_LEG_X_LIMIT]),
               F_S(core_b_cmd.params[P_LEG_X_MIN_STEP]), F_I(core_b_cmd.params[P_LEG_X_MIN_STEP]), F_D(core_b_cmd.params[P_LEG_X_MIN_STEP]),
               F_S(core_b_cmd.params[P_LEG_X_STEP_LIMIT]), F_I(core_b_cmd.params[P_LEG_X_STEP_LIMIT]), F_D(core_b_cmd.params[P_LEG_X_STEP_LIMIT]));

        LOG_Printf(" [ Air ]  P: %s%d.%04d, I: %s%d.%04d, D: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_AIR_ROLL_P]), F_I(core_b_cmd.params[P_AIR_ROLL_P]), F_D(core_b_cmd.params[P_AIR_ROLL_P]),
               F_S(core_b_cmd.params[P_AIR_ROLL_I]), F_I(core_b_cmd.params[P_AIR_ROLL_I]), F_D(core_b_cmd.params[P_AIR_ROLL_I]),
               F_S(core_b_cmd.params[P_AIR_ROLL_D]), F_I(core_b_cmd.params[P_AIR_ROLL_D]), F_D(core_b_cmd.params[P_AIR_ROLL_D]));

        LOG_Printf(" [ Dir ]  P: %s%d.%04d, I: %s%d.%04d, D: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_DIR_P]), F_I(core_b_cmd.params[P_DIR_P]), F_D(core_b_cmd.params[P_DIR_P]),
               F_S(core_b_cmd.params[P_DIR_I]), F_I(core_b_cmd.params[P_DIR_I]), F_D(core_b_cmd.params[P_DIR_I]),
               F_S(core_b_cmd.params[P_DIR_D]), F_I(core_b_cmd.params[P_DIR_D]), F_D(core_b_cmd.params[P_DIR_D]));

        LOG_Printf(" [ Nav ]  qv: %s%d.%05d, qw: %s%d.%05d, qba: %s%d.%05d\r\n",
               F_S(core_b_cmd.params[P_NAV_Q_V]), F_I(core_b_cmd.params[P_NAV_Q_V]), F_D5(core_b_cmd.params[P_NAV_Q_V]),
               F_S(core_b_cmd.params[P_NAV_Q_W]), F_I(core_b_cmd.params[P_NAV_Q_W]), F_D5(core_b_cmd.params[P_NAV_Q_W]),
               F_S(core_b_cmd.params[P_NAV_Q_BIAS_AX]), F_I(core_b_cmd.params[P_NAV_Q_BIAS_AX]), F_D5(core_b_cmd.params[P_NAV_Q_BIAS_AX]));

        LOG_Printf(" [ Nav ]  qbw: %s%d.%05d, rvn: %s%d.%05d, rvs: %s%d.%05d\r\n",
               F_S(core_b_cmd.params[P_NAV_Q_BIAS_W]), F_I(core_b_cmd.params[P_NAV_Q_BIAS_W]), F_D5(core_b_cmd.params[P_NAV_Q_BIAS_W]),
               F_S(core_b_cmd.params[P_NAV_R_V_NORMAL]), F_I(core_b_cmd.params[P_NAV_R_V_NORMAL]), F_D5(core_b_cmd.params[P_NAV_R_V_NORMAL]),
               F_S(core_b_cmd.params[P_NAV_R_V_SLIP]), F_I(core_b_cmd.params[P_NAV_R_V_SLIP]), F_D5(core_b_cmd.params[P_NAV_R_V_SLIP]));

        LOG_Printf(" [ Nav ]  rwn: %s%d.%05d, rws: %s%d.%05d, rgy: %s%d.%05d\r\n",
               F_S(core_b_cmd.params[P_NAV_R_W_NORMAL]), F_I(core_b_cmd.params[P_NAV_R_W_NORMAL]), F_D5(core_b_cmd.params[P_NAV_R_W_NORMAL]),
               F_S(core_b_cmd.params[P_NAV_R_W_SLIP]), F_I(core_b_cmd.params[P_NAV_R_W_SLIP]), F_D5(core_b_cmd.params[P_NAV_R_W_SLIP]),
               F_S(core_b_cmd.params[P_NAV_R_GYRO]), F_I(core_b_cmd.params[P_NAV_R_GYRO]), F_D5(core_b_cmd.params[P_NAV_R_GYRO]));
        LOG_Printf(" [NavCtl] Driver: %s%d.%04d, Map: %s%d.%04d, Rec: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_NAVI_MODE_DRIVER]), F_I(core_b_cmd.params[P_NAVI_MODE_DRIVER]), F_D(core_b_cmd.params[P_NAVI_MODE_DRIVER]),
               F_S(core_b_cmd.params[P_NAVI_MODE_MAP]), F_I(core_b_cmd.params[P_NAVI_MODE_MAP]), F_D(core_b_cmd.params[P_NAVI_MODE_MAP]),
               F_S(core_b_cmd.params[P_NAVI_TRIGGER_RECORD]), F_I(core_b_cmd.params[P_NAVI_TRIGGER_RECORD]), F_D(core_b_cmd.params[P_NAVI_TRIGGER_RECORD]));

        LOG_Printf(" [NavCtl] Print: %s%d.%04d, Period: %s%d.%04d, Cmd: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_NAVI_PRINT_POSE_EN]), F_I(core_b_cmd.params[P_NAVI_PRINT_POSE_EN]), F_D(core_b_cmd.params[P_NAVI_PRINT_POSE_EN]),
               F_S(core_b_cmd.params[P_NAVI_PRINT_PERIOD]), F_I(core_b_cmd.params[P_NAVI_PRINT_PERIOD]), F_D(core_b_cmd.params[P_NAVI_PRINT_PERIOD]),
               F_S(core_b_cmd.params[P_NAVI_WIFI_CMD]), F_I(core_b_cmd.params[P_NAVI_WIFI_CMD]), F_D(core_b_cmd.params[P_NAVI_WIFI_CMD]));

        LOG_Printf(" [NavCtl] Type: %s%d.%04d, Action: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_NAVI_WIFI_TYPE]), F_I(core_b_cmd.params[P_NAVI_WIFI_TYPE]), F_D(core_b_cmd.params[P_NAVI_WIFI_TYPE]),
               F_S(core_b_cmd.params[P_NAVI_WIFI_ACTION]), F_I(core_b_cmd.params[P_NAVI_WIFI_ACTION]), F_D(core_b_cmd.params[P_NAVI_WIFI_ACTION]));

        LOG_Printf(" [NavSpd] Kp: %s%d.%04d, Ki: %s%d.%04d, Kd: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_NAVI_SPEED_KP]), F_I(core_b_cmd.params[P_NAVI_SPEED_KP]), F_D(core_b_cmd.params[P_NAVI_SPEED_KP]),
               F_S(core_b_cmd.params[P_NAVI_SPEED_KI]), F_I(core_b_cmd.params[P_NAVI_SPEED_KI]), F_D(core_b_cmd.params[P_NAVI_SPEED_KI]),
               F_S(core_b_cmd.params[P_NAVI_SPEED_KD]), F_I(core_b_cmd.params[P_NAVI_SPEED_KD]), F_D(core_b_cmd.params[P_NAVI_SPEED_KD]));

        LOG_Printf(" [NavSpd] Max: %s%d.%04d, Step: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_NAVI_SPEED_MAX]), F_I(core_b_cmd.params[P_NAVI_SPEED_MAX]), F_D(core_b_cmd.params[P_NAVI_SPEED_MAX]),
               F_S(core_b_cmd.params[P_NAVI_SPEED_MAX_STEP]), F_I(core_b_cmd.params[P_NAVI_SPEED_MAX_STEP]), F_D(core_b_cmd.params[P_NAVI_SPEED_MAX_STEP]));
        LOG_Printf(" [ Mag ]  off_x: %s%d.%04d, off_y: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_MAG_OFFSET_X]), F_I(core_b_cmd.params[P_MAG_OFFSET_X]), F_D(core_b_cmd.params[P_MAG_OFFSET_X]),
               F_S(core_b_cmd.params[P_MAG_OFFSET_Y]), F_I(core_b_cmd.params[P_MAG_OFFSET_Y]), F_D(core_b_cmd.params[P_MAG_OFFSET_Y]));

        LOG_Printf(" [ Mag ]  scl_x: %s%d.%04d, scl_y: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_MAG_SCALE_X]), F_I(core_b_cmd.params[P_MAG_SCALE_X]), F_D(core_b_cmd.params[P_MAG_SCALE_X]),
               F_S(core_b_cmd.params[P_MAG_SCALE_Y]), F_I(core_b_cmd.params[P_MAG_SCALE_Y]), F_D(core_b_cmd.params[P_MAG_SCALE_Y]));
        LOG_Printf(" [Jump ]  PWM: %s%d.%04d, Bms: %s%d.%04d, AirY: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_JUMP_BURST_PWM]), F_I(core_b_cmd.params[P_JUMP_BURST_PWM]), F_D(core_b_cmd.params[P_JUMP_BURST_PWM]),
               F_S(core_b_cmd.params[P_JUMP_BURST_MS]), F_I(core_b_cmd.params[P_JUMP_BURST_MS]), F_D(core_b_cmd.params[P_JUMP_BURST_MS]),
               F_S(core_b_cmd.params[P_JUMP_AIR_RETRACT_Y]), F_I(core_b_cmd.params[P_JUMP_AIR_RETRACT_Y]), F_D(core_b_cmd.params[P_JUMP_AIR_RETRACT_Y]));

        LOG_Printf(" [Jump ]  BufY: %s%d.%04d, LandMax: %s%d.%04d\r\n",
               F_S(core_b_cmd.params[P_JUMP_BUFFER_Y]), F_I(core_b_cmd.params[P_JUMP_BUFFER_Y]), F_D(core_b_cmd.params[P_JUMP_BUFFER_Y]),
               F_S(core_b_cmd.params[P_JUMP_LANDING_MAX_MS]), F_I(core_b_cmd.params[P_JUMP_LANDING_MAX_MS]), F_D(core_b_cmd.params[P_JUMP_LANDING_MAX_MS]));

        LOG_Printf("=============================================\r\n\r\n");
        if (VOFA_Get_Param_Log_Detail())
        {
            VOFA_Log_Param_Bulk("[SYS] Flash parameters applied", "FLASH_LOAD", PARAM_COUNT);
        }

        // 3. 敲响门铃，让 Core A 一次性全量更新
        core_b_cmd.update_mask = IPC_Get_All_Param_Mask(); // 【关键修改：64个1】
        core_b_cmd.param_update_flag = 1;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }
    else
    {
        VOFA_Set_Param_Rx_Source(VOFA_PARAM_RX_SRC_DEFAULT);
        LOG_Printf("\r\n[SYS] Flash is empty or invalid. Using default params.\r\n");
        if (VOFA_Get_Param_Log_Detail())
        {
            VOFA_Log_Param_Bulk("[SYS] Default parameters applied", "DEFAULT_LOAD", PARAM_COUNT);
        }

        // ?? 【必须补上这三行救命代码！】
        // 如果 Flash 读取失败，必须敲响门铃，把刚才在 Init 里装填的 _init 出厂默认值强行同步给 Core 0
        core_b_cmd.update_mask = IPC_Get_All_Param_Mask();
        core_b_cmd.param_update_flag = 1;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }
}

// ================= Multi-group navigation record storage =================

#define NAV_STORE_MAGIC              (0x4E415647UL)
#define NAV_STORE_VERSION            (1U)
#define NAV_STORE_BANK_PAGES         (4U)
#define NAV_STORE_GROUP_PAGES        (8U)
#define NAV_STORE_IMAGE_BYTES        (NAV_STORE_BANK_PAGES * FLASH_PAGE_SIZE)
#define NAV_STORE_HEADER_BYTES       (32U)
#define NAV_STORE_CORE0_SAVE_TICKS   (100U)
#define NAV_STORE_CORE1_SAVE_TICKS   (20U)
#define NAV_STORE_NO_BANK            (0xFFU)

#pragma pack(push, 1)
typedef struct {
    uint32 magic;
    uint16 version;
    uint8 group;
    uint8 flags;
    uint32 sequence;
    uint16 count;
    uint16 record_size;
    uint32 payload_crc;
    uint32 header_crc;
    uint32 reserved[2];
} NavStoreHeader_t;
#pragma pack(pop)

typedef char nav_store_header_must_be_32_bytes[(sizeof(NavStoreHeader_t) == NAV_STORE_HEADER_BYTES) ? 1 : -1];
typedef char nav_store_image_must_hold_500_points[(NAV_STORE_HEADER_BYTES + NAV_GROUP_POINT_MAX * sizeof(IpcNavFlashPoint_t) <= NAV_STORE_IMAGE_BYTES) ? 1 : -1];
typedef char nav_store_page_layout_must_use_0_to_79[((NAV_GROUP_COUNT * NAV_STORE_GROUP_PAGES) == 80U && PARAM_FLASH_PAGE == 95U) ? 1 : -1];

#if defined(CY_CORE_CM7_1)

static uint32 nav_store_crc32(const uint8 *data, uint32 length)
{
    uint32 crc = 0xFFFFFFFFUL;
    uint32 i;
    uint8 bit;

    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8U; bit++)
        {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
        }
    }
    return ~crc;
}

static NavGroupSummary_t g_nav_group_summary[NAV_GROUP_COUNT];
static IpcNavFlashPoint_t g_nav_store_points[NAV_GROUP_POINT_MAX];
static uint32 g_nav_store_image[NAV_STORE_IMAGE_BYTES / sizeof(uint32)];
static uint16 g_nav_store_count = 0;
static uint8 g_nav_store_selected_group = 0;
static volatile uint16 g_nav_store_save_delay = 0;
static volatile uint8 g_nav_store_save_due = 0;
static uint8 g_nav_store_local_dirty = 0;
static uint32 g_nav_record_tx_seen = 0;
static uint32 g_nav_load_seq_next = 0;
static uint16 g_nav_load_start = 0;
static uint8 g_nav_load_group = 0;
static uint8 g_nav_load_intent = NAV_GROUP_INTENT_NONE;
static uint8 g_nav_load_busy = 0;
static uint8 g_nav_load_wait_ack = 0;

static uint32 nav_store_bank_base_page(uint8 group, uint8 bank)
{
    return (uint32)group * NAV_STORE_GROUP_PAGES + (uint32)bank * NAV_STORE_BANK_PAGES;
}

static int nav_store_read_bank(uint8 group, uint8 bank, IpcNavFlashPoint_t *out_points, NavStoreHeader_t *out_header)
{
    NavStoreHeader_t header;
    NavStoreHeader_t header_for_crc;
    uint32 page;
    uint32 stored_header_crc;
    uint8 *image_bytes = (uint8 *)g_nav_store_image;

    for (page = 0; page < NAV_STORE_BANK_PAGES; page++)
    {
        flash_read_page_to_buffer(0, nav_store_bank_base_page(group, bank) + page, FLASH_PAGE_LENGTH);
        memcpy(&image_bytes[page * FLASH_PAGE_SIZE], flash_union_buffer, FLASH_PAGE_SIZE);
    }

    memcpy(&header, image_bytes, sizeof(header));
    if (header.magic == 0xFFFFFFFFUL)
    {
        return 0;
    }
    // Work Flash may contain data written by earlier firmware versions.  It is
    // not a damaged navigation record unless it carries this format's magic.
    if (header.magic != NAV_STORE_MAGIC)
    {
        return 0;
    }
    if (header.version != NAV_STORE_VERSION ||
        header.group != group ||
        header.record_size != sizeof(IpcNavFlashPoint_t) ||
        header.count > NAV_GROUP_POINT_MAX)
    {
        return -1;
    }

    header_for_crc = header;
    stored_header_crc = header_for_crc.header_crc;
    header_for_crc.header_crc = 0;
    if (stored_header_crc != nav_store_crc32((const uint8 *)&header_for_crc, sizeof(header_for_crc)))
    {
        return -1;
    }
    if (header.payload_crc != nav_store_crc32(&image_bytes[NAV_STORE_HEADER_BYTES],
                                              (uint32)header.count * sizeof(IpcNavFlashPoint_t)))
    {
        return -1;
    }

    if (out_points != NULL && header.count > 0U)
    {
        memcpy(out_points, &image_bytes[NAV_STORE_HEADER_BYTES],
               (uint32)header.count * sizeof(IpcNavFlashPoint_t));
    }
    if (out_header != NULL)
    {
        *out_header = header;
    }
    return 1;
}

static uint8 nav_store_sequence_newer(uint32 lhs, uint32 rhs)
{
    return ((int32)(lhs - rhs) > 0) ? 1U : 0U;
}

static void nav_store_scan_group(uint8 group)
{
    NavStoreHeader_t header_a;
    NavStoreHeader_t header_b;
    int state_a = nav_store_read_bank(group, 0, NULL, &header_a);
    int state_b = nav_store_read_bank(group, 1, NULL, &header_b);
    NavGroupSummary_t *summary = &g_nav_group_summary[group];

    memset(summary, 0, sizeof(*summary));
    summary->active_bank = NAV_STORE_NO_BANK;

    if (state_a == 1 && state_b == 1)
    {
        if (nav_store_sequence_newer(header_b.sequence, header_a.sequence))
        {
            summary->active_bank = 1;
            summary->sequence = header_b.sequence;
            summary->count = header_b.count;
        }
        else
        {
            summary->active_bank = 0;
            summary->sequence = header_a.sequence;
            summary->count = header_a.count;
        }
        summary->state = (summary->count == 0U) ? NAV_STORE_EMPTY : NAV_STORE_SAVED;
    }
    else if (state_a == 1 || state_b == 1)
    {
        NavStoreHeader_t *valid_header = (state_a == 1) ? &header_a : &header_b;
        summary->active_bank = (state_a == 1) ? 0U : 1U;
        summary->sequence = valid_header->sequence;
        summary->count = valid_header->count;
        summary->state = (summary->count == 0U) ? NAV_STORE_EMPTY : NAV_STORE_SAVED;
    }
    else if (state_a == 0 && state_b == 0)
    {
        summary->state = NAV_STORE_EMPTY;
    }
    else
    {
        summary->state = NAV_STORE_DAMAGED;
    }
}

static uint8 nav_store_load_group(uint8 group)
{
    NavGroupSummary_t *summary;
    NavStoreHeader_t header;

    if (group >= NAV_GROUP_COUNT)
    {
        return 0;
    }
    summary = &g_nav_group_summary[group];
    g_nav_store_selected_group = group;
    g_nav_store_count = 0;
    memset(g_nav_store_points, 0, sizeof(g_nav_store_points));

    if (summary->state == NAV_STORE_EMPTY)
    {
        return 1;
    }
    if (summary->state == NAV_STORE_DAMAGED || summary->active_bank >= 2U)
    {
        return 0;
    }
    if (nav_store_read_bank(group, summary->active_bank, g_nav_store_points, &header) != 1)
    {
        nav_store_scan_group(group);
        return 0;
    }
    g_nav_store_count = header.count;
    return 1;
}

static uint8 nav_store_commit_selected(void)
{
    NavStoreHeader_t header;
    NavStoreHeader_t verify_header;
    NavGroupSummary_t *summary = &g_nav_group_summary[g_nav_store_selected_group];
    uint8 target_bank = (summary->active_bank == 0U) ? 1U : 0U;
    uint8 *image_bytes = (uint8 *)g_nav_store_image;
    uint32 used_bytes;
    uint32 used_pages;
    int32 page;

    memset(g_nav_store_image, 0xFF, sizeof(g_nav_store_image));
    memset(&header, 0, sizeof(header));
    header.magic = NAV_STORE_MAGIC;
    header.version = NAV_STORE_VERSION;
    header.group = g_nav_store_selected_group;
    header.sequence = summary->sequence + 1U;
    header.count = g_nav_store_count;
    header.record_size = sizeof(IpcNavFlashPoint_t);
    if (g_nav_store_count > 0U)
    {
        memcpy(&image_bytes[NAV_STORE_HEADER_BYTES], g_nav_store_points,
               (uint32)g_nav_store_count * sizeof(IpcNavFlashPoint_t));
    }
    header.payload_crc = nav_store_crc32(&image_bytes[NAV_STORE_HEADER_BYTES],
                                         (uint32)g_nav_store_count * sizeof(IpcNavFlashPoint_t));
    header.header_crc = 0;
    header.header_crc = nav_store_crc32((const uint8 *)&header, sizeof(header));
    memcpy(image_bytes, &header, sizeof(header));

    used_bytes = NAV_STORE_HEADER_BYTES + (uint32)g_nav_store_count * sizeof(IpcNavFlashPoint_t);
    used_pages = (used_bytes + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE;
    summary->state = NAV_STORE_SAVING;

    for (page = (int32)used_pages - 1; page >= 1; page--)
    {
        memcpy(flash_union_buffer, &image_bytes[(uint32)page * FLASH_PAGE_SIZE], FLASH_PAGE_SIZE);
        flash_write_page_from_buffer(0, nav_store_bank_base_page(g_nav_store_selected_group, target_bank) + (uint32)page,
                                     FLASH_PAGE_LENGTH);
    }
    memcpy(flash_union_buffer, image_bytes, FLASH_PAGE_SIZE);
    flash_write_page_from_buffer(0, nav_store_bank_base_page(g_nav_store_selected_group, target_bank), FLASH_PAGE_LENGTH);

    if (nav_store_read_bank(g_nav_store_selected_group, target_bank, NULL, &verify_header) != 1 ||
        verify_header.sequence != header.sequence || verify_header.count != header.count)
    {
        summary->state = NAV_STORE_ERROR;
        return 0;
    }

    summary->active_bank = target_bank;
    summary->sequence = header.sequence;
    summary->count = header.count;
    summary->state = (header.count == 0U) ? NAV_STORE_EMPTY : NAV_STORE_SAVED;
    g_nav_store_local_dirty = 0;
    g_nav_store_save_delay = 0;
    return 1;
}

void NavStore_Init(void)
{
    uint8 group;
    memset(g_nav_group_summary, 0, sizeof(g_nav_group_summary));
    for (group = 0; group < NAV_GROUP_COUNT; group++)
    {
        nav_store_scan_group(group);
    }
    (void)nav_store_load_group(0);
}

const NavGroupSummary_t *NavStore_Get_Summary(uint8 group)
{
    return (group < NAV_GROUP_COUNT) ? &g_nav_group_summary[group] : NULL;
}

uint8 NavStore_Select_For_View(uint8 group)
{
    if (g_nav_store_local_dirty && !nav_store_commit_selected())
    {
        return 0;
    }
    return nav_store_load_group(group);
}

uint8 NavStore_Get_Selected_Group(void)
{
    return g_nav_store_selected_group;
}

uint16 NavStore_Get_Selected_Count(void)
{
    return g_nav_store_count;
}

uint8 NavStore_Get_Point(uint16 index, IpcNavFlashPoint_t *point)
{
    if (point == NULL || index >= g_nav_store_count)
    {
        return 0;
    }
    *point = g_nav_store_points[index];
    return 1;
}

uint8 NavStore_Undo_Selected(void)
{
    if (g_nav_store_count == 0U || g_nav_group_summary[g_nav_store_selected_group].state == NAV_STORE_DAMAGED)
    {
        return 0;
    }
    g_nav_store_count--;
    memset(&g_nav_store_points[g_nav_store_count], 0, sizeof(g_nav_store_points[g_nav_store_count]));
    g_nav_group_summary[g_nav_store_selected_group].count = g_nav_store_count;
    g_nav_group_summary[g_nav_store_selected_group].state = NAV_STORE_DIRTY;
    g_nav_store_local_dirty = 1;
    g_nav_store_save_delay = NAV_STORE_CORE1_SAVE_TICKS;
    g_nav_store_save_due = 0;
    return 1;
}

uint8 NavStore_Clear_Selected(void)
{
    g_nav_store_count = 0;
    memset(g_nav_store_points, 0, sizeof(g_nav_store_points));
    g_nav_group_summary[g_nav_store_selected_group].count = 0;
    g_nav_group_summary[g_nav_store_selected_group].state = NAV_STORE_DIRTY;
    g_nav_store_local_dirty = 1;
    g_nav_store_save_delay = 0;
    return nav_store_commit_selected();
}

uint8 NavStore_Flush_Selected(void)
{
    if (!g_nav_store_local_dirty)
    {
        return 1;
    }

    g_nav_store_save_delay = 0;
    g_nav_store_save_due = 0;
    return nav_store_commit_selected();
}

void NavStore_Tick50ms(void)
{
    if (g_nav_store_local_dirty && g_nav_store_save_delay > 0U)
    {
        g_nav_store_save_delay--;
        if (g_nav_store_save_delay == 0U)
        {
            g_nav_store_save_due = 1;
        }
    }
}

uint8 NavStore_Request_Core0_Load(uint8 group, NavGroupIntent_t intent)
{
    if (group >= NAV_GROUP_COUNT || g_nav_load_busy)
    {
        return 0;
    }
    if (!NavStore_Select_For_View(group))
    {
        return 0;
    }
    if (intent == NAV_GROUP_INTENT_EXECUTE && g_nav_store_count < 2U)
    {
        return 0;
    }
    g_nav_load_group = group;
    g_nav_load_intent = (uint8)intent;
    g_nav_load_start = 0;
    g_nav_load_busy = 1;
    g_nav_load_wait_ack = 0;
    return 1;
}

uint8 NavStore_Load_Is_Busy(void)
{
    return g_nav_load_busy;
}

void NavStore_Request_Core0_Flush(void)
{
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    core_b_cmd.nav_force_save_seq++;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
}

uint8 NavStore_Get_Record_State(void)
{
    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    if (core_a_status.nav_record_tx_active)
    {
        return NAV_STORE_SAVING;
    }
    if (core_a_status.nav_record_dirty &&
        core_b_cmd.nav_record_save_result == 2U &&
        core_b_cmd.nav_record_saved_generation == core_a_status.nav_record_generation)
    {
        return NAV_STORE_ERROR;
    }
    if (core_a_status.nav_record_dirty)
    {
        return NAV_STORE_DIRTY;
    }
    return g_nav_group_summary[g_nav_store_selected_group].state;
}

void NavStore_Task(void)
{
    uint16 count;
    uint16 i;

    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));

    if (core_a_status.nav_record_tx_active && core_a_status.nav_record_tx_seq != g_nav_record_tx_seen)
    {
        if (core_a_status.nav_record_tx_group < NAV_GROUP_COUNT &&
            core_a_status.nav_record_tx_count <= NAV_GROUP_CHUNK_POINTS &&
            (uint32)core_a_status.nav_record_tx_start + core_a_status.nav_record_tx_count <= NAV_GROUP_POINT_MAX)
        {
            if (core_a_status.nav_record_tx_start == 0U)
            {
                g_nav_store_selected_group = core_a_status.nav_record_tx_group;
                g_nav_store_count = 0;
                memset(g_nav_store_points, 0, sizeof(g_nav_store_points));
            }
            memcpy(&g_nav_store_points[core_a_status.nav_record_tx_start], core_a_status.nav_record_tx_points,
                   (uint32)core_a_status.nav_record_tx_count * sizeof(IpcNavFlashPoint_t));

            if (core_a_status.nav_record_tx_final)
            {
                g_nav_store_count = core_a_status.nav_record_tx_total;
                g_nav_group_summary[g_nav_store_selected_group].count = g_nav_store_count;
                core_b_cmd.nav_record_save_result = nav_store_commit_selected() ? 1U : 2U;
                core_b_cmd.nav_record_saved_generation = core_a_status.nav_record_tx_generation;
            }
        }
        else
        {
            core_b_cmd.nav_record_save_result = 2U;
        }
        g_nav_record_tx_seen = core_a_status.nav_record_tx_seq;
        core_b_cmd.nav_record_tx_ack_seq = g_nav_record_tx_seen;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }

    if (g_nav_store_local_dirty && g_nav_store_save_due)
    {
        g_nav_store_save_due = 0;
        (void)nav_store_commit_selected();
    }

    if (g_nav_load_busy)
    {
        if (g_nav_load_wait_ack)
        {
            if (core_a_status.nav_load_ack_seq == core_b_cmd.nav_load_seq)
            {
                if (core_b_cmd.nav_load_final)
                {
                    g_nav_load_busy = 0;
                }
                else
                {
                    g_nav_load_start = (uint16)(g_nav_load_start + core_b_cmd.nav_load_count);
                }
                g_nav_load_wait_ack = 0;
            }
        }
        else
        {
            count = (g_nav_store_count > g_nav_load_start) ? (uint16)(g_nav_store_count - g_nav_load_start) : 0U;
            if (count > NAV_GROUP_CHUNK_POINTS)
            {
                count = NAV_GROUP_CHUNK_POINTS;
            }
            core_b_cmd.nav_load_group = g_nav_load_group;
            core_b_cmd.nav_load_intent = g_nav_load_intent;
            core_b_cmd.nav_load_start = g_nav_load_start;
            core_b_cmd.nav_load_count = count;
            core_b_cmd.nav_load_total = g_nav_store_count;
            core_b_cmd.nav_load_final = ((uint32)g_nav_load_start + count >= g_nav_store_count) ? 1U : 0U;
            for (i = 0; i < count; i++)
            {
                core_b_cmd.nav_load_points[i] = g_nav_store_points[g_nav_load_start + i];
            }
            g_nav_load_seq_next++;
            if (g_nav_load_seq_next == 0U)
            {
                g_nav_load_seq_next = 1U;
            }
            core_b_cmd.nav_load_seq = g_nav_load_seq_next;
            SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
            g_nav_load_wait_ack = 1;
        }
    }

    memcpy(core_b_cmd.nav_group_summary, g_nav_group_summary, sizeof(g_nav_group_summary));
    core_b_cmd.nav_selected_group = g_nav_store_selected_group;
    core_b_cmd.nav_store_busy = g_nav_load_busy;
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
}

#else

void NavStore_Init(void) {}
void NavStore_Task(void) {}
void NavStore_Tick50ms(void) {}
const NavGroupSummary_t *NavStore_Get_Summary(uint8 group) { (void)group; return NULL; }
uint8 NavStore_Select_For_View(uint8 group) { (void)group; return 0; }
uint8 NavStore_Get_Selected_Group(void) { return 0; }
uint16 NavStore_Get_Selected_Count(void) { return 0; }
uint8 NavStore_Get_Point(uint16 index, IpcNavFlashPoint_t *point) { (void)index; (void)point; return 0; }
uint8 NavStore_Request_Core0_Load(uint8 group, NavGroupIntent_t intent) { (void)group; (void)intent; return 0; }
uint8 NavStore_Load_Is_Busy(void) { return 0; }
uint8 NavStore_Undo_Selected(void) { return 0; }
uint8 NavStore_Clear_Selected(void) { return 0; }
uint8 NavStore_Flush_Selected(void) { return 0; }
void NavStore_Request_Core0_Flush(void) {}
uint8 NavStore_Get_Record_State(void) { return NAV_STORE_EMPTY; }

#endif

// ================= Core0 navigation-group IPC endpoint =================

static volatile uint32 g_nav_record_generation = 0;
static volatile uint8 g_nav_record_dirty = 0;
static volatile uint16 g_nav_record_save_delay = 0;
#if defined(CY_CORE_CM7_0)
static uint8 g_nav_record_active_group = 0;
#endif

void IPC_Nav_Record_Mark_Dirty(void)
{
    g_nav_record_generation++;
    g_nav_record_dirty = 1;
    g_nav_record_save_delay = NAV_STORE_CORE0_SAVE_TICKS;
}

void IPC_Nav_Group_Core0_Tick10ms(void)
{
    if (g_nav_record_dirty && g_nav_record_save_delay > 0U)
    {
        g_nav_record_save_delay--;
    }
}

#if defined(CY_CORE_CM7_0)

void IPC_Nav_Group_Core0_Task(void)
{
    static uint32 load_seq_seen = 0;
    static uint32 force_seq_seen = 0;
    static uint32 tx_seq = 0;
    static uint32 tx_generation = 0;
    static uint16 tx_start = 0;
    static uint16 tx_total = 0;
    static uint8 tx_wait_ack = 0;
    static uint8 tx_active = 0;
    uint16 count;
    uint16 i;

    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));

    if (core_b_cmd.nav_load_seq != 0U && core_b_cmd.nav_load_seq != load_seq_seen)
    {
        core_a_status.nav_load_result = 0;
        if (core_b_cmd.nav_load_group < NAV_GROUP_COUNT &&
            core_b_cmd.nav_load_count <= NAV_GROUP_CHUNK_POINTS &&
            core_b_cmd.nav_load_total <= NAV_GROUP_POINT_MAX &&
            (uint32)core_b_cmd.nav_load_start + core_b_cmd.nav_load_count <= NAV_GROUP_POINT_MAX)
        {
            if (core_b_cmd.nav_load_start == 0U)
            {
                vofa_mode_driver = 0.0f;
                navi_ctrl.navi_mode_driver = 0;
                memset(record_point_map, 0, sizeof(Navi_WayPoint_t) * NAVI_POINT_MAX);
                record_point_count = 0;
            }
            for (i = 0; i < core_b_cmd.nav_load_count; i++)
            {
                IpcNavFlashPoint_t *src = &core_b_cmd.nav_load_points[i];
                Navi_WayPoint_t *dst = &record_point_map[core_b_cmd.nav_load_start + i];
                dst->x = src->x;
                dst->y = src->y;
                dst->yaw = src->yaw;
                dst->action_cmd = src->action_cmd;
                dst->type = (WayPoint_Type)src->type;
                dst->valid = src->valid;
            }
            if (core_b_cmd.nav_load_final)
            {
                record_point_count = core_b_cmd.nav_load_total;
                g_nav_record_active_group = core_b_cmd.nav_load_group;
                g_nav_record_dirty = 0;
                g_nav_record_save_delay = 0;
                navi_ctrl.origin_set_flag = 0;
                navi_record_update_status();
                vofa_mode_map = 1.0f;
                navi_ctrl.navi_mode_map = 1;
                if (core_b_cmd.nav_load_intent == NAV_GROUP_INTENT_COLLECT)
                {
                    vofa_mode_driver = 2.0f;
                    navi_ctrl.navi_mode_driver = 2;
                }
                else if (core_b_cmd.nav_load_intent == NAV_GROUP_INTENT_EXECUTE)
                {
                    vofa_mode_driver = 1.0f;
                    navi_ctrl.navi_mode_driver = 1;
                }
            }
            core_a_status.nav_load_result = 1;
        }
        else
        {
            core_a_status.nav_load_result = 2;
        }
        load_seq_seen = core_b_cmd.nav_load_seq;
        core_a_status.nav_load_ack_seq = load_seq_seen;
        SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
    }

    if (core_b_cmd.nav_force_save_seq != force_seq_seen)
    {
        force_seq_seen = core_b_cmd.nav_force_save_seq;
        g_nav_record_save_delay = 0;
    }

    if (tx_active && tx_wait_ack && core_b_cmd.nav_record_tx_ack_seq == tx_seq)
    {
        tx_wait_ack = 0;
        if (core_a_status.nav_record_tx_final)
        {
            if (core_b_cmd.nav_record_save_result == 1U &&
                core_b_cmd.nav_record_saved_generation == tx_generation &&
                g_nav_record_generation == tx_generation)
            {
                g_nav_record_dirty = 0;
            }
            else if (g_nav_record_generation == tx_generation)
            {
                g_nav_record_save_delay = NAV_STORE_CORE0_SAVE_TICKS;
            }
            tx_active = 0;
            core_a_status.nav_record_tx_active = 0;
        }
        else
        {
            tx_start = (uint16)(tx_start + core_a_status.nav_record_tx_count);
        }
    }

    if (!tx_active && g_nav_record_dirty && g_nav_record_save_delay == 0U)
    {
        tx_active = 1;
        tx_wait_ack = 0;
        tx_start = 0;
        tx_total = record_point_count;
        tx_generation = g_nav_record_generation;
    }

    if (tx_active && !tx_wait_ack)
    {
        if (tx_start > 0U && tx_generation != g_nav_record_generation)
        {
            tx_active = 0;
            g_nav_record_save_delay = NAV_STORE_CORE0_SAVE_TICKS;
        }
        else
        {
            count = (tx_total > tx_start) ? (uint16)(tx_total - tx_start) : 0U;
            if (count > NAV_GROUP_CHUNK_POINTS)
            {
                count = NAV_GROUP_CHUNK_POINTS;
            }
            for (i = 0; i < count; i++)
            {
                Navi_WayPoint_t *src = &record_point_map[tx_start + i];
                IpcNavFlashPoint_t *dst = &core_a_status.nav_record_tx_points[i];
                dst->x = src->x;
                dst->y = src->y;
                dst->yaw = src->yaw;
                dst->action_cmd = src->action_cmd;
                dst->type = (uint8)src->type;
                dst->valid = src->valid;
            }
            tx_seq++;
            if (tx_seq == 0U)
            {
                tx_seq = 1U;
            }
            core_a_status.nav_record_tx_generation = tx_generation;
            core_a_status.nav_record_tx_seq = tx_seq;
            core_a_status.nav_record_tx_start = tx_start;
            core_a_status.nav_record_tx_count = count;
            core_a_status.nav_record_tx_total = tx_total;
            core_a_status.nav_record_tx_group = g_nav_record_active_group;
            core_a_status.nav_record_tx_final = ((uint32)tx_start + count >= tx_total) ? 1U : 0U;
            core_a_status.nav_record_tx_active = 1;
            tx_wait_ack = 1;
        }
    }

    core_a_status.nav_active_group = g_nav_record_active_group;
    core_a_status.nav_record_generation = g_nav_record_generation;
    core_a_status.nav_record_dirty = g_nav_record_dirty;
    core_a_status.nav_record_tx_active = tx_active;
    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
}

#else

void IPC_Nav_Group_Core0_Task(void) {}

#endif
