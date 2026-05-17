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