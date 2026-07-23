#ifndef _IPC_SHARED_DATA_H
#define _IPC_SHARED_DATA_H

#include "zf_common_headfile.h"

// IPC shared memory limits. Keep PARAM_COUNT <= 63 because update_mask uses bit 0..62 safely.
#define IPC_PARAM_MAX_COUNT        (63U)
#define IPC_CORE_A_SHARED_ADDR     (0x28001000UL)
#define IPC_LOG_SHARED_ADDR        (0x28001400UL)
#define IPC_CORE_B_SHARED_ADDR     (0x28001800UL)
#define IPC_CORE_A_SHARED_SIZE     (IPC_LOG_SHARED_ADDR - IPC_CORE_A_SHARED_ADDR)
#define IPC_LOG_SHARED_SIZE        (IPC_CORE_B_SHARED_ADDR - IPC_LOG_SHARED_ADDR)
#define IPC_LOG_SLOT_COUNT         (6U)
#define IPC_LOG_TEXT_SIZE          (160U)
#define IPC_NAV_RECORD_PREVIEW_ROWS (4U)

// ==========================================================
// 参数字典枚举
// =========================================================
typedef enum {
#define PARAM_ITEM(id, runtime_var, init_val, display_name) id,
#include "param_registry.def"
#undef PARAM_ITEM
    PARAM_COUNT
} ParamID_e;
// ==========================================================
// Core A (Core 0) 状态结构体
// ==========================================================
typedef struct {
    uint16 idx;
    uint8 valid;
    uint8 type;
    float x;
    float y;
    float yaw;
} IpcNavRecordPreviewPoint_t;

typedef struct {
#define STATUS_ITEM(type, name, source_expr) type name;
#include "status_registry.def"
#undef STATUS_ITEM
    uint16 navi_record_preview_start;
    uint16 navi_record_preview_count;
    IpcNavRecordPreviewPoint_t navi_record_preview[IPC_NAV_RECORD_PREVIEW_ROWS];

    uint32 heartbeat;
    uint8 motor_reason;
    uint8 balance_reason;
    uint8 servo_reason;
    uint8 remote_reason;
    uint8 motor_zero_state;

    // 【优化】用数组统一管理真实的参数！
    float act_params[PARAM_COUNT];
} CoreA_Status_t;

// ==========================================================
// Core B (Core 1) 指令与调参结构体
// ==========================================================
typedef struct {
    float params[PARAM_COUNT];

    uint64_t update_mask;    // 【必须升级为 64 位！】
    uint8 param_update_flag;
    uint8 wifi_connected;
    uint8 vehicle_mode;
    uint8 runtime_status_valid;
    uint8 motor_zero_request;
    uint8 nav_jump_request;
    uint16 nav_record_preview_start;
    uint32 runtime_module_enable_mask;
} CoreB_Command_t;

// ==========================================================
// Core A -> Core B 日志邮箱
// ==========================================================
typedef struct {
    volatile uint32 write_seq;
    volatile uint32 read_seq;
    volatile uint32 dropped_count;
    char text[IPC_LOG_SLOT_COUNT][IPC_LOG_TEXT_SIZE];
} IpcLogBox_t;

typedef char ipc_param_count_must_not_exceed_63[(PARAM_COUNT <= IPC_PARAM_MAX_COUNT) ? 1 : -1];
typedef char ipc_core_a_status_must_fit_before_log[(sizeof(CoreA_Status_t) <= IPC_CORE_A_SHARED_SIZE) ? 1 : -1];
typedef char ipc_log_box_must_fit_before_core_b[(sizeof(IpcLogBox_t) <= IPC_LOG_SHARED_SIZE) ? 1 : -1];

// 绝对地址声明
extern __no_init CoreA_Status_t core_a_status;
extern __no_init CoreB_Command_t core_b_cmd;
extern __no_init IpcLogBox_t ipc_log_box;
extern const char *const g_param_names[PARAM_COUNT];

// 函数声明
void IPC_Init_Shared_Memory(void);
void IPC_Push_Status_From_CoreA(void);
void IPC_Pull_Status_To_CoreB(void);
void IPC_Check_And_Apply_Params_To_Core0(void);
void IPC_Update_Wifi_Status_From_CoreB(uint8 connected);
uint8 IPC_CoreB_Wifi_Is_Connected(void); // Core 0 专用更新函数
uint64_t IPC_Get_All_Param_Mask(void);
void IPC_Request_Param_Update(ParamID_e id, float value);
void IPC_Request_All_Params_Update(void);
void IPC_Request_Motor_Zero_Calibration(void);
void IPC_Request_Nav_Jump(void);
void IPC_Set_Nav_Record_Preview_Start(uint16 start);
uint8 IPC_Consume_Motor_Zero_Request_Core0(void);
uint8 IPC_Consume_Nav_Jump_Request_Core0(void);
void IPC_Update_Motor_Zero_State_From_Core0(uint8 state);
void IPC_LOG_Printf(const char *format, ...);
void IPC_Flush_Log_To_CoreB(void);
// Flash 参数固化与读取接口
void IPC_Save_Params_To_Flash(void);
void IPC_Load_Params_From_Flash(void);

#endif