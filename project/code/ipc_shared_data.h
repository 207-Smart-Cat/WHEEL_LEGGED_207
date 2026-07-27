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
#define IPC_CORE_B_SHARED_SIZE     (0x400U)
#define IPC_LOG_SLOT_COUNT         (6U)
#define IPC_LOG_TEXT_SIZE          (160U)
#define IPC_NAV_RECORD_PREVIEW_ROWS (4U)
#define NAV_GROUP_COUNT              (10U)
#define NAV_GROUP_POINT_MAX          (500U)
#define NAV_GROUP_CHUNK_POINTS       (16U)

typedef enum {
    NAV_GROUP_INTENT_NONE = 0,
    NAV_GROUP_INTENT_COLLECT = 1,
    NAV_GROUP_INTENT_EXECUTE = 2
} NavGroupIntent_t;

typedef enum {
    NAV_STORE_EMPTY = 0,
    NAV_STORE_SAVED = 1,
    NAV_STORE_DAMAGED = 2,
    NAV_STORE_DIRTY = 3,
    NAV_STORE_SAVING = 4,
    NAV_STORE_ERROR = 5
} NavStoreState_t;

#pragma pack(push, 1)
typedef struct {
    float x;
    float y;
    float yaw;
    uint16 action_cmd;
    uint8 type;
    uint8 valid;
} IpcNavFlashPoint_t;
#pragma pack(pop)

typedef struct {
    uint16 count;
    uint8 state;
    uint8 active_bank;
    uint32 sequence;
} NavGroupSummary_t;

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

    uint32 nav_load_ack_seq;
    uint32 nav_record_generation;
    uint32 nav_record_tx_generation;
    uint32 nav_record_tx_seq;
    uint16 nav_record_tx_start;
    uint16 nav_record_tx_count;
    uint16 nav_record_tx_total;
    uint8 nav_load_result;
    uint8 nav_active_group;
    uint8 nav_record_dirty;
    uint8 nav_record_tx_active;
    uint8 nav_record_tx_group;
    uint8 nav_record_tx_final;
    uint8 nav_record_tx_reserved[2];
    IpcNavFlashPoint_t nav_record_tx_points[NAV_GROUP_CHUNK_POINTS];

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
    uint32 nav_load_seq;
    uint32 nav_record_tx_ack_seq;
    uint32 nav_record_saved_generation;
    uint32 nav_force_save_seq;
    uint16 nav_load_start;
    uint16 nav_load_count;
    uint16 nav_load_total;
    uint8 nav_load_group;
    uint8 nav_load_intent;
    uint8 nav_load_final;
    uint8 nav_record_save_result;
    IpcNavFlashPoint_t nav_load_points[NAV_GROUP_CHUNK_POINTS];
    NavGroupSummary_t nav_group_summary[NAV_GROUP_COUNT];
    uint8 nav_selected_group;
    uint8 nav_store_busy;
    uint8 nav_store_reserved[2];
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
typedef char ipc_nav_flash_point_must_be_16_bytes[(sizeof(IpcNavFlashPoint_t) == 16U) ? 1 : -1];
typedef char ipc_core_a_status_must_fit_before_log[(sizeof(CoreA_Status_t) <= IPC_CORE_A_SHARED_SIZE) ? 1 : -1];
typedef char ipc_log_box_must_fit_before_core_b[(sizeof(IpcLogBox_t) <= IPC_LOG_SHARED_SIZE) ? 1 : -1];
typedef char ipc_core_b_command_must_fit_reserved_block[(sizeof(CoreB_Command_t) <= IPC_CORE_B_SHARED_SIZE) ? 1 : -1];

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
void IPC_Nav_Group_Core0_Task(void);
void IPC_Nav_Group_Core0_Tick10ms(void);
void IPC_Nav_Record_Mark_Dirty(void);
void NavStore_Init(void);
void NavStore_Task(void);
void NavStore_Tick50ms(void);
const NavGroupSummary_t *NavStore_Get_Summary(uint8 group);
uint8 NavStore_Select_For_View(uint8 group);
uint8 NavStore_Get_Selected_Group(void);
uint16 NavStore_Get_Selected_Count(void);
uint8 NavStore_Get_Point(uint16 index, IpcNavFlashPoint_t *point);
uint8 NavStore_Request_Core0_Load(uint8 group, NavGroupIntent_t intent);
uint8 NavStore_Load_Is_Busy(void);
uint8 NavStore_Undo_Selected(void);
uint8 NavStore_Clear_Selected(void);
uint8 NavStore_Flush_Selected(void);
void NavStore_Request_Core0_Flush(void);
uint8 NavStore_Get_Record_State(void);
void IPC_LOG_Printf(const char *format, ...);
void IPC_Flush_Log_To_CoreB(void);
// Flash 参数固化与读取接口
void IPC_Save_Params_To_Flash(void);
void IPC_Load_Params_From_Flash(void);

#endif
