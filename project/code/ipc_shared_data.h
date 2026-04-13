#ifndef _IPC_SHARED_DATA_H
#define _IPC_SHARED_DATA_H

#include "zf_common_headfile.h"

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
#define STATUS_ITEM(type, name, source_expr) type name;
#include "status_registry.def"
#undef STATUS_ITEM
    uint32 heartbeat;

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
} CoreB_Command_t;
// 绝对地址声明
extern __no_init CoreA_Status_t core_a_status;
extern __no_init CoreB_Command_t core_b_cmd;
extern const char *const g_param_names[PARAM_COUNT];

// 函数声明
void IPC_Init_Shared_Memory(void);
void IPC_Push_Status_From_CoreA(void);
void IPC_Pull_Status_To_CoreB(void);
void IPC_Check_And_Apply_Params_To_Core0(void); // Core 0 专用更新函数


// Flash 参数固化与读取接口
void IPC_Save_Params_To_Flash(void);
void IPC_Load_Params_From_Flash(void);

#endif