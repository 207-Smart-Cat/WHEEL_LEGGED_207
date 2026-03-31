#ifndef _IPC_SHARED_DATA_H
#define _IPC_SHARED_DATA_H

#include "zf_common_headfile.h"

// ==========================================================
// 参数字典枚举
// =========================================================
typedef enum {
    P_Q_YAW = 0,
    P_Q_PR,
    P_Q_BIAS,
    P_R_YAW,
    P_R_PR,
    P_SPEED_P,
    P_SPEED_I,
    P_SPEED_D,
    P_ANGLE_P,
    P_ANGLE_I,
    P_ANGLE_D,
    P_GYRO_P,
    P_GYRO_I,
    P_GYRO_D,
    P_TARGET_VELOCITY,
    P_TARGET_ANGLE,
    P_TARGET_MOTOR_STAND,
    
    P_LEG_KP,
    P_LEG_KI,
    P_LEG_KD,
    P_X_CURRENT,
    P_Y_CURRENT,
    
    P_AIR_ROLL_P,
    P_AIR_ROLL_I,
    P_AIR_ROLL_D,
    P_DIR_P,
    P_DIR_I,
    P_DIR_D,
    
    P_NAV_Q_V,
    P_NAV_Q_W,
    P_NAV_Q_BIAS_AX,
    P_NAV_Q_BIAS_W,
    P_NAV_R_V_NORMAL,
    P_NAV_R_V_SLIP,
    P_NAV_R_W_NORMAL,
    P_NAV_R_W_SLIP,
    P_NAV_R_GYRO,
    
    // 你的磁力计参数必须排在最后面！
    P_MAG_OFFSET_X,
    P_MAG_OFFSET_Y,
    P_MAG_SCALE_X,
    P_MAG_SCALE_Y,
    
    PARAM_COUNT // 这个枚举的终极妙用：它会自动等于参数的总个数！
} ParamID_e;
// ==========================================================
// Core A (Core 0) 状态结构体
// ==========================================================
typedef struct {
    float roll, pitch, yaw;
    int16 left_wheel_speed, right_wheel_speed;
    int16 left_pwm_duty, right_pwm_duty;
    
    float nav_x,nav_y,nav_v,nav_w;
    uint32 heartbeat;
    
    float pid_out_speed_l, pid_out_speed_r;
    float pid_out_angle_l, pid_out_angle_r;
    float pid_out_gyro_l,  pid_out_gyro_r;
    float pid_out_turn;
    float pid_out_leg;
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

// 函数声明
void IPC_Init_Shared_Memory(void);
void IPC_Push_Status_From_CoreA(void);
void IPC_Pull_Status_To_CoreB(void);
void IPC_Check_And_Apply_Params_To_Core0(void); // Core 0 专用更新函数


// Flash 参数固化与读取接口
void IPC_Save_Params_To_Flash(void);
void IPC_Load_Params_From_Flash(void);

#endif