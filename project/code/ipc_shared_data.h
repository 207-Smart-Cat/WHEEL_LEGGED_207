#ifndef _IPC_SHARED_DATA_H
#define _IPC_SHARED_DATA_H

#include "zf_common_headfile.h"

// Core A (Core 0) 状态结构体
typedef struct {
    float roll, pitch, yaw;
    float left_wheel_speed, right_wheel_speed;
    int16 left_pwm_duty, right_pwm_duty;
    uint32 heartbeat;
} CoreA_Status_t;

// Core B (Core 1) 指令与调参结构体
typedef struct {
    // 调参数据镜像区
    float q_yaw;   // ID 1
    float q_pr;    // ID 2
    float q_bias;  // ID 3
    float r_yaw;   // ID 4
    float r_pr;    // ID 5
    float speed_p; //ID 6
    float speed_i; //ID 7
    float speed_d; //ID 8
    float angle_p; //ID 9
    float angle_i; //ID 10
    float angle_d; //ID 11
    float gyro_p;  //ID 12
    float gyro_i;  //ID 13 
    float gyro_d;  //ID 14
    
    // 状态控制
    uint8 update_mask;       // 掩码位：bit0~bit4 对应上述 5 个参数
    uint8 param_update_flag; // 总门铃：1-有更新，0-已处理
    
    // 其他指令
    float target_speed;
    float target_yaw;
} CoreB_Command_t;

// 绝对地址声明
extern __no_init CoreA_Status_t core_a_status;
extern __no_init CoreB_Command_t core_b_cmd;

// 函数声明
void IPC_Init_Shared_Memory(void);
void IPC_Push_Status_From_CoreA(void);
void IPC_Pull_Status_To_CoreB(void);
void IPC_Check_And_Apply_Params_To_Core0(void); // Core 0 专用更新函数

#endif