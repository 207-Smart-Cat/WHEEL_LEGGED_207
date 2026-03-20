#include "ipc_shared_data.h"
#include "imu.h"
#include "control.h"
#include "param.h"
// --- 1. 绝对地址内存分配 ---
#pragma location = 0x28001000
__no_init CoreA_Status_t core_a_status; 

#pragma location = 0x28001200
__no_init CoreB_Command_t core_b_cmd; 

// --- 2. 初始化函数 ---
#include <string.h>  // 必须包含这个头文件才能使用 memset

void IPC_Init_Shared_Memory(void) {
    memset(&core_a_status, 0, sizeof(CoreA_Status_t));
    memset(&core_b_cmd, 0, sizeof(CoreB_Command_t));
    
    // 赋予安全的初始值，防止 Core 0 启动时读到 0 崩溃
    core_b_cmd.q_yaw = 0.001f;
    core_b_cmd.q_pr  = 0.003f;
    core_b_cmd.q_bias = 0.001f;
    core_b_cmd.r_yaw = 0.05f;
    core_b_cmd.r_pr  = 0.05f;
    core_b_cmd.update_mask = 0;
    core_b_cmd.param_update_flag = 0;
    
    core_b_cmd.speed_p = 0.06f; core_b_cmd.speed_i = 0.0f; core_b_cmd.speed_d = 0.022f;
    core_b_cmd.angle_p = 6.0f;  core_b_cmd.angle_i = 0.0f; core_b_cmd.angle_d = 0.4f;
    core_b_cmd.gyro_p  = 5.0f;  core_b_cmd.gyro_i  = 0.5f; core_b_cmd.gyro_d  = 0.27f;

    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
}

// --- 3. Core A 专属：将全局变量打包进共享内存，并刷入 SRAM ---
void IPC_Push_Status_From_CoreA(void) {
    core_a_status.roll  = IMU_data.filter_result.roll;
    core_a_status.pitch = IMU_data.filter_result.pitch;
    core_a_status.yaw   = IMU_data.filter_result.yaw;
    core_a_status.left_wheel_speed =  motor_value.receive_left_speed_data;
    core_a_status.right_wheel_speed = motor_value.receive_right_speed_data;
    core_a_status.left_pwm_duty = Motor_Left;
    core_a_status.right_pwm_duty = Motor_Right;
    
    // 更新时间戳防卡死
    core_a_status.heartbeat++;
    
    // 【关键步骤】将写好的 Cache 刷进真正的 SRAM 中
    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
}

// --- 4. Core B 专属：从 SRAM 拉取最新数据到结构体 ---
void IPC_Pull_Status_To_CoreB(void) {
    // 【关键步骤】让 Core B 的 Cache 失效，强制从真正的 SRAM 读取最新数据
    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
    
    // 读取完后，Core B 就可以直接在程序里使用 core_a_status.yaw 等变量了
}


/**
 * @brief Core A (Core 0) 专属：从 SRAM 拉取 Core B 发来的调参指令并应用
 * @note  建议放在 Core 0 的 5ms 定时器中断最开始执行
 */
// Core 0 定时器调用的精准更新函数
void IPC_Check_And_Apply_Params_To_Core0(void) {

    // 1. 强制从 SRAM 读取最新数据
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    
    if(core_b_cmd.param_update_flag == 1) {
        // 2. 检查掩码，精准打击
        if(core_b_cmd.update_mask & (1 << 0)) filter.Qyaw        = core_b_cmd.q_yaw;
        if(core_b_cmd.update_mask & (1 << 1)) filter.Qpitch_roll = core_b_cmd.q_pr;
        if(core_b_cmd.update_mask & (1 << 2)) filter.Qgyrobias   = core_b_cmd.q_bias;
        if(core_b_cmd.update_mask & (1 << 3)) filter.Ryaw        = core_b_cmd.r_yaw;
        if(core_b_cmd.update_mask & (1 << 4)) filter.Rpitch_roll = core_b_cmd.r_pr;
        
        if(core_b_cmd.update_mask & (1 << 5))  Speed_p = core_b_cmd.speed_p;
        if(core_b_cmd.update_mask & (1 << 6))  Speed_i = core_b_cmd.speed_i;
        if(core_b_cmd.update_mask & (1 << 7))  Speed_d = core_b_cmd.speed_d;
        
        if(core_b_cmd.update_mask & (1 << 8))  Angle_p = core_b_cmd.angle_p;
        if(core_b_cmd.update_mask & (1 << 9))  Angle_i = core_b_cmd.angle_i;
        if(core_b_cmd.update_mask & (1 << 10)) Angle_d = core_b_cmd.angle_d;
        
        if(core_b_cmd.update_mask & (1 << 11)) Gyro_p  = core_b_cmd.gyro_p;
        if(core_b_cmd.update_mask & (1 << 12)) Gyro_i  = core_b_cmd.gyro_i;
        if(core_b_cmd.update_mask & (1 << 13)) Gyro_d  = core_b_cmd.gyro_d;

        // 3. 清除标记并写回 SRAM
        core_b_cmd.update_mask = 0;
        core_b_cmd.param_update_flag = 0;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }
}