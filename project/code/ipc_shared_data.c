#include "zf_common_headfile.h"
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

// 【核心映射表】把底层全局变量的地址，按枚举顺序放进这个指针数组里
float* const param_map[PARAM_COUNT] = {
    &filter.Qyaw,          // P_Q_YAW (0)
    &filter.Qpitch_roll,   // P_Q_PR  (1)
    &filter.Qgyrobias,
    &filter.Ryaw,
    &filter.Rpitch_roll,
    &Speed_p,
    &Speed_i,
    &Speed_d,
    &Angle_p,
    &Angle_i,
    &Angle_d,
    &Gyro_p,
    &Gyro_i,
    &Gyro_d,
    &target_velocity,
    &target_angle,
    &target_motor_Stand    // P_TARGET_MOTOR_STAND (16)
};

// --- 初始化函数 ---
void IPC_Init_Shared_Memory(void) {
    memset(&core_a_status, 0, sizeof(CoreA_Status_t));
    memset(&core_b_cmd, 0, sizeof(CoreB_Command_t));
    
    // 【修改点】使用枚举下标给数组赋初值
    core_b_cmd.params[P_Q_YAW]  = 0.001f;
    core_b_cmd.params[P_Q_PR]   = 0.003f;
    core_b_cmd.params[P_Q_BIAS] = 0.001f;
    core_b_cmd.params[P_R_YAW]  = 0.05f;
    core_b_cmd.params[P_R_PR]   = 0.05f;
    
    core_b_cmd.params[P_SPEED_P] = 0.06f; 
    core_b_cmd.params[P_SPEED_I] = 0.0f; 
    core_b_cmd.params[P_SPEED_D] = 0.022f;
    
    core_b_cmd.params[P_ANGLE_P] = 6.0f;  
    core_b_cmd.params[P_ANGLE_I] = 0.0f; 
    core_b_cmd.params[P_ANGLE_D] = 0.4f;
    
    core_b_cmd.params[P_GYRO_P]  = 5.0f;  
    core_b_cmd.params[P_GYRO_I]  = 0.5f; 
    core_b_cmd.params[P_GYRO_D]  = 0.27f;
    
    core_b_cmd.params[P_TARGET_VELOCITY]    = 0.0f;
    core_b_cmd.params[P_TARGET_ANGLE]       = 0.0f;
    core_b_cmd.params[P_TARGET_MOTOR_STAND] = 0.0f;

    core_b_cmd.update_mask = 0;
    core_b_cmd.param_update_flag = 0;

    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
}

// --- 3. Core A 专属：将全局变量打包进共享内存，并刷入 SRAM ---
void IPC_Push_Status_From_CoreA(void) {
    // 1. 打包高频运动状态
    core_a_status.roll  = IMU_data.filter_result.roll;
    core_a_status.pitch = IMU_data.filter_result.pitch;
    core_a_status.yaw   = IMU_data.filter_result.yaw;
    core_a_status.left_wheel_speed =  motor_value.receive_left_speed_data;
    core_a_status.right_wheel_speed = motor_value.receive_right_speed_data;
    core_a_status.left_pwm_duty = Motor_Left;
    core_a_status.right_pwm_duty = Motor_Right;
    
    // 2. 【核心新增】打包当前真正在使用的底层参数 (Ground Truth)
    for(int i = 0; i < PARAM_COUNT; i++) {
            core_a_status.act_params[i] = *(param_map[i]);
        }

    // 3. 更新时间戳防卡死
    core_a_status.heartbeat++;
    
    // 4. 将写好的 Cache 刷进真正的 SRAM 中
    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
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
void IPC_Check_And_Apply_Params_To_Core0(void) {
    SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    
    if(core_b_cmd.param_update_flag == 1) {
        for(int i = 0; i < PARAM_COUNT; i++) {
            // 检查对应位是否被置 1
            if(core_b_cmd.update_mask & (1 << i)) {
                *(param_map[i]) = core_b_cmd.params[i]; // 直接把值写到底层变量内存里！
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
#define PARAM_DATA_LENGTH    (19)   // 保存 19 个数据（含头尾）

// ====================================================================
// 功能 1：真正操作硬件，把参数写入 Flash (Core B 调用)
// ====================================================================
void IPC_Save_Params_To_Flash(void) {
    SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
    __disable_irq();
    flash_buffer_clear();

    flash_union_buffer[0].uint32_type = 0x55AA55AA;  

    // 一个循环搞定所有参数的写入
    for(int i = 0; i < PARAM_COUNT; i++) {
        flash_union_buffer[i + 1].float_type = core_a_status.act_params[i];
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
    flash_read_page_to_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, PARAM_COUNT + 2);

    if(flash_union_buffer[0].uint32_type == 0x55AA55AA && 
       flash_union_buffer[PARAM_COUNT + 1].uint32_type == 0x11223344)
    {
        LOG_Printf("\r\n[SYS] Valid parameters found in Flash! Loading to Core A...\r\n");
        
        // 1. 极致精简：用循环把数据从 Flash 全部倒进结构体的数组里
        for(int i = 0; i < PARAM_COUNT; i++) {
            core_b_cmd.params[i] = flash_union_buffer[i + 1].float_type;
        }

        // 2. 经典排版：用枚举作为下标，恢复你最习惯的分组显示格式
        LOG_Printf("\r\n============= FLASH LOAD RESULT =============\r\n");
        LOG_Printf(" [Filter] Q_yaw: %.4f, Q_pr: %.4f, Q_bias: %.4f\r\n", 
               core_b_cmd.params[P_Q_YAW], core_b_cmd.params[P_Q_PR], core_b_cmd.params[P_Q_BIAS]);
        LOG_Printf(" [Filter] R_yaw: %.4f, R_pr: %.4f\r\n", 
               core_b_cmd.params[P_R_YAW], core_b_cmd.params[P_R_PR]);
        LOG_Printf("---------------------------------------------\r\n");
        LOG_Printf(" [Speed]  P: %.4f, I: %.4f, D: %.4f\r\n", 
               core_b_cmd.params[P_SPEED_P], core_b_cmd.params[P_SPEED_I], core_b_cmd.params[P_SPEED_D]);
        LOG_Printf(" [Angle]  P: %.4f, I: %.4f, D: %.4f\r\n", 
               core_b_cmd.params[P_ANGLE_P], core_b_cmd.params[P_ANGLE_I], core_b_cmd.params[P_ANGLE_D]);
        LOG_Printf(" [Gyro]   P: %.4f, I: %.4f, D: %.4f\r\n", 
               core_b_cmd.params[P_GYRO_P], core_b_cmd.params[P_GYRO_I], core_b_cmd.params[P_GYRO_D]);
        LOG_Printf("---------------------------------------------\r\n");
        LOG_Printf(" [Target] Vel: %.4f, Ang: %.4f, Stand: %.4f\r\n", 
               core_b_cmd.params[P_TARGET_VELOCITY], core_b_cmd.params[P_TARGET_ANGLE], core_b_cmd.params[P_TARGET_MOTOR_STAND]);
        LOG_Printf("---------------------------------------------\r\n");
        LOG_Printf(" [ Leg ]  Kp: %.4f, Ki: %.4f, Kd: %.4f\r\n", 
               core_b_cmd.params[P_LEG_KP], core_b_cmd.params[P_LEG_KI], core_b_cmd.params[P_LEG_KD]);
        LOG_Printf(" [ Pos ]  X: %.4f, Y: %.4f\r\n", 
               core_b_cmd.params[P_X_CURRENT], core_b_cmd.params[P_Y_CURRENT]);
        LOG_Printf("=============================================\r\n\r\n");

        // 3. 敲响门铃，让 Core A 一次性全量更新
        core_b_cmd.update_mask = 0xFFFFFFFF; 
        core_b_cmd.param_update_flag = 1;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }
    else
    {
        LOG_Printf("\r\n[SYS] Flash is empty or invalid. Using default params.\r\n");
    }
}