#include "zf_common_headfile.h"
#include "ipc_shared_data.h"
#include "imu.h"
#include "control.h"
#include "param.h"
#include "navigation_data_handling.h"
#include "kalman_rm.h"
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
    &target_motor_Stand,    // P_TARGET_MOTOR_STAND (16)
    
    &leg_Kp,               // 17
    &leg_Ki,               // 18
    &leg_Kd,               // 19
    &x_current,            // 20
    &y_current,             // 21
      
    &Air_roll_p,           // 22
    &Air_roll_i,           // 23
    &Air_roll_d,           // 24
    &Direction_p,          // 25
    &Direction_i,          // 26
    &Direction_d,           // 27
      
    &nav_q_x,              // 28
    &nav_q_y,              // 29
    &nav_q_v,              // 30
    &nav_q_bias_ax,        // 31
    &nav_r_v_normal,       // 32
    &nav_r_v_slip,          // 33
      
    &mag_offset_x,         // 34
    &mag_offset_y,         // 35
    &mag_scale_x,          // 36
    &mag_scale_y           // 37
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
    
    core_b_cmd.params[P_AIR_ROLL_P] = 45.0f;
    core_b_cmd.params[P_AIR_ROLL_I] = 0.0f;
    core_b_cmd.params[P_AIR_ROLL_D] = 2.0f;
    
    core_b_cmd.params[P_DIR_P] = 0.0099f;
    core_b_cmd.params[P_DIR_I] = 0.015f;
    core_b_cmd.params[P_DIR_D] = 0.001f;
    
    core_b_cmd.params[P_NAV_Q_X] = 0.001f;
    core_b_cmd.params[P_NAV_Q_Y] = 0.001f;
    core_b_cmd.params[P_NAV_Q_V] = 0.001f;
    core_b_cmd.params[P_NAV_Q_BIAS_AX] = 0.001f;
    core_b_cmd.params[P_NAV_R_V_NORMAL] = 0.001f;
    core_b_cmd.params[P_NAV_R_V_SLIP] = 0.001f;
    
    core_b_cmd.params[P_MAG_OFFSET_X] = -0.080f;
    core_b_cmd.params[P_MAG_OFFSET_Y] = 0.040f;
    core_b_cmd.params[P_MAG_SCALE_X]  = 1.0f;
    core_b_cmd.params[P_MAG_SCALE_Y]  = 1.05f;

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
    
    core_a_status.nav_x = (float)robot_pose.x; 
    core_a_status.nav_y = (float)robot_pose.y;
    core_a_status.nav_v = robot_pose.v;
    core_a_status.nav_w = robot_pose.w;
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
               
        LOG_Printf(" [ Air ]  P: %s%d.%04d, I: %s%d.%04d, D: %s%d.%04d\r\n", 
               F_S(core_b_cmd.params[P_AIR_ROLL_P]), F_I(core_b_cmd.params[P_AIR_ROLL_P]), F_D(core_b_cmd.params[P_AIR_ROLL_P]),
               F_S(core_b_cmd.params[P_AIR_ROLL_I]), F_I(core_b_cmd.params[P_AIR_ROLL_I]), F_D(core_b_cmd.params[P_AIR_ROLL_I]),
               F_S(core_b_cmd.params[P_AIR_ROLL_D]), F_I(core_b_cmd.params[P_AIR_ROLL_D]), F_D(core_b_cmd.params[P_AIR_ROLL_D]));
               
        LOG_Printf(" [ Dir ]  P: %s%d.%04d, I: %s%d.%04d, D: %s%d.%04d\r\n", 
               F_S(core_b_cmd.params[P_DIR_P]), F_I(core_b_cmd.params[P_DIR_P]), F_D(core_b_cmd.params[P_DIR_P]),
               F_S(core_b_cmd.params[P_DIR_I]), F_I(core_b_cmd.params[P_DIR_I]), F_D(core_b_cmd.params[P_DIR_I]),
               F_S(core_b_cmd.params[P_DIR_D]), F_I(core_b_cmd.params[P_DIR_D]), F_D(core_b_cmd.params[P_DIR_D]));
        
        LOG_Printf(" [ Nav ]  qx: %s%d.%05d, qy: %s%d.%05d, qv: %s%d.%05d\r\n", 
               F_S(core_b_cmd.params[P_NAV_Q_X]), F_I(core_b_cmd.params[P_NAV_Q_X]), F_D5(core_b_cmd.params[P_NAV_Q_X]),
               F_S(core_b_cmd.params[P_NAV_Q_Y]), F_I(core_b_cmd.params[P_NAV_Q_Y]), F_D5(core_b_cmd.params[P_NAV_Q_Y]),
               F_S(core_b_cmd.params[P_NAV_Q_V]), F_I(core_b_cmd.params[P_NAV_Q_V]), F_D5(core_b_cmd.params[P_NAV_Q_V]));
               
        LOG_Printf(" [ Nav ]  qba: %s%d.%05d, rvn: %s%d.%05d, rvs: %s%d.%05d\r\n", 
               F_S(core_b_cmd.params[P_NAV_Q_BIAS_AX]), F_I(core_b_cmd.params[P_NAV_Q_BIAS_AX]), F_D5(core_b_cmd.params[P_NAV_Q_BIAS_AX]),
               F_S(core_b_cmd.params[P_NAV_R_V_NORMAL]), F_I(core_b_cmd.params[P_NAV_R_V_NORMAL]), F_D5(core_b_cmd.params[P_NAV_R_V_NORMAL]),
               F_S(core_b_cmd.params[P_NAV_R_V_SLIP]), F_I(core_b_cmd.params[P_NAV_R_V_SLIP]), F_D5(core_b_cmd.params[P_NAV_R_V_SLIP]));
        
        LOG_Printf(" [ Mag ]  off_x: %s%d.%04d, off_y: %s%d.%04d\r\n", 
               F_S(core_b_cmd.params[P_MAG_OFFSET_X]), F_I(core_b_cmd.params[P_MAG_OFFSET_X]), F_D(core_b_cmd.params[P_MAG_OFFSET_X]),
               F_S(core_b_cmd.params[P_MAG_OFFSET_Y]), F_I(core_b_cmd.params[P_MAG_OFFSET_Y]), F_D(core_b_cmd.params[P_MAG_OFFSET_Y]));
               
        LOG_Printf(" [ Mag ]  scl_x: %s%d.%04d, scl_y: %s%d.%04d\r\n", 
               F_S(core_b_cmd.params[P_MAG_SCALE_X]), F_I(core_b_cmd.params[P_MAG_SCALE_X]), F_D(core_b_cmd.params[P_MAG_SCALE_X]),
               F_S(core_b_cmd.params[P_MAG_SCALE_Y]), F_I(core_b_cmd.params[P_MAG_SCALE_Y]), F_D(core_b_cmd.params[P_MAG_SCALE_Y]));
        
        LOG_Printf("=============================================\r\n\r\n");

        // 3. 敲响门铃，让 Core A 一次性全量更新
        core_b_cmd.update_mask = 0xFFFFFFFFFFFFFFFFULL; // 【关键修改：64个1】
        core_b_cmd.param_update_flag = 1;
        SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
    }
    else
    {
        LOG_Printf("\r\n[SYS] Flash is empty or invalid. Using default params.\r\n");
    }
}