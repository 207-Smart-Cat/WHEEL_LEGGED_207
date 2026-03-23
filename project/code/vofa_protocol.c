#include "vofa_protocol.h"
#include "param.h"
#include "small_driver_uart_control.h"
#include "wifi.h" 
#include "ipc_shared_data.h" // 必须包含 IPC 头文件

// 引入定义在 wifi.c 中的全局状态变量
extern wifi_mode_t current_wifi_mode;
extern uint8 wave_format; 
extern uint8 channel_show[5];

typedef union {
    float f_val;
    uint8 b_val[4];
} FloatConverter_t;



/**
 * @brief 终极通用协议解析器
 */
void VOFA_Protocol_Parse(uint8 *rx_buffer, uint32 data_length)
{
    if(data_length == 0 || rx_buffer == NULL) return;

    for(uint32 i = 0; i < data_length; i++)
    {
        // ========================================================
        // 协议 1: 模式切换 (AA EE) - 保持不变
        // ========================================================
        if((i + 2) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xEE)
        {
            uint8 target_mode = rx_buffer[i+2];
            if(target_mode <= 3) 
            {
                if(target_mode == WIFI_MODE_WAVE)
                {
                    if(current_wifi_mode != WIFI_MODE_WAVE) 
                    {
                        current_wifi_mode = WIFI_MODE_WAVE;
                        LOG_Printf("\r\n >>> Enter WAVE Mode <<< \r\n");
                    }
                    else wave_format ^= 1;
                }
                else current_wifi_mode = (wifi_mode_t)target_mode;
            }
            i += 2; continue; 
        }

        // ========================================================
        // 协议 1.5: 显隐切换 (AA FF) - 保持不变
        // ========================================================
        else if((i + 2) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xFF)
        {
            uint8 ch = rx_buffer[i+2];
            if(ch >= 1 && ch <= 5) channel_show[ch - 1] ^= 1; 
            i += 2; continue;
        }

        // ========================================================
        // 协议 2: 动作控制 (AA C1) - 保持不变
        // ========================================================
        else if((i + 5) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC1)
        {
            uint8 sum_check = 0;
            for(uint8 j = 0; j < 5; j++) sum_check += rx_buffer[i + j]; 
            if(sum_check == rx_buffer[i + 5]) 
            {
                uint8 cmd_id = rx_buffer[i + 2]; 
                if(cmd_id == 0x01) LOG_Printf("\r\n [VOFA] KILL SWITCH! \r\n");
                else if(cmd_id == 0x02) LOG_Printf("\r\n [VOFA] STAY STILL! \r\n");
                i += 5; continue;
            }
        }
        
        // ========================================================
        // 协议 3: VOFA+ 极速 HEX 调参 (AA C2)
        // 修改点：使用掩码机制，实现精准单项更新，不牵一发而动全身
        // ========================================================
        else if((i + 6) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC2)
        {
            uint8 param_id = rx_buffer[i + 2]; // VOFA 下发的是 0x01 到 0x11
            
            // 安全检查：如果 ID 超出了我们现有的参数总数，直接丢弃
            if (param_id == 0 || param_id > PARAM_COUNT) {
                i += 6; continue;
            }

            FloatConverter_t temp_float;
            temp_float.b_val[0] = rx_buffer[i + 3];
            temp_float.b_val[1] = rx_buffer[i + 4];
            temp_float.b_val[2] = rx_buffer[i + 5];
            temp_float.b_val[3] = rx_buffer[i + 6];

            // 把从 1 开始的 ID 转换为从 0 开始的数组下标
            uint8 index = param_id - 1; 

            __disable_irq(); 
            
            // 仅仅两行代码，替代了你原来的 17 个 case！
            core_b_cmd.params[index] = temp_float.f_val; 
            core_b_cmd.update_mask |= (1 << index); 

            core_b_cmd.param_update_flag = 1; 
            SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
            __enable_irq(); 
            
            // 字符串映射表
            static const char* param_names[] = {
                "Q_yaw", "Q_pr", "Q_bias", "R_yaw", "R_pr", 
                "Speed_P", "Speed_I", "Speed_D",            
                "Angle_P", "Angle_I", "Angle_D",            
                "Gyro_P", "Gyro_I", "Gyro_D",               
<<<<<<< HEAD
                "Target_Velocity", "Target_Angle", "Target_Motor_Stand"
=======
                "Target_Velocity", "Target_Angle", "Target_Motor_Stand", // <--- 加上这个逗号！
                "Leg_Kp", "Leg_Ki", "Leg_Kd", "X_Current", "Y_Current"
>>>>>>> p2
            };
            
            LOG_Printf("[Core 1] Param '%s' set to %.4f (Mask: 0x%02X)\r\n", 
                   param_names[index], temp_float.f_val, core_b_cmd.update_mask);
            
            i += 6; continue;
        }
        // ========================================================
        // 协议 4: 系统级指令 - 一键保存参数到 Flash (AA C3)
        // 格式: AA C3 88 55
        // ========================================================
        else if((i + 3) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC3 && 
                rx_buffer[i+2] == 0x88 && rx_buffer[i+3] == 0x55)
        {
            LOG_Printf("\r\n[VOFA] Received SAVE TO FLASH command!\r\n");
            
            SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));
            
<<<<<<< HEAD
            if ( (core_a_status.left_pwm_duty > 1000) || (core_a_status.right_pwm_duty) > 1000) 
=======
            if ( abs(core_a_status.left_pwm_duty) > 1000 || abs(core_a_status.right_pwm_duty) > 1000 )
>>>>>>> p2
            {
                LOG_Printf("[ERROR] Motor is running! PLEASE STOP CAR FIRST!\r\n");
            }
            else
            {
                LOG_Printf("Writing real params from Core A to Flash...\r\n");
                // 【修改这里】直接调用 ipc_shared_data 里的函数
                IPC_Save_Params_To_Flash(); 
                LOG_Printf("Save Complete!\r\n");
            }
            
            i += 3; continue;
        }
<<<<<<< HEAD
=======
        // ========================================================
        // 协议 5: 批量下发所有参数 (AA C4)
        // 格式: AA C4 [22个float (88字节)] [1字节校验和]
        // ========================================================
        else if((i + 2 + PARAM_COUNT * 4 + 1) <= data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC4)
        {
            // 1. 计算校验和 (仅校验 88 字节的数据区)
            uint8 sum_check = 0;
            for(int j = 0; j < PARAM_COUNT * 4; j++) {
                sum_check += rx_buffer[i + 2 + j];
            }
            
            // 2. 校验通过，执行批量更新
            if(sum_check == rx_buffer[i + 2 + PARAM_COUNT * 4])
            {
                LOG_Printf("\r\n[VOFA] Received BULK PARAM UPDATE (AA C4)!\r\n");
                
                __disable_irq(); 
                // 神级操作：利用内存拷贝，一行代码直接把 88 字节的数据灌入 Core B 的参数数组中
                memcpy(core_b_cmd.params, &rx_buffer[i + 2], PARAM_COUNT * sizeof(float));
                
                // 触发全量更新掩码 (0xFFFFFFFF 表示所有位都是 1)
                core_b_cmd.update_mask = 0xFFFFFFFF; 
                core_b_cmd.param_update_flag = 1; 
                
                SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
                __enable_irq(); 
                
                LOG_Printf("[VOFA] All 22 Parameters Updated Successfully!\r\n");
            }
            else
            {
                LOG_Printf("[ERROR] Bulk Update Checksum Failed!\r\n");
            }
            
            i += (2 + PARAM_COUNT * 4 + 1); // 跳过这 91 个字节
            continue;
        }

        // ========================================================
        // 协议 6: 上位机请求读取当前参数 (AA C5)
        // 格式: AA C5 88 55
        // ========================================================
        else if((i + 3) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC5 && 
                rx_buffer[i+2] == 0x88 && rx_buffer[i+3] == 0x55)
        {
            LOG_Printf("\r\n[VOFA] Received PARAM REQUEST (AA C5)! Sending...\r\n");
            
            // 1. 获取最新真值
            IPC_Pull_Status_To_CoreB(); 
            
            // 2. 构建二进制返回包 (AA C4)
            uint8 tx_buf[128]; // 91字节足够放下，开128保底
            tx_buf[0] = 0xAA;
            tx_buf[1] = 0xC4;
            
            // 拷贝 88 字节的 float 数组
            memcpy(&tx_buf[2], core_a_status.act_params, PARAM_COUNT * sizeof(float));
            
            // 计算校验和
            uint8 tx_sum = 0;
            for(int j = 0; j < PARAM_COUNT * 4; j++) {
                tx_sum += tx_buf[2 + j];
            }
            tx_buf[2 + PARAM_COUNT * 4] = tx_sum;
            
            // 发送给电脑（为了以后兼容你自己的上位机，这步保留）
            wifi_spi_send_buffer(tx_buf, 3 + PARAM_COUNT * 4);
            #if (WIFI_PROTOCOL_MODE == 1)
            wifi_spi_udp_send_now(); 
            #endif
            
            // 3. 打印给人类看的清单
            LOG_Printf("\r\n============= CURRENT PARAMS =============\r\n");
            LOG_Printf(" Ang_P: %.4f | Ang_D: %.4f \r\n", core_a_status.act_params[P_ANGLE_P], core_a_status.act_params[P_ANGLE_D]);
            LOG_Printf(" Spd_P: %.4f | Spd_I: %.4f | Spd_D: %.4f\r\n", core_a_status.act_params[P_SPEED_P], core_a_status.act_params[P_SPEED_I], core_a_status.act_params[P_SPEED_D]);
            LOG_Printf(" Gyr_P: %.4f | Gyr_I: %.4f | Gyr_D: %.4f\r\n", core_a_status.act_params[P_GYRO_P], core_a_status.act_params[P_GYRO_I], core_a_status.act_params[P_GYRO_D]);
            LOG_Printf("=============================================\r\n");

            // 4. 【核心新增】打印 VOFA+ 可以直接复制粘贴的 HEX 覆盖指令
            LOG_Printf("\r\n>>> VOFA+ COPY & PASTE COMMAND (AA C4) <<<\r\n");
            
            // 因为 print 太长可能会超出单片机缓存，这里用一个简单的 for 循环挨个打印 HEX
            for(int k = 0; k < (3 + PARAM_COUNT * 4); k++) 
            {
                LOG_Printf("%02X ", tx_buf[k]);
            }
            LOG_Printf("\r\n>>> END OF COMMAND <<<\r\n\r\n");
            
            i += 3; continue;
        }
>>>>>>> p2
    }
}

// ====================================================================
// UART 初始化与后台处理函数（保持你原来的逻辑，无需修改）
// ====================================================================
#define UART_INDEX      (DEBUG_UART_INDEX)
#define UART_BAUDRATE   (115200)
#define UART_TX_PIN     (DEBUG_UART_TX_PIN)
#define UART_RX_PIN     (DEBUG_UART_RX_PIN)

static uint8 uart_get_data[128]; 
static uint8 fifo_get_data[128]; 
fifo_struct uart_data_fifo;

void VOFA_UART_Init(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, sizeof(uart_get_data)); 
    system_delay_ms(15); 
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN); 
    uart_rx_interrupt(UART_INDEX, 1); 
}

void VOFA_UART_Process(void)
{
    uint32 fifo_data_count = fifo_used(&uart_data_fifo); 
    if(fifo_data_count >= 3) 
    {
        system_delay_ms(2); 
        fifo_data_count = fifo_used(&uart_data_fifo);
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN); 
        VOFA_Protocol_Parse(fifo_get_data, fifo_data_count);
    }
}