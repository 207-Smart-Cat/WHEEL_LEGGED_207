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
                        printf("\r\n >>> Enter WAVE Mode <<< \r\n");
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
                if(cmd_id == 0x01) printf("\r\n [VOFA] KILL SWITCH! \r\n");
                else if(cmd_id == 0x02) printf("\r\n [VOFA] STAY STILL! \r\n");
                i += 5; continue;
            }
        }
        
        // ========================================================
        // 协议 3: VOFA+ 极速 HEX 调参 (AA C2)
        // 修改点：使用掩码机制，实现精准单项更新，不牵一发而动全身
        // ========================================================
        else if((i + 6) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC2)
        {
            uint8 param_id = rx_buffer[i + 2];
            FloatConverter_t temp_float;
            temp_float.b_val[0] = rx_buffer[i + 3];
            temp_float.b_val[1] = rx_buffer[i + 4];
            temp_float.b_val[2] = rx_buffer[i + 5];
            temp_float.b_val[3] = rx_buffer[i + 6];

            __disable_irq(); // 保护共享内存写入过程

            // 根据 param_id 只填充对应的坑位，并标记掩码位
            switch(param_id)
            {
                case 0x01: 
                    core_b_cmd.q_yaw = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 0); 
                    break;
                case 0x02: 
                    core_b_cmd.q_pr = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 1); 
                    break;
                case 0x03: 
                    core_b_cmd.q_bias = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 2); 
                    break;
                case 0x04: 
                    core_b_cmd.r_yaw = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 3); 
                    break;
                case 0x05: 
                    core_b_cmd.r_pr = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 4); 
                    break;
                case 0x06: 
                    core_b_cmd.speed_p = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 5); 
                    break;
                case 0x07: 
                    core_b_cmd.speed_i = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 6); 
                    break;
                case 0x08: 
                    core_b_cmd.speed_d = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 7); 
                    break;
                
                case 0x09: 
                    core_b_cmd.angle_p = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 8); 
                    break;
                case 0x0A: 
                    core_b_cmd.angle_i = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 9); 
                    break;
                case 0x0B: 
                    core_b_cmd.angle_d = temp_float.f_val;
                    core_b_cmd.update_mask |= (1 << 10); 
                    break;
                case 0x0C: 
                    core_b_cmd.gyro_p  = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 11); 
                    break;
                case 0x0D: 
                    core_b_cmd.gyro_i  = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 12); 
                    break;
                case 0x0E: 
                    core_b_cmd.gyro_d  = temp_float.f_val; 
                    core_b_cmd.update_mask |= (1 << 13); 
                    break;
                default: break;
            }

            // 只要有改动，就敲响门铃
            core_b_cmd.param_update_flag = 1; 

            // ?? 关键：写完后立刻同步 D-Cache 到 SRAM 物理地址
            SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));

            __enable_irq(); 
            
            printf("[Core 1] Param %d set to %.4f (Mask: 0x%02X)\n", param_id, temp_float.f_val, core_b_cmd.update_mask);
            
            i += 6; continue;
        }
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
    if(fifo_data_count >= 6) 
    {
        system_delay_ms(2); 
        fifo_data_count = fifo_used(&uart_data_fifo);
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN); 
        VOFA_Protocol_Parse(fifo_get_data, fifo_data_count);
    }
}