#include "wifi.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h> // 用于支持 LOG_Printf 的可变参数
#include "small_driver_uart_control.h" 
#include "imu.h"
#include "control.h"
#include "vofa_protocol.h"
#include "ipc_shared_data.h"
#include "navigation_data_handling.h"

// 宏定义和缓冲区
#define WIFI_RX_BUF_SIZE    256         
#define WIFI_MAX_RETRY      5           
uint8 wifi_spi_receive_data[WIFI_RX_BUF_SIZE];          
static uint32 data_length;                       

// ================= 全局状态变量 =================
wifi_mode_t current_wifi_mode = WIFI_MODE_SILENT;         
uint8 wave_format = 1; 
uint8 channel_show[5] = {1, 1, 1, 0, 0}; 
volatile uint8 WIFI_Send_flag = 0;               

uint8 wifi_is_connected = 0; // 标记 WiFi 是否通关成功 (0:未连接, 1:已连接)

// ================= 断线重连与防粘包专用变量 =================
static uint16 wifi_error_count = 0;       // 连续发送失败计数器
static uint16 wifi_reconnect_cooldown = 0;// 重连冷却倒计时

static uint8 wifi_fifo_buffer[512];       // WiFi 专属底层 FIFO 数组
fifo_struct wifi_data_fifo;               // WiFi 专属 FIFO 结构体
static uint8 wifi_parse_buffer[512];      // 拼装完的完整数据包缓冲区

/**
 * @brief WiFi模块初始化并连接 (带超时限制)
 */
void wifi_init(void)
{
    uint8 wifi_spi_test_buffer[] = "Wheel-Leg Robot Online!\r\n";
    uint8 retry_count = 0;
    
    // 初始化 WiFi 的 FIFO (防 VOFA+ 数据碎片化)
    fifo_init(&wifi_data_fifo, FIFO_DATA_8BIT, wifi_fifo_buffer, sizeof(wifi_fifo_buffer));
    
    wifi_is_connected = 0; 
    
    // 1. 初始化并连接热点
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. retry: %d \r\n", retry_count + 1);
        system_delay_ms(100); 
        retry_count++;
        if(retry_count >= WIFI_MAX_RETRY)
        {
            printf("\r\n WiFi Init Timeout! Skip WiFi. \r\n");
            return; 
        }
    }
    
    printf("\r\n module version:%s", wifi_spi_version);
    printf("\r\n module mac    :%s", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s", wifi_spi_ip_addr_port);

    // 2. 建立TCP/UDP连接
    retry_count = 0;
    if(0 == WIFI_SPI_AUTO_CONNECT)
    {
        while(wifi_spi_socket_connect(WIFI_PROTOCOL_STR, WIFI_TARGET_IP, WIFI_TARGET_PORT, WIFI_LOCAL_PORT))
        {
            printf("\r\n Connect %s Servers error, try again.", WIFI_PROTOCOL_STR);
            system_delay_ms(100);
            
            retry_count++;
            if(retry_count >= WIFI_MAX_RETRY)
            {
                printf("\r\n %s Socket Timeout! Skip WiFi Log.\r\n", WIFI_PROTOCOL_STR);
                return; 
            }
        }
    }

    // 3. 发送握手数据
    if(!wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer) - 1)) 
    {
        printf("\r\n wifi init & send success.\r\n");
        wifi_error_count = 0;
        wifi_is_connected = 1; // 三关全部通过，赋予 WiFi 发送权限
    }
    
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}

/**
 * @brief WiFi数据处理轮询 (防粘包拼图版)
 */
void wifi_process_loop(void)
{
    // 1. 读取零碎数据，扔进 FIFO 存起来
    data_length = wifi_spi_read_buffer(wifi_spi_receive_data, WIFI_RX_BUF_SIZE);
    if(data_length > 0)
    {
        fifo_write_buffer(&wifi_data_fifo, wifi_spi_receive_data, data_length);
    }
    
    // 2. 检查 FIFO 里的数据量，看是否凑够了一句话
    uint32 fifo_data_count = fifo_used(&wifi_data_fifo); 
    if(fifo_data_count >= 3) // 至少大于最短指令长度
    {
        // 稍微等 2 毫秒，让后续的碎片数据到达拼成一整帧
        system_delay_ms(2); 
        
        fifo_data_count = fifo_used(&wifi_data_fifo);
        
        // 把拼装好的完整数据一口气提取出来，并清空 FIFO
        fifo_read_buffer(&wifi_data_fifo, wifi_parse_buffer, &fifo_data_count, FIFO_READ_AND_CLEAN); 
        
        // 送去给大统领解析
        VOFA_Protocol_Parse(wifi_parse_buffer, fifo_data_count);
    }
}

/**
 * @brief WiFi 自动断线重连状态机 (非阻塞式冷却设计)
 * @note  需放在 main 函数的 while(1) 循环中持续调用
 */
void wifi_auto_reconnect_task(void)
{
    if (wifi_is_connected == 1) return;

    if (wifi_reconnect_cooldown > 0)
    {
        wifi_reconnect_cooldown--;
        return;
    }

    printf("\r\n[WIFI] Attempting Auto-Reconnect...\r\n");

    // 尝试重连热点和端口
    if (wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST) == 0)
    {
        if (wifi_spi_socket_connect(WIFI_PROTOCOL_STR, WIFI_TARGET_IP, WIFI_TARGET_PORT, WIFI_LOCAL_PORT) == 0)
        {
            printf("\r\n[WIFI] Auto-Reconnect Success!!!\r\n");
            wifi_error_count = 0;
            wifi_is_connected = 1; // 恢复连接，复活！
            
            // ========================================================
            // 【新增体验优化】重连成功后，主动把当前 RAM 里的真实参数推给电脑！
            // 坚决不读 Flash，而是把刚调好的参数同步给 VOFA+
            // ========================================================
            IPC_Pull_Status_To_CoreB(); 
            uint8 tx_buf[200]; // 【修复：扩容到 200 字节防止溢出死机！】
            tx_buf[0] = 0xAA;
            tx_buf[1] = 0xC4;
            // 拷贝 float 数组
            memcpy(&tx_buf[2], core_a_status.act_params, PARAM_COUNT * sizeof(float));
            
            // 计算校验和
            uint8 tx_sum = 0;
            for(int j = 0; j < PARAM_COUNT * 4; j++) {
                tx_sum += tx_buf[2 + j];
            }
            tx_buf[2 + PARAM_COUNT * 4] = tx_sum;
            
            // 发送给电脑端同步 UI
            wifi_spi_send_buffer(tx_buf, 3 + PARAM_COUNT * 4);
            #if (WIFI_PROTOCOL_MODE == 1)
            wifi_spi_udp_send_now(); 
            #endif
            printf("[WIFI] Current RAM Params Synced to PC!\r\n");
            // ========================================================

            return; 
        }
    }

    // 重连失败，进入长冷却期 (大约半秒到1秒，取决于你的主循环速度)
    printf("\r\n[WIFI] Reconnect Failed. Cooldown for next attempt...\r\n");
    wifi_reconnect_cooldown = 500; 
}

/**
 * @brief WiFi 定时波形发送任务
 */
void wifi_report_task(void)
{
    if(WIFI_Send_flag)
    {
        WIFI_Send_flag = 0;
        
        // --- 掉线保护：如果断线了，不要尝试发波形，浪费时间 ---
        if(wifi_is_connected == 0) return;

        switch(current_wifi_mode)
        {
            case WIFI_MODE_WAVE:
                if(wave_format == 0) // ============ HEX 模式 ============
                {
                    uint8 idx = 0; 
                    
                    if(channel_show[0]) 
                    {
                        seekfree_assistant_oscilloscope_data.data[idx++] = IMU_data.filter_result.yaw;
                        seekfree_assistant_oscilloscope_data.data[idx++] = IMU_data.filter_result.pitch;
                        seekfree_assistant_oscilloscope_data.data[idx++] = IMU_data.filter_result.roll;
                    }
                    if(channel_show[1]) 
                    {
                        seekfree_assistant_oscilloscope_data.data[idx++] = motor_value.receive_left_speed_data;
                        seekfree_assistant_oscilloscope_data.data[idx++] = motor_value.receive_right_speed_data;
                    }
                    if(channel_show[2]) 
                    {
                        seekfree_assistant_oscilloscope_data.data[idx++] = (float)Motor_Left;
                        seekfree_assistant_oscilloscope_data.data[idx++] = (float)Motor_Right;
                    }
                    
                    if(channel_show[3]) 
                    {
                        // 【修复：通过 IPC 状态机安全读取导航数据】
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.nav_x;
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.nav_y;
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.yaw; // 用姿态融合的Yaw
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.nav_v;
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.nav_w;
                    }

                    if(channel_show[4]) 
                    {
                        // 【新增：通过 IPC 状态机安全读取串级 PID 输出】
                        // 你可以根据需要随时在这里增删你想看的变量
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.pid_out_speed_l;
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.pid_out_angle_l;
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.pid_out_gyro_l;
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.pid_out_turn;
                        seekfree_assistant_oscilloscope_data.data[idx++] = core_a_status.pid_out_leg;
                    }

                    seekfree_assistant_oscilloscope_data.channel_num = idx; 
                    if(idx > 0) 
                    {
                        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
                        #if (WIFI_PROTOCOL_MODE == 1)
                        wifi_spi_udp_send_now(); 
                        #endif
                    }
                }
                else // ============ 文本 (TEXT) 模式 ============
                {
                    char text_buffer[200] = {0}; 
                    char temp[64];               
                    
                    if(channel_show[0]) 
                    {
                        sprintf(temp, "Y:%.2f P:%.2f R:%.2f  ", IMU_data.filter_result.yaw, IMU_data.filter_result.pitch, IMU_data.filter_result.roll);
                        strcat(text_buffer, temp); 
                    }
                    // ... channel 1 和 2 保持你的原样 ...
                    if(channel_show[1]) 
                    {
                        sprintf(temp, "L_Spd:%d R_Spd:%d  ", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
                        strcat(text_buffer, temp);
                    }
                    if(channel_show[2]) 
                    {
                        sprintf(temp, "L_PWM:%d R_PWM:%d  ", Motor_Left, Motor_Right);
                        strcat(text_buffer, temp);
                    }

                    if(channel_show[3])
                    {
                        // 【修复：安全读取导航文本】
                        sprintf(temp, "x:%.3f y:%.3f yaw:%.2f v:%.3f w:%.3f ", 
                                core_a_status.nav_x, core_a_status.nav_y, core_a_status.yaw, 
                                core_a_status.nav_v, core_a_status.nav_w);
                        strcat(text_buffer, temp);
                    }

                    if(channel_show[4])
                    {
                        // 【新增：安全读取 PID 文本】
                        sprintf(temp, "SpdL:%.1f AngL:%.1f GyrL:%.1f Turn:%.1f ", 
                                core_a_status.pid_out_speed_l, core_a_status.pid_out_angle_l, 
                                core_a_status.pid_out_gyro_l, core_a_status.pid_out_turn);
                        strcat(text_buffer, temp);
                    }
                    
                    if(strlen(text_buffer) > 0) 
                    {
                        strcat(text_buffer, "\r\n");
                        wifi_spi_send_buffer((uint8*)text_buffer, strlen(text_buffer));
                        #if (WIFI_PROTOCOL_MODE == 1)
                        wifi_spi_udp_send_now(); 
                        #endif
                    }
                }
                break;

            case WIFI_MODE_IMAGE:
                wifi_spi_send_buffer((uint8*)"IMAGE MODE NOW\r\n", 16);
                break;
            case WIFI_MODE_LOG:
                wifi_spi_send_buffer((uint8*)"System Running OK...\r\n", 22);
                break;
            default:
                break; 
        }
    }
}

// ====================================================================
// 智能双路日志打印函数 (UART + WiFi) - 附带掉线嗅探功能
// ====================================================================
void LOG_Printf(const char *format, ...)
{
    char log_buf[256]; 
    
    va_list args;
    va_start(args, format);
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    va_end(args);

    // 1. 物理串口：始终盲发
    printf("%s", log_buf); 

    // 2. WiFi 通道与掉线检测
    if (wifi_is_connected == 1)
    {
        // 如果底层发送函数返回非 0，说明发生网络/SPI错误
        if (wifi_spi_send_buffer((uint8*)log_buf, strlen(log_buf)) != 0)
        {
            wifi_error_count++;
            if (wifi_error_count > 10) // 连续失败 10 次，判定为彻底断开
            {
                wifi_is_connected = 0;         // 拔掉发送权限，阻止主循环卡死
                wifi_reconnect_cooldown = 100; // 设定一个初始冷却期
                printf("\r\n[SYS] WiFi Connection Lost! Entering Auto-Reconnect Mode...\r\n");
            }
        }
        else
        {
            wifi_error_count = 0; // 发送成功则清零计数器
        }
        
        #if (WIFI_PROTOCOL_MODE == 1)
        wifi_spi_udp_send_now(); 
        #endif
    }
}