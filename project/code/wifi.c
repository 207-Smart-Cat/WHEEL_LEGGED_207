#include "wifi.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h> // 【新增】用于支持 LOG_Printf 的可变参数
#include "small_driver_uart_control.h" 
#include "imu.h"
#include "control.h"
#include "vofa_protocol.h"

// 宏定义和缓冲区保留...
#define WIFI_RX_BUF_SIZE    256         
#define WIFI_MAX_RETRY      5           
uint8 wifi_spi_receive_data[WIFI_RX_BUF_SIZE];          
uint8 wifi_spi_receive_data_ips[WIFI_RX_BUF_SIZE];
static uint32 data_length;                       

// ================= 这些状态变量必须保留在这里！ =================
// 因为 vofa_protocol.c 是通过 extern 借用这几个变量的，它们的“老家”还得在 wifi.c
wifi_mode_t current_wifi_mode = WIFI_MODE_SILENT;         
uint8 wave_format = 1; 
uint8 channel_show[5] = {1, 1, 1, 0, 0}; 
volatile uint8 WIFI_Send_flag = 0;               

uint8 wifi_is_connected = 0; // 【新增】标记 WiFi 是否通关成功 (0:未连接, 1:已连接)

/**
 * @brief WiFi模块初始化并连接 (带超时限制)
 */
void wifi_init(void)
{
    uint8 wifi_spi_test_buffer[] = "Wheel-Leg Robot Online!\r\n";
    uint8 retry_count = 0;
    
    wifi_is_connected = 0; // 初始状态置为 0
    
    // 1. 初始化并连接热点 (加入最大重试次数限制)
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. retry: %d \r\n", retry_count + 1);
        system_delay_ms(100); 
        retry_count++;
        if(retry_count >= WIFI_MAX_RETRY)
        {
            printf("\r\n WiFi Init Timeout! Skip WiFi. \r\n");
            return; // 放弃初始化，直接退出，保证控制代码能运行
        }
    }
    
    printf("\r\n module version:%s", wifi_spi_version);
    printf("\r\n module mac    :%s", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s", wifi_spi_ip_addr_port);

    // 2. 建立TCP/UDP连接 (同样加入重试限制)
    retry_count = 0;
    if(0 == WIFI_SPI_AUTO_CONNECT)
    {
        while(wifi_spi_socket_connect(
            WIFI_PROTOCOL_STR,    // 编译器会自动把它替换成 "TCP" 或 "UDP"
            WIFI_TARGET_IP,       // 编译器会自动把它替换成 特定IP 或 广播IP
            WIFI_TARGET_PORT,
            WIFI_LOCAL_PORT))
        {
            // 打印报错信息也会跟着变
            printf("\r\n Connect %s Servers error, try again.", WIFI_PROTOCOL_STR);
            system_delay_ms(100);
            
            // 【修复】加入重试计数，防止在 TCP 模式下目标未开启导致单片机遇险卡死
            retry_count++;
            if(retry_count >= WIFI_MAX_RETRY)
            {
                printf("\r\n %s Socket Timeout! Skip WiFi Log.\r\n", WIFI_PROTOCOL_STR);
                return; 
            }
        }
    }

    // 3. 发送握手数据
    if(!wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer) - 1)) // 减1是不发送末尾的'\0'
    {
        printf("\r\n wifi init & send success.\r\n");
        wifi_is_connected = 1; // 【新增】三关全部通过，赋予 WiFi 发送双路日志的权限
    }
    
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}

/**
 * @brief WiFi数据处理轮询 (防粘包、多协议解析版 - 现已接入VOFA+)
 */
void wifi_process_loop(void)
{
    // 1. 读取数据：注意这里不需要减 1 预留 '\0' 了，因为我们现在处理的是十六进制二进制流
    data_length = wifi_spi_read_buffer(wifi_spi_receive_data, WIFI_RX_BUF_SIZE);
    
    if(data_length > 0)
    {
      VOFA_Protocol_Parse(wifi_spi_receive_data, data_length);
    }
}

/**
 * @brief WiFi 定时发送任务
 * @note  【已重构】此函数包含大量耗时字符串操作，绝对不能放在中断中！
 * 请在 PIT 中断(20ms)中置位 WIFI_Send_flag = 1，然后在 main 循环中调用此函数。
 */
void wifi_report_task(void)
{
  if(WIFI_Send_flag)
  {
    WIFI_Send_flag = 0;
     switch(current_wifi_mode)
    {
        case WIFI_MODE_WAVE:
            if(wave_format == 0) // ============ HEX 模式 ============
            {
                uint8 idx = 0; // 这个指针负责记录当前装了几个数据
                
                // 【通道 1】：RPY 姿态角 (占用 3 根线)
                if(channel_show[0]) 
                {
                    seekfree_assistant_oscilloscope_data.data[idx++] = IMU_data.filter_result.yaw;
                    seekfree_assistant_oscilloscope_data.data[idx++] = IMU_data.filter_result.pitch;
                    seekfree_assistant_oscilloscope_data.data[idx++] = IMU_data.filter_result.roll;
                }
                
                // 【通道 2】：电机真实转速 (占用 2 根线)
                if(channel_show[1]) 
                {
                    seekfree_assistant_oscilloscope_data.data[idx++] = motor_value.receive_left_speed_data;
                    seekfree_assistant_oscilloscope_data.data[idx++] = motor_value.receive_right_speed_data;
                }
                
                // 【通道 3】：电机底层 PWM (占用 2 根线)
                if(channel_show[2]) 
                {
                    seekfree_assistant_oscilloscope_data.data[idx++] = (float)Motor_Left;
                    seekfree_assistant_oscilloscope_data.data[idx++] = (float)Motor_Right;
                }
                
                // 如果以后有通道 4，直接在下面无脑加 if(channel_show[3]){...} 即可！
                
                seekfree_assistant_oscilloscope_data.channel_num = idx; // 告诉底层一共要发几根线
                
                // 只有当至少有1根线开启时，才发送 HEX 包
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
                char text_buffer[200] = {0}; // 准备一个大车厢
                char temp[64];               // 准备一个小推车
                
                // 【通道 1】：拼装 RPY 姿态角
                if(channel_show[0]) 
                {
                    sprintf(temp, "Y:%.2f P:%.2f R:%.2f  ", IMU_data.filter_result.yaw, IMU_data.filter_result.pitch, IMU_data.filter_result.roll);
                    strcat(text_buffer, temp); // 把小推车挂到大车厢后面
                }
                
                // 【通道 2】：拼装 电机转速
                if(channel_show[1]) 
                {
                    sprintf(temp, "L_Spd:%d R_Spd:%d  ", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
                    strcat(text_buffer, temp);
                }
                
                // 【通道 3】：拼装 电机 PWM
                if(channel_show[2]) 
                {
                    sprintf(temp, "L_PWM:%d R_PWM:%d  ", Motor_Left, Motor_Right);
                    strcat(text_buffer, temp);
                }
                
                // 如果最终大车厢不是空的，加上换行符发出去
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
            // 【模式2：图传请求】
            wifi_spi_send_buffer((uint8*)"IMAGE MODE NOW\r\n", 16);
            break;

        case WIFI_MODE_LOG:
            // 【模式3：日志模式测试】
            wifi_spi_send_buffer((uint8*)"System Running OK...\r\n", 22);
            break;

        default:
            break; // 模式0：静默
    }
  }
}

// ====================================================================
// 【新增】智能双路日志打印函数 (UART + WiFi)
// ====================================================================
void LOG_Printf(const char *format, ...)
{
    char log_buf[256]; 
    
    // 提取可变参数并将其格式化进本地数组
    va_list args;
    va_start(args, format);
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    va_end(args);

    // 1. 物理串口：只要调用就必定发送 (单片机无法感知串口线是否拔出，盲发最安全)
    //printf("%s", log_buf); 

    // 2. WiFi 通道：只有底层握手成功才发送，避免 SPI 阻塞
//    if (wifi_is_connected == 1)
//    {
//        wifi_spi_send_buffer((uint8*)log_buf, strlen(log_buf));
//        
//        #if (WIFI_PROTOCOL_MODE == 1)
//        // 如果用的是 UDP，触发立即推送
//        wifi_spi_udp_send_now(); 
//        #endif
//    }
}