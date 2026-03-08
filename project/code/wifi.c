#include "wifi.h"
#include <string.h>
#include <stdio.h>
#include "small_driver_uart_control.h" // 确保包含了无刷电机的头文件
#include "imu.h"

// 宏定义：消除魔法数字，方便统一管理
#define WIFI_RX_BUF_SIZE    256         //WiFi接收数组最大容量
#define WIFI_MAX_RETRY      5           // 最大重试次数

// 定义内部使用的缓冲区
uint8 wifi_spi_receive_data[WIFI_RX_BUF_SIZE];          
uint8 wifi_spi_receive_data_ips[WIFI_RX_BUF_SIZE];
static uint32 data_length;                       //数据长度变量
wifi_mode_t current_wifi_mode = WIFI_MODE_SILENT;         //当前WiFi模式变量,默认静默模式，也就是WiFi不发送任何东西

// ================= 新增：交互状态记忆变量 =================
// 0: HEX 波形模式, 1: 纯文本模式。初始化默认为文本模式
uint8 wave_format = 1; 

// 5个通道的显示状态 (1: 显示, 0: 隐藏)，初始状态全显示。对应: Yaw, Pitch, Roll, Left, Right
uint8 channel_show[5] = {1, 1, 1, 1, 1};

//static uint8 test_dummy_image[100] = {0xFF}; //测试图像
//**下面这几行用作摄像头图传，目前不使用
//    // --- 假设这是你用于存放备份图像的数组 (参考官方例程) ---
//    extern uint8 image_copy[MT9V03X_H][MT9V03X_W]; 
//    extern uint8 mt9v03x_finish_flag; // 摄像头的采集完成标志位
//
//    // 专门给主循环用的图传触发标志位
//    volatile uint8 trigger_image_send_in_main = 0;
/**
 * @brief WiFi模块初始化并连接 (带超时限制)
 */
void wifi_init(void)
{
    uint8 wifi_spi_test_buffer[] = "Wheel-Leg Robot Online!";
    uint8 retry_count = 0;
    
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

    // 2. 建立TCP连接 (同样加入重试限制)
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
        }
    }

    // 3. 发送握手数据
    if(!wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer) - 1)) // 减1是不发送末尾的'\0'
    {
        printf("\r\n wifi init & send success.");
    }
    
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}


/**
 * @brief WiFi数据处理轮询 (防粘包、多协议解析版)
 */
void wifi_process_loop(void)
{
    // 1. 读取数据：注意这里不需要减 1 预留 '\0' 了，因为我们现在处理的是十六进制二进制流
    data_length = wifi_spi_read_buffer(wifi_spi_receive_data, WIFI_RX_BUF_SIZE);
    
    if(data_length > 0)
    {
        // 2. 使用 i 作为滑动指针，遍历整个接收到的缓冲区，寻找所有合法的指令包
        for(uint32 i = 0; i < data_length; i++)
        {
            // ========================================================
            // 协议 1: 模式切换 & 格式翻转 二合一指令 (帧头: 0xAA 0xEE)
            // ========================================================
            if((i + 2) < data_length && wifi_spi_receive_data[i] == 0xAA && wifi_spi_receive_data[i+1] == 0xEE)
            {
                uint8 target_mode = wifi_spi_receive_data[i+2];
                if(target_mode <= 3) 
                {
                    // 如果用户发送的是 AA EE 01 (目标是波形模式)
                    if(target_mode == WIFI_MODE_WAVE)
                    {
                        if(current_wifi_mode != WIFI_MODE_WAVE) 
                        {
                            // 情况A：从其他模式首次切入波形模式 -> 保留上次的文本格式
                            current_wifi_mode = WIFI_MODE_WAVE;
                            printf("\r\n >>> Enter WAVE Mode, Format: %s <<< \r\n", wave_format ? "TEXT" : "HEX"););
                        }
                        else 
                        {
                            // 情况B：当前已经在波形模式了，再次收到 AA EE 01 -> 触发异或翻转格式 (1变0，0变1)
                            wave_format ^= 1; 
                            printf("\r\n >>> Format Toggled to: %s <<< \r\n", wave_format ? "TEXT" : "HEX");
                        }
                    }
                    else
                    {
                        // 如果用户发送的是 AA EE 00 / 02 / 03 等其他模式，正常切换即可
                        current_wifi_mode = (wifi_mode_t)target_mode;
                        printf("\r\n >>> WiFi Mode Switched to: %d <<< \r\n", target_mode);
                    }
                }
                i += 2; 
                continue; 
            }

            // ========================================================
            // 协议 1.5: 通道独立显隐切换指令 (AA FF 01 ~ 05)
            // ========================================================
            else if((i + 2) < data_length && wifi_spi_receive_data[i] == 0xAA && wifi_spi_receive_data[i+1] == 0xFF)
            {
                uint8 ch = wifi_spi_receive_data[i+2];
                if(ch >= 1 && ch <= 5) // 确保通道号在 1 到 5 之间
                {
                    // 翻转对应通道的状态 (数组下标是 0 到 4)
                    channel_show[ch - 1] ^= 1; 
                    printf("\r\n >>> Channel %d Visibility: %d <<< \r\n", ch, channel_show[ch - 1]);
                }
                i += 2;
                continue;
            }
            // ========================================================
            // 协议 2: 电机控制指令 (帧头: 0xA5 + 功能字 + 数据 + 校验 = 7字节)
            // ========================================================
            // 确保剩余未处理的数据至少还有 7 个字节
            else if((i + 6) < data_length && wifi_spi_receive_data[i] == 0xA5)
            {
                uint8 sum_check = 0;
                
                // 计算校验和 (Byte 0 到 Byte 5 的累加低8位)
                for(uint8 j = 0; j < 6; j++)
                {
                    sum_check += wifi_spi_receive_data[i + j];
                }
                
                // 验证校验和是否正确
                if(sum_check == wifi_spi_receive_data[i + 6])
                {
                    uint8 cmd = wifi_spi_receive_data[i + 1]; // 提取功能字
                    
                    if(cmd == 0x01) // 0x01: 收到修改占空比的指令
                    {
                        // 拼合 16 位有符号整数
                        int16 l_duty = (int16)((wifi_spi_receive_data[i+2] << 8) | wifi_spi_receive_data[i+3]);
                        int16 r_duty = (int16)((wifi_spi_receive_data[i+4] << 8) | wifi_spi_receive_data[i+5]);
                        
                        // 调用无刷电机驱动函数
                        small_driver_set_duty(l_duty, r_duty);
                        
                        printf("\r\n Motor Cmd: L=%d, R=%d", l_duty, r_duty); // 调试时可以打开看看
                    }
                    
                    i += 6; // 成功解析一帧，跳过这 7 个字节
                    continue;
                }
            }
        }
    }
}
/**
 * @brief WiFi 定时发送任务 (已完美适配逐飞官方库)
 * 建议在 PIT 定时器中断中调用，周期 20ms
 */
void wifi_report_task(void)
{
    switch(current_wifi_mode)
    {
        case WIFI_MODE_WAVE:
            if(wave_format == 0) // ============ HEX 模式 ============
            {
                uint8 idx = 0; // 用来记录当前塞进去了几个有效数据
                
                // 只有标志位为 1，才把数据塞进逐飞的结构体里
                if(channel_show[0]) seekfree_assistant_oscilloscope_data.data[idx++] = IMU_data.filter_result.yaw;
                if(channel_show[1]) seekfree_assistant_oscilloscope_data.data[idx++] = IMU_data.filter_result.pitch;
                if(channel_show[2]) seekfree_assistant_oscilloscope_data.data[idx++] = IMU_data.filter_result.roll;
                if(channel_show[3]) seekfree_assistant_oscilloscope_data.data[idx++] = motor_value.receive_left_speed_data;
                if(channel_show[4]) seekfree_assistant_oscilloscope_data.data[idx++] = motor_value.receive_right_speed_data;
                
                seekfree_assistant_oscilloscope_data.channel_num = idx;
                
                // 只有当至少有1个通道开启时，才发送 HEX 包
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
                char text_buffer[128] = {0}; // 总的字符串盒子
                char temp[32];               // 临时拼装的小盒子
                
                // 利用 strcat 像接火车车厢一样，把开启的通道接上去
                if(channel_show[0]) { sprintf(temp, "Y:%.2f  ", IMU_data.filter_result.yaw);   strcat(text_buffer, temp); }
                if(channel_show[1]) { sprintf(temp, "P:%.2f  ", IMU_data.filter_result.pitch); strcat(text_buffer, temp); }
                if(channel_show[2]) { sprintf(temp, "R:%.2f  ", IMU_data.filter_result.roll);  strcat(text_buffer, temp); }
                if(channel_show[3]) { sprintf(temp, "L:%d  ", motor_value.receive_left_speed_data); strcat(text_buffer, temp); }
                if(channel_show[4]) { sprintf(temp, "R:%d  ", motor_value.receive_right_speed_data); strcat(text_buffer, temp); }
                
                // 如果最终接好的火车不是空的，就加上换行符发出去
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
            // ?? 致命警告：绝对不能在定时器中断里直接调用 seekfree_assistant_camera_send()！
            // 两万字节的数据会把中断卡死！所以这里只置位一个标志，让主循环去发。
            wifi_spi_send_buffer((uint8*)"IMAGE MODE NOW\r\n", 22);
            //trigger_image_send_in_main = 1; 
            break;

        case WIFI_MODE_LOG:
            // 【模式3：日志模式测试】
            wifi_spi_send_buffer((uint8*)"System Running OK...\r\n", 22);
            break;

        default:
            break; // 模式0：静默
    }
}