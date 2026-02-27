#include "imu.h"
//中断相关宏定义
#define PIT_IMU                 (PIT_CH0 )  
#define PIT_IPS               (PIT_CH1 )
//GPIO端口宏定义
#define LED1                    (P19_0)
//IPS200相关宏定义
#define IPS200_TYPE     (IPS200_TYPE_SPI) 
#define RGB565_SKYBLUE  0x87CE
//WIFI通信相关宏定义
#define WIFI_SSID_TEST          "test207"
#define WIFI_PASSWORD_TEST      "12345678"                      // 如果需要连接的WIFI 没有密码 替换为 NULL
#define TCP_TARGET_IP           "192.168.99.136"                 // 连接目标的 IP
#define TCP_TARGET_PORT         "8086"                          // 连接目标的端口
#define WIFI__LOCAL_PORT        "6666"                          // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

// **************************** 全局变量区域 ****************************
// 1. 屏幕显示flag（PIT中断置位）
uint8 IPS200_flag = 0;  
uint32 IPC_ReceiveData;
uint8 wifi_spi_receive_data[2560];
uint8 wifi_spi_receive_data_ips[2560];
uint32 data_length;
// **************************** 封装函数区域 ****************************
// 2. 封装屏幕显示函数（原pit0_ch1_isr的耗时操作）
void screen_display_process(void)
{
    if(IPS200_flag)
    {
        IPS200_flag = 0;  // 清除flag
        
        // 原中断里的所有屏幕显示逻辑（完全保留，仅移到主循环）
        // 2. 显示顶部logo图片（240×80，坐标(0,0)）
        ips200_show_rgb565_image(0, 0, (const uint16_t *)gImage_seekfree_logo, 240, 80, 240, 80, 0);

        // 3. 绘制图片与数据区的粉色分隔线（横线，宽度240）
        ips200_draw_line(0, 80, 239, 80, RGB565_SKYBLUE);

        // 4. 分栏参数配置
        uint16_t col_height = 30;             // 每个栏位的高度（像素）
        uint16_t y_start = 81;                // 第一个栏位的起始Y坐标（分隔线下）
        uint16_t text_x = 10;                 // 文字起始X坐标
        uint16_t line_color = RGB565_SKYBLUE;    // 分隔线颜色

        // 5. 逐行绘制8个栏位 + 粉色分隔线
        // ---------------- 第1栏：模式 ----------------
        ips200_show_string(text_x, y_start + 5, "Mode:");
        //    ips200_show_string(text_x + 60, y_start + 5, disp_data.mode);
        // 绘制栏位底部分隔线
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height; // 切换到下一栏

        // ---------------- 第2栏：Roll角 ----------------
        ips200_show_string(text_x, y_start + 5, "Roll:");
        ips200_show_float(text_x + 60, y_start + 5, IMU_data.filter_result.roll, 3, 3);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;

        // ---------------- 第3栏：Pitch角 ----------------
        ips200_show_string(text_x, y_start + 5, "Pitch:");
        ips200_show_float(text_x + 60, y_start + 5, IMU_data.filter_result.pitch, 3, 3);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;

        // ---------------- 第4栏：Yaw角 ----------------
        ips200_show_string(text_x, y_start + 5, "Yaw:");
        ips200_show_float(text_x + 60, y_start + 5, IMU_data.filter_result.yaw, 3, 3);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;
        
        // ---------------- 第5栏：ipc传输内容----------------
        ips200_show_string(text_x,y_start + 5,"IPC:");
        ips200_show_uint(text_x + 60, y_start + 5,IPC_ReceiveData,4);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;
        
        // ---------------- 第6栏：WIFI传输内容--------------
        ips200_show_string(text_x,y_start + 5,"WIFI:");
        ips200_show_string(text_x + 60, y_start + 5,(char *)wifi_spi_receive_data_ips);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;
        
    }
}
//3.WiFi操作封装函数 
/*
 * @brief WiFi模块初始化并连接指定WiFi热点
 * @return uint8 0-初始化成功，非0-失败（实际本函数会循环重试直到成功）
 */
static void wifi_spi_init_connect(void)
{
    uint8 wifi_spi_test_buffer[] = "this is wifi spi test buffer";

    
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }
    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if(0 == WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
            TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI__LOCAL_PORT))                                                  // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }


    // 发送测试数据至服务器
    data_length = wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer));
    if(!data_length)
    {
        printf("\r\n send success.");
    }
    else
    {
        printf("\r\n %d bytes data send failed.", data_length);
    }

}
static void wifi_spi_receive_loop(void)
{
    data_length = wifi_spi_read_buffer(wifi_spi_receive_data, sizeof(wifi_spi_receive_data));
    if(data_length)                                                     // 如果接收到数据 则进行数据类型判断
    {
        printf("\r\n Get data: <%s>.", wifi_spi_receive_data);

        if(!wifi_spi_send_buffer(wifi_spi_receive_data, data_length))
        {
            printf("\r\n send success.");
            memset(wifi_spi_receive_data_ips, 0, data_length); 
            strcpy((char *)wifi_spi_receive_data_ips,(char *)wifi_spi_receive_data);
            memset(wifi_spi_receive_data, 0, data_length);           // 数据发送完成 清空数据
        }
        else
        {
            printf("\r\n %d bytes data send failed.", data_length);
        }
    }
}
// 定义数据接收回调函数 如果另外一个核心发送信息 此核心会触发中断并且可以在回调函数读取数据
void my_ipc_callback(uint32 receive_data)
{
      IPC_ReceiveData=receive_data;
}


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
//    SCB_DisableDCache(); // 关闭DCache
    interrupt_global_disable();	
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);          // 初始化 LED1 输出 默认高电平 推挽输出模式
    imu_init(LED1);
    gnss_init(TAU1201);
//    ipc_communicate_init(IPC_PORT_1, my_ipc_callback);
    
    
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
 //   wifi_spi_init_connect();
    
    pit_ms_init(PIT_IMU, 5);  
    pit_ms_init(PIT_IPS, 100);
    
    interrupt_global_enable(0);	
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
      
//        wifi_spi_receive_loop();
        screen_display_process();
        
        
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
