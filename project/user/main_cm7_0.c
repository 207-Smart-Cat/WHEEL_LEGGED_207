#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"
#include "imu.h"
#include "engine.h"
#include "control.h"

// **************************** 宏定义区域 ****************************
// 中断
#define PIT_IMU (PIT_CH0)
#define PIT_IPS (PIT_CH1)
#define PIT_Motor_Control (PIT_CH2)
#define PIT_Balance (PIT_CH10)
// GPIO端口
#define LED1 (P19_0)
// IPS200
#define IPS200_TYPE (IPS200_TYPE_SPI)
#define RGB565_SKYBLUE 0x87CE
// 电机

// **************************** 全局变量区域 ****************************
// IPS200
uint8 IPS200_flag = 0; //  屏幕显示flag（PIT中断置位）
// 电机（调试时）
// uint8 Motor_Control_flag = 0; // 50ms 电机控制标志位
// int duty = 0;
// bool dir = true;
// 电机+舵机（运行）
float angel_init = 0;
int duty = 1;
int stop = 0;
extern float pitch1, roll1, yaw1;
float v_buchang;
/*腿部姿态设置*/
float x_current, y_current=0.08;     // 用于舵机（x，y）位置的调整,务必记住应该给一个符合区间的初始值，否则舵机将不在转动
extern pid_param_t engine_high; // 发动机高度PID参数（暂时我也不知道干什么的）
int stop_flash = 0;             // 完赛标志位

int Bridge_position = 1;     // 可能是过单边桥时候需要的（腿部自适应）
int yanshi_biaozhiwei = 100; // 可能是过单边桥时候需要的（腿部自适应模式）
int change_speed = 0;

// imu

// **************************** 封装调试部分函数区域 ****************************
// ================= 函数1：屏幕控制函数(200ms) ==============
void screen_display_process(void) //  封装屏幕显示函数
{
    if (IPS200_flag)
    {
        IPS200_flag = 0; // 清除flag

        // 2. 显示顶部logo图片（240×80，坐标(0,0)）
        ips200_show_rgb565_image(0, 0, (const uint16_t *)gImage_seekfree_logo, 240, 80, 240, 80, 0);

        // 3. 绘制图片与数据区的粉色分隔线（横线，宽度240）
        ips200_draw_line(0, 80, 239, 80, RGB565_SKYBLUE);

        // 4. 分栏参数配置
        uint16_t col_height = 30;             // 每个栏位的高度（像素）
        uint16_t y_start = 81;                // 第一个栏位的起始Y坐标（分隔线下）
        uint16_t text_x = 10;                 // 文字起始X坐标
        uint16_t line_color = RGB565_SKYBLUE; // 分隔线颜色
        char temp_str[16];                    // 用于格式化字符串的缓冲区

        // ---------------- 第1栏：模式 ----------------
        ips200_show_string(text_x, y_start + 5, "Mode: Run");
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;

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

        // ---------------- 第5栏：左轮转速 ----------------
        ips200_show_string(text_x, y_start + 5, "L_Spd:");
        // 使用 sprintf 格式化为占用 5 个字符宽度并左对齐，防止数字变小时尾部残影
        sprintf(temp_str, "%-5d", motor_value.receive_left_speed_data);
        ips200_show_string(text_x + 60, y_start + 5, temp_str);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;

        // ---------------- 第6栏：右轮转速 ----------------
        ips200_show_string(text_x, y_start + 5, "R_Spd:");
        sprintf(temp_str, "%-5d", motor_value.receive_right_speed_data);
        ips200_show_string(text_x + 60, y_start + 5, temp_str);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;
    }
}

// ================= 主函数 =================
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_init();                  // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等

    interrupt_global_disable(); // 初始化外设之前先关闭中断
    //=================================GPIO初始化=======================
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化 LED1 输出 默认高电平 推挽输出模式
    //=================================IMU初始化=======================
    imu_init(LED1);
    pit_ms_init(PIT_IMU, 5);
    //=================================屏幕初始化============================
    // ips200_set_dir(IPS200_PORTAIT);
    // ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    // ips200_init(IPS200_TYPE);
    // pit_ms_init(PIT_IPS, 200);
    //==========================平衡动作初始化===========================
    Balance_init(); // 初始化平衡控制（设置Kalman滤波的各个参数）
    pit_ms_init(PIT_Balance, 10);
    //=================================电机控制初始化=======================
    small_driver_uart_init();
    //pit_ms_init(PIT_Motor_Control, 50);

    interrupt_global_enable(0); // 在初始化后使能中断

    
    system_delay_ms(1000);

    while (true)
    {
        // 此处编写需要循环执行的代码
        //screen_display_process();
        // Motor_Control();
        system_delay_ms(1);
    }
}