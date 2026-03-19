#include "screen_display.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "ipc_shared_data.h"

// ** 全局变量区域 **
uint8 IPS200_flag = 0; //  屏幕显示flag（PIT中断置位）

/**
 * @brief 屏幕硬件初始化
 */
void screen_display_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
}

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
        uint16_t col_height = 30;              // 每个栏位的高度（像素）
        uint16_t y_start = 81;                 // 第一个栏位的起始Y坐标（分隔线下）
        uint16_t text_x = 10;                  // 文字起始X坐标
        uint16_t line_color = RGB565_SKYBLUE; // 分隔线颜色
        char temp_str[16];                    // 用于格式化字符串的缓冲区

        // ---------------- 第1栏：模式 ----------------
        ips200_show_string(text_x, y_start + 5, "Mode: Run");
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;

        // ---------------- 第2栏：Roll角 ----------------
        ips200_show_string(text_x, y_start + 5, "Roll:");
        ips200_show_float(text_x + 60, y_start + 5, core_a_status.roll, 3, 3);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;

        // ---------------- 第3栏：Pitch角 ----------------
        ips200_show_string(text_x, y_start + 5, "Pitch:");
        ips200_show_float(text_x + 60, y_start + 5, core_a_status.pitch, 3, 3);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;

        // ---------------- 第4栏：Yaw角 ----------------
        ips200_show_string(text_x, y_start + 5, "Yaw:");
        ips200_show_float(text_x + 60, y_start + 5, core_a_status.yaw, 3, 3);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;

        // ---------------- 第5栏：左轮转速 ----------------
        ips200_show_string(text_x, y_start + 5, "L_Spd:");
        // 使用 sprintf 格式化为占用 5 个字符宽度并左对齐，防止数字变小时尾部残影
        sprintf(temp_str, "%-5d", core_a_status.left_wheel_speed);
        ips200_show_string(text_x + 60, y_start + 5, temp_str);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;

        // ---------------- 第6栏：右轮转速 ----------------
        ips200_show_string(text_x, y_start + 5, "R_Spd:");
        sprintf(temp_str, "%-5d", core_a_status.right_wheel_speed);
        ips200_show_string(text_x + 60, y_start + 5, temp_str);
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, line_color);
        y_start += col_height;
    }
}