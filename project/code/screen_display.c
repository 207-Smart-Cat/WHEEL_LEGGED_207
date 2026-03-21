#include "zf_common_headfile.h"
#include "screen_display.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "ipc_shared_data.h"

// ** 全局变量区域 **
uint8 IPS200_flag = 0;      // 屏幕显示flag（PIT中断置位）
uint8 current_page = 0;     // 当前页面索引 (0:第一页, 1:第二页)
uint8 force_ui_refresh = 1;
// 定义翻页按键引脚
#define PAGE_SWITCH_BTN  P20_0

/**
 * @brief 屏幕与硬件初始化
 */
void screen_display_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE);

    // 初始化按键引脚：方向为输入(GPI)，初始电平1，开启内部上拉
    gpio_init(PAGE_SWITCH_BTN, GPI, 1, GPI_PULL_UP);
}
// ================= 页面 1：运动状态监视 ==============
void show_page_1(void)
{
    uint16_t col_height = 30; 
    uint16_t y_start = 81;                 
    uint16_t text_x = 10;                  
    char temp_str[20];                     

    // ==========================================
    // 【静态绘制区】只在刚开机或刚翻页时画一次！
    // ==========================================
    if (force_ui_refresh)
    {
        // 这个 38KB 的性能刺客，终于被关进笼子了！
        ips200_show_rgb565_image(0, 0, (const uint16_t *)gImage_seekfree_logo, 240, 80, 240, 80, 0);
        ips200_draw_line(0, 80, 239, 80, RGB565_SKYBLUE);
        
        // 提前画好死文字和死线条
        ips200_show_string(text_x, y_start + 5, "Roll  :");
        ips200_draw_line(0, y_start + col_height, 239, y_start + col_height, RGB565_SKYBLUE);
        ips200_show_string(text_x, y_start + col_height + 5, "Pitch :");
        ips200_draw_line(0, y_start + col_height*2, 239, y_start + col_height*2, RGB565_SKYBLUE);
        ips200_show_string(text_x, y_start + col_height*2 + 5, "Yaw   :");
        ips200_draw_line(0, y_start + col_height*3, 239, y_start + col_height*3, RGB565_SKYBLUE);
        
        // 中间的十字分割线等
        ips200_draw_line(119, y_start + col_height*3, 119, y_start + col_height*5, RGB565_SKYBLUE);
        ips200_draw_line(0, y_start + col_height*4, 239, y_start + col_height*4, RGB565_SKYBLUE);
        ips200_draw_line(0, y_start + col_height*5, 239, y_start + col_height*5, RGB565_SKYBLUE);
    }

    // ==========================================
    // 【动态刷新区】每 50ms 都会执行，只更新跳动的数字
    // ==========================================
    ips200_show_float(text_x + 60, y_start + 5, core_a_status.roll, 3, 3);
    ips200_show_float(text_x + 60, y_start + col_height + 5, core_a_status.pitch, 3, 3);
    ips200_show_float(text_x + 60, y_start + col_height*2 + 5, core_a_status.yaw, 3, 3);

    // 速度更新 (因为宽度不定，用了 %-5d，能自动清除后面的残影)
    sprintf(temp_str, "Spd L:%-5d", core_a_status.left_wheel_speed);
    ips200_show_string(text_x, y_start + col_height*3 + 5, temp_str);
    sprintf(temp_str, "R:%-5d", core_a_status.right_wheel_speed);
    ips200_show_string(125, y_start + col_height*3 + 5, temp_str);

    // PWM更新
    sprintf(temp_str, "PWM L:%-5d", core_a_status.left_pwm_duty);
    ips200_show_string(text_x, y_start + col_height*4 + 5, temp_str);
    sprintf(temp_str, "R:%-5d", core_a_status.right_pwm_duty);
    ips200_show_string(125, y_start + col_height*4 + 5, temp_str);
}

// ================= 页面 2：系统参数矩阵 ==============
void show_page_2(void)
{
    // 顶部标题区域
    ips200_show_string(10, 5, "--- System Parameters ---");
    ips200_draw_line(0, 25, 239, 25, RGB565_SKYBLUE);

    // 参数简写映射表（适配小屏幕）
    static const char* short_names[PARAM_COUNT] = {
        "Q_Yw", "Q_Pr", "Q_Bs", "R_Yw", "R_Pr", 
        "Sp_P", "Sp_I", "Sp_D",            
        "An_P", "An_I", "An_D",            
        "Gy_P", "Gy_I", "Gy_D",               
        "T_Vl", "T_Ag", "T_St"
    };

    char temp_str[20];
    uint16_t start_y = 26;          // 表格数据区起始Y
    uint16_t row_height = 23;       // 极致压缩的表格行高（9行恰好占满剩余高度）

    // 【新增】画一条垂直中轴线，让双列布局变成真正的表格
    ips200_draw_line(119, 25, 119, 239, RGB565_SKYBLUE); 

    // 遍历所有参数，绘制网格和数据
    for (int i = 0; i < PARAM_COUNT; i++)
    {
        uint16_t row = i / 2;     // 计算在第几行
        uint16_t col = i % 2;     // 计算在左列(0)还是右列(1)
        
        uint16_t x = col * 120 + 5;                  // 文本X坐标
        uint16_t y = start_y + row * row_height + 4; // 文本Y坐标，+4是为了文字在格子中居中对齐

        // %-4s预留4个字符的变量名，%-5.2f预留带两位小数的值
        sprintf(temp_str, "%-4s:%-5.3f", short_names[i], core_b_cmd.params[i]);
        ips200_show_string(x, y, temp_str);
        
        // 当画完右边这一列，或者当前是最后一个参数时，画一条贯穿屏幕的水平网格线
        if (col == 1 || i == PARAM_COUNT - 1) 
        {
            uint16_t line_y = start_y + (row + 1) * row_height;
            ips200_draw_line(0, line_y, 239, line_y, RGB565_SKYBLUE);
        }
    }
}

// ================= 主控：屏幕刷新与按键检测 ==============
void screen_display_process(void) 
{
    if (IPS200_flag)
    {
        IPS200_flag = 0; 
        static uint8 last_btn_state = 1;               
        uint8 curr_btn_state = gpio_get_level(PAGE_SWITCH_BTN); 

        if (last_btn_state == 1 && curr_btn_state == 0) 
        {
            current_page = !current_page; 
            ips200_clear();   
            force_ui_refresh = 1; // 【新增】翻页后，必须重新画新页面的死物（网格、Logo）
        }
        last_btn_state = curr_btn_state;  

        // 2. 路由到对应的页面绘制逻辑
        if (current_page == 0) show_page_1();
        else                   show_page_2();
        
        force_ui_refresh = 0; // 【新增】画完一帧后，清除全局刷新标志
    }
}