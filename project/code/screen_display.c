#include "zf_common_headfile.h"
#include "screen_display.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "ipc_shared_data.h"
#include "navigation_data_handling.h"
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
    uint16_t col_height = 34; 
    uint16_t y_start = 81;                 
    char temp_str[30];                     

    if (force_ui_refresh)
    {
        ips200_show_rgb565_image(0, 0, (const uint16_t *)gImage_seekfree_logo, 240, 80, 240, 80, 0);
        ips200_draw_line(0, 80, 239, 80, RGB565_SKYBLUE);
        for(int i = 1; i <= 7; i++) {
            ips200_draw_line(0, y_start + col_height*i, 239, y_start + col_height*i, RGB565_SKYBLUE);
        }
        ips200_draw_line(119, y_start + col_height*3, 119, y_start + col_height*7, RGB565_SKYBLUE);
    }

    // Row 1 到 Row 5 保持不变 ...
    sprintf(temp_str, "Roll : %-6.2f", core_a_status.roll);
    ips200_show_string(10, y_start + 8, temp_str);
    sprintf(temp_str, "Pitch: %-6.2f", core_a_status.pitch);
    ips200_show_string(10, y_start + col_height + 8, temp_str);
    sprintf(temp_str, "Yaw  : %-6.2f", core_a_status.yaw);
    ips200_show_string(10, y_start + col_height*2 + 8, temp_str);
    sprintf(temp_str, "SpdL:%-5d", core_a_status.left_wheel_speed);
    ips200_show_string(5, y_start + col_height*3 + 8, temp_str);
    sprintf(temp_str, "R:%-5d", core_a_status.right_wheel_speed);
    ips200_show_string(125, y_start + col_height*3 + 8, temp_str);
    sprintf(temp_str, "PWML:%-5d", core_a_status.left_pwm_duty);
    ips200_show_string(5, y_start + col_height*4 + 8, temp_str);
    sprintf(temp_str, "R:%-5d", core_a_status.right_pwm_duty);
    ips200_show_string(125, y_start + col_height*4 + 8, temp_str);

    // ==========================================
    // Row 6 & 7: 【安全修改】直接从 IPC 状态机里读取坐标与速度
    // ==========================================
    sprintf(temp_str, "X:%-6.3f", core_a_status.nav_x);
    ips200_show_string(5, y_start + col_height*5 + 8, temp_str);
    
    sprintf(temp_str, "Y:%-6.3f", core_a_status.nav_y);
    ips200_show_string(125, y_start + col_height*5 + 8, temp_str);

    sprintf(temp_str, "V:%-6.3f", core_a_status.nav_v);
    ips200_show_string(5, y_start + col_height*6 + 8, temp_str);
    
    sprintf(temp_str, "W:%-6.1f", core_a_status.nav_w);
    ips200_show_string(125, y_start + col_height*6 + 8, temp_str);
}

// ================= 页面 2：系统参数矩阵 ==============
void show_page_2(void)
{
    // ==========================================
    // 【静态绘制区】只画一次网格、表头和侧边栏
    // ==========================================
    if (force_ui_refresh)
    {
        ips200_show_string(25, 10, "--- System Parameters ---");
        
        // --- 绘制水平分割线 (为 28 个参数重新压缩布局) ---
        ips200_draw_line(0, 32, 239, 32, RGB565_SKYBLUE);   // 标题底线
        ips200_draw_line(38, 55, 239, 55, RGB565_SKYBLUE);  // PID表头底线
        ips200_draw_line(0, 190, 239, 190, RGB565_SKYBLUE); // 目标指令区顶线 (下移)
        ips200_draw_line(0, 255, 239, 255, RGB565_SKYBLUE); // 滤波参数区顶线 (下移)
        
        // --- 绘制垂直侧边栏分割线 ---
        ips200_draw_line(38, 32, 38, 319, RGB565_SKYBLUE); 

        // --- 上半部：PID 区专属表头 ---
        ips200_show_string(52,  37, "P");
        ips200_show_string(118, 37, "I");
        ips200_show_string(184, 37, "D");

        // --- 左侧固定分类标签 (行间距压缩为 22) ---
        ips200_show_string(4, 60,  "Spd");
        ips200_show_string(4, 82,  "Ang");
        ips200_show_string(4, 104, "Gyr");
        ips200_show_string(4, 126, "Leg");
        ips200_show_string(4, 148, "Air"); // 【新增】空中环
        ips200_show_string(4, 170, "Dir"); // 【新增】方向环
        
        ips200_show_string(4, 215, "Tar"); // 居中于指令区
        ips200_show_string(4, 280, "Kal"); // 居中于滤波区
    }

    // ==========================================
    // 【动态刷新区】每 50ms 更新具体数值
    // ==========================================
    char temp_str[20];
    float *p = core_b_cmd.params; 
    
    // 【上半部分】6 行 PID 坐标 
    uint16_t x1 = 40, x2 = 106, x3 = 172; 
    
    sprintf(temp_str, "%-6.3f", p[P_SPEED_P]); ips200_show_string(x1, 60, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_SPEED_I]); ips200_show_string(x2, 60, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_SPEED_D]); ips200_show_string(x3, 60, temp_str);
    
    sprintf(temp_str, "%-6.2f", p[P_ANGLE_P]); ips200_show_string(x1, 82, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_ANGLE_I]); ips200_show_string(x2, 82, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_ANGLE_D]); ips200_show_string(x3, 82, temp_str);
    
    sprintf(temp_str, "%-6.2f", p[P_GYRO_P]);  ips200_show_string(x1, 104, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_GYRO_I]);  ips200_show_string(x2, 104, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_GYRO_D]);  ips200_show_string(x3, 104, temp_str);
    
    sprintf(temp_str, "%-6.3f", p[P_LEG_KP]);  ips200_show_string(x1, 126, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_LEG_KI]);  ips200_show_string(x2, 126, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_LEG_KD]);  ips200_show_string(x3, 126, temp_str);

    // 【新增】空中环 (保留1到3位小数)
    sprintf(temp_str, "%-6.1f", p[P_AIR_ROLL_P]); ips200_show_string(x1, 148, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_AIR_ROLL_I]); ips200_show_string(x2, 148, temp_str);
    sprintf(temp_str, "%-6.3f", p[P_AIR_ROLL_D]); ips200_show_string(x3, 148, temp_str);

    // 【新增】方向环 (极小值，强制保留4位小数，防止显示为0)
    sprintf(temp_str, "%-6.4f", p[P_DIR_P]); ips200_show_string(x1, 170, temp_str);
    sprintf(temp_str, "%-6.4f", p[P_DIR_I]); ips200_show_string(x2, 170, temp_str);
    sprintf(temp_str, "%-6.4f", p[P_DIR_D]); ips200_show_string(x3, 170, temp_str);

    // 【下半部分】目标与滤波区 (共 6 行，直达屏幕底部 320 边缘)
    uint16_t c1 = 42, c2 = 135;
    
    // --- 目标指令与腿部位置区 (重组布局) ---
    sprintf(temp_str, "V:%-5.1f", p[P_TARGET_VELOCITY]);    ips200_show_string(c1, 195, temp_str);
    sprintf(temp_str, "A:%-5.1f", p[P_TARGET_ANGLE]);       ips200_show_string(c2, 195, temp_str);
    
    sprintf(temp_str, "S:%-5.2f", p[P_TARGET_MOTOR_STAND]); ips200_show_string(c1, 215, temp_str);
    sprintf(temp_str, "X:%-5.3f", p[P_X_CURRENT]);          ips200_show_string(c2, 215, temp_str);
    
    sprintf(temp_str, "Y:%-5.3f", p[P_Y_CURRENT]);          ips200_show_string(c1, 235, temp_str);

    // --- 卡尔曼滤波参数区 ---
    sprintf(temp_str, "Qy:%-5.3f", p[P_Q_YAW]);             ips200_show_string(c1, 260, temp_str);
    sprintf(temp_str, "Qp:%-5.3f", p[P_Q_PR]);              ips200_show_string(c2, 260, temp_str);
    
    sprintf(temp_str, "Qb:%-5.3f", p[P_Q_BIAS]);            ips200_show_string(c1, 280, temp_str);
    sprintf(temp_str, "Ry:%-5.3f", p[P_R_YAW]);             ips200_show_string(c2, 280, temp_str);
    
    sprintf(temp_str, "Rp:%-5.3f", p[P_R_PR]);              ips200_show_string(c1, 300, temp_str); // 最底下一行
}

// ================= 页面 3：导航卡尔曼滤波专页 ==============
void show_page_3(void)
{
    // ==========================================
    // 【静态绘制区】完全复刻 Page 2 的侧边栏与网格样式
    // ==========================================
    if (force_ui_refresh)
    {
        ips200_show_string(25, 10, "--- Navigation Params ---");
        
        // --- 绘制水平分割线 ---
        ips200_draw_line(0, 32, 239, 32, RGB565_SKYBLUE);   // 标题底线
        ips200_draw_line(38, 55, 239, 55, RGB565_SKYBLUE);  // 表头底线
        
        // --- 绘制垂直侧边栏分割线 ---
        ips200_draw_line(38, 32, 38, 195, RGB565_SKYBLUE);  // 延伸到参数结束处

        // --- 表头提示 ---
        ips200_show_string(45, 37, "Parameter Name");
        ips200_show_string(155, 37, "Value");

        // --- 左侧固定分类标签 (沿用 Page 2 的 22 像素行距) ---
        ips200_show_string(4, 60,  "Nav"); 
        
        // 为了美观，给每行画一个小短线或者直接标注变量名
        ips200_show_string(42, 60,  "Q_X:");
        ips200_show_string(42, 82,  "Q_Y:");
        ips200_show_string(42, 104, "Q_V:");
        ips200_show_string(42, 126, "Q_B_Ax:");
        ips200_show_string(42, 148, "R_Norm:");
        ips200_show_string(42, 170, "R_Slip:");

        // 封底线
        ips200_draw_line(0, 195, 239, 195, RGB565_SKYBLUE);
    }

    // ==========================================
    // 【动态刷新区】从 IPC 参数池读取数值
    // ==========================================
    char temp_str[20];
    float *p = core_b_cmd.params; 
    uint16_t val_x = 145; // 数值起始 X 坐标，避开变量名

    // 针对导航参数，统一保留 5 位小数，左对齐
    sprintf(temp_str, "%-8.5f", p[P_NAV_Q_X]);       ips200_show_string(val_x, 60,  temp_str);
    sprintf(temp_str, "%-8.5f", p[P_NAV_Q_Y]);       ips200_show_string(val_x, 82,  temp_str);
    sprintf(temp_str, "%-8.5f", p[P_NAV_Q_V]);       ips200_show_string(val_x, 104, temp_str);
    sprintf(temp_str, "%-8.5f", p[P_NAV_Q_BIAS_AX]); ips200_show_string(val_x, 126, temp_str);
    sprintf(temp_str, "%-8.5f", p[P_NAV_R_V_NORMAL]);ips200_show_string(val_x, 148, temp_str);
    sprintf(temp_str, "%-8.5f", p[P_NAV_R_V_SLIP]);  ips200_show_string(val_x, 170, temp_str);
}

// ================= 主控：屏幕刷新与按键检测 ==============
void screen_display_process(void) 
{
    if (IPS200_flag)
    {
        IPS200_flag = 0; 
        
        static uint8 last_btn_state = 1;               
        uint8 curr_btn_state = gpio_get_level(PAGE_SWITCH_BTN); 
        static uint8 debounce_cnt = 0; // 【防抖倒计时】

        // 按键防抖处理，防止一次按下狂翻好几页
        if (last_btn_state == 1 && curr_btn_state == 0 && debounce_cnt == 0) 
        {
            // 【核心修改 1】：从非0即1，变成 0 -> 1 -> 2 -> 0 的三阶循环切换
            current_page = (current_page + 1) % 3; 
            
            ips200_clear();   
            force_ui_refresh = 1; 
            debounce_cnt = 5; // 锁定几帧，拒绝连续响应 (大约 250ms)
        }
        last_btn_state = curr_btn_state;  
        if (debounce_cnt > 0) debounce_cnt--;

        // 【核心修改 2】：路由增加第三个页面的出口
        if (current_page == 0)      show_page_1();
        else if (current_page == 1) show_page_2();
        else                        show_page_3(); // 新增的进阶控制与导航页！
        
        force_ui_refresh = 0; 
    }
}