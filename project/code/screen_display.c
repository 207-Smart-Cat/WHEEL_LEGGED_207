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
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

typedef struct {
    ParamID_e id;
    uint16_t x;
    uint16_t y;
    const char *format;
} screen_param_item_t;

static const screen_param_item_t k_page2_items[] = {
    {P_SPEED_P, 40,  60, "%-6.3f"},
    {P_SPEED_I, 106, 60, "%-6.3f"},
    {P_SPEED_D, 172, 60, "%-6.3f"},
    {P_ANGLE_P, 40,  82, "%-6.2f"},
    {P_ANGLE_I, 106, 82, "%-6.3f"},
    {P_ANGLE_D, 172, 82, "%-6.3f"},
    {P_GYRO_P,  40,  104, "%-6.2f"},
    {P_GYRO_I,  106, 104, "%-6.3f"},
    {P_GYRO_D,  172, 104, "%-6.3f"},
    {P_LEG_KP,  40,  126, "%-6.3f"},
    {P_LEG_KI,  106, 126, "%-6.3f"},
    {P_LEG_KD,  172, 126, "%-6.3f"},
    {P_AIR_ROLL_P, 40,  148, "%-6.1f"},
    {P_AIR_ROLL_I, 106, 148, "%-6.3f"},
    {P_AIR_ROLL_D, 172, 148, "%-6.3f"},
    {P_DIR_P,   40,  170, "%-6.4f"},
    {P_DIR_I,   106, 170, "%-6.4f"},
    {P_DIR_D,   172, 170, "%-6.4f"},
    {P_TARGET_VELOCITY, 42,  195, "V:%-5.1f"},
    {P_TARGET_ANGLE, 135, 195, "A:%-5.1f"},
    {P_TARGET_MOTOR_STAND, 42,  215, "S:%-5.2f"},
    {P_X_CURRENT, 135, 215, "X:%-5.3f"},
    {P_Y_CURRENT, 42,  235, "Y:%-5.3f"},
    {P_Q_YAW,   42,  260, "Qy:%-5.3f"},
    {P_Q_PR,    135, 260, "Qp:%-5.3f"},
    {P_Q_BIAS,  42,  280, "Qb:%-5.3f"},
    {P_R_YAW,   135, 280, "Ry:%-5.3f"},
    {P_R_PR,    42,  300, "Rp:%-5.3f"}
};

static const screen_param_item_t k_page3_items[] = {
    {P_NAV_Q_V, 145, 60,  "%-8.5f"},
    {P_NAV_Q_W, 145, 82,  "%-8.5f"},
    {P_NAV_Q_BIAS_AX, 145, 104, "%-8.5f"},
    {P_NAV_Q_BIAS_W, 145, 126, "%-8.5f"},
    {P_NAV_R_V_NORMAL, 145, 148, "%-8.5f"},
    {P_NAV_R_V_SLIP, 145, 170, "%-8.5f"},
    {P_NAV_R_W_NORMAL, 145, 192, "%-8.5f"},
    {P_NAV_R_W_SLIP, 145, 214, "%-8.5f"},
    {P_NAV_R_GYRO, 145, 236, "%-8.5f"},
    {P_MAG_OFFSET_X, 75,  265, "%-6.1f"},
    {P_MAG_OFFSET_Y, 170, 265, "%-6.1f"},
    {P_MAG_SCALE_X,  75,  287, "%-6.2f"},
    {P_MAG_SCALE_Y,  170, 287, "%-6.2f"}
};

static const float *screen_get_param_values(void)
{
    if (core_a_status.heartbeat > 0)
    {
        return core_a_status.act_params;
    }

    return core_b_cmd.params;
}

static void screen_show_param_items(const screen_param_item_t *items, uint32_t count, const float *values)
{
    char temp_str[24];

    for (uint32_t i = 0; i < count; i++)
    {
        sprintf(temp_str, items[i].format, values[items[i].id]);
        ips200_show_string(items[i].x, items[i].y, temp_str);
    }
}

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
    uint16_t row_height = 26;
    uint16_t y_start = 32;
    char temp_str[32];

    if (force_ui_refresh)
    {
        ips200_show_string(28, 8, "--- Realtime Monitor ---");
        ips200_draw_line(0, 26, 239, 26, RGB565_SKYBLUE);

        for (int i = 0; i <= 11; i++) {
            ips200_draw_line(0, y_start + row_height * i, 239, y_start + row_height * i, RGB565_SKYBLUE);
        }

        ips200_draw_line(119, y_start + row_height * 3, 119, y_start + row_height * 11, RGB565_SKYBLUE);
    }

    sprintf(temp_str, "Roll : %-6.2f", core_a_status.roll);
    ips200_show_string(8, y_start + 5, temp_str);
    sprintf(temp_str, "Pitch: %-6.2f", core_a_status.pitch);
    ips200_show_string(8, y_start + row_height + 5, temp_str);
    sprintf(temp_str, "Yaw  : %-6.2f", core_a_status.yaw);
    ips200_show_string(8, y_start + row_height * 2 + 5, temp_str);

    sprintf(temp_str, "SpdL:%-5d", core_a_status.left_wheel_speed);
    ips200_show_string(5, y_start + row_height * 3 + 5, temp_str);
    sprintf(temp_str, "R:%-5d", core_a_status.right_wheel_speed);
    ips200_show_string(125, y_start + row_height * 3 + 5, temp_str);

    sprintf(temp_str, "PWML:%-5d", core_a_status.left_pwm_duty);
    ips200_show_string(5, y_start + row_height * 4 + 5, temp_str);
    sprintf(temp_str, "R:%-5d", core_a_status.right_pwm_duty);
    ips200_show_string(125, y_start + row_height * 4 + 5, temp_str);

    sprintf(temp_str, "SpdO:%-5.1f", core_a_status.pid_out_speed_l);
    ips200_show_string(5, y_start + row_height * 5 + 5, temp_str);
    sprintf(temp_str, "R:%-5.1f", core_a_status.pid_out_speed_r);
    ips200_show_string(125, y_start + row_height * 5 + 5, temp_str);

    sprintf(temp_str, "AngO:%-5.1f", core_a_status.pid_out_angle_l);
    ips200_show_string(5, y_start + row_height * 6 + 5, temp_str);
    sprintf(temp_str, "R:%-5.1f", core_a_status.pid_out_angle_r);
    ips200_show_string(125, y_start + row_height * 6 + 5, temp_str);

    sprintf(temp_str, "GyrO:%-5.1f", core_a_status.pid_out_gyro_l);
    ips200_show_string(5, y_start + row_height * 7 + 5, temp_str);
    sprintf(temp_str, "R:%-5.1f", core_a_status.pid_out_gyro_r);
    ips200_show_string(125, y_start + row_height * 7 + 5, temp_str);

    sprintf(temp_str, "Turn:%-6.1f", core_a_status.pid_out_turn);
    ips200_show_string(5, y_start + row_height * 8 + 5, temp_str);
    sprintf(temp_str, "Leg:%-6.3f", core_a_status.pid_out_leg);
    ips200_show_string(125, y_start + row_height * 8 + 5, temp_str);

    sprintf(temp_str, "X:%-6.3f", core_a_status.nav_x);
    ips200_show_string(5, y_start + row_height * 9 + 5, temp_str);
    sprintf(temp_str, "Y:%-6.3f", core_a_status.nav_y);
    ips200_show_string(125, y_start + row_height * 9 + 5, temp_str);

    sprintf(temp_str, "V:%-6.3f", core_a_status.nav_v);
    ips200_show_string(5, y_start + row_height * 10 + 5, temp_str);
    sprintf(temp_str, "W:%-6.1f", core_a_status.nav_w);
    ips200_show_string(125, y_start + row_height * 10 + 5, temp_str);
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
    // 【动态刷新区】优先显示 Core A 实际生效参数
    // ==========================================
    const float *p = screen_get_param_values();
    screen_show_param_items(k_page2_items, ARRAY_SIZE(k_page2_items), p);
}

// ================= 页面 3：导航与磁力计专页 ==============
void show_page_3(void)
{
    // ==========================================
    // 【静态绘制区】
    // ==========================================
    if (force_ui_refresh)
    {
        ips200_show_string(25, 10, "--- Nav & Mag Params ---");
        
        // --- 绘制水平分割线 ---
        ips200_draw_line(0, 32, 239, 32, RGB565_SKYBLUE);   // 标题底线
        ips200_draw_line(38, 55, 239, 55, RGB565_SKYBLUE);  // 表头底线
        
        // --- 绘制垂直侧边栏分割线 (贯穿全屏到底部 319) ---
        ips200_draw_line(38, 32, 38, 319, RGB565_SKYBLUE);  

        // --- 表头提示 ---
        ips200_show_string(45, 37, "Parameter Name");
        ips200_show_string(155, 37, "Value");

        // --- [上半区] 队友的 9 个导航参数 ---
        ips200_show_string(4, 60,  "Nav"); 
        
        ips200_show_string(42, 60,  "Q_V:");      
        ips200_show_string(42, 82,  "Q_W:");      
        ips200_show_string(42, 104, "Q_BAx:");    
        ips200_show_string(42, 126, "Q_BW:");     
        ips200_show_string(42, 148, "RV_Nor:");   
        ips200_show_string(42, 170, "RV_Slp:");   
        ips200_show_string(42, 192, "RW_Nor:");   
        ips200_show_string(42, 214, "RW_Slp:");   
        ips200_show_string(42, 236, "R_Gyro:");   

        // --- [下半区] 你的 4 个磁力计参数 (双列紧凑布局) ---
        ips200_draw_line(38, 255, 239, 255, RGB565_SKYBLUE); // 区块横向分割线
        ips200_show_string(4, 275, "Mag"); 
        
        // 左列
        ips200_show_string(42, 265, "OX:");
        ips200_show_string(42, 287, "SX:");
        // 右列
        ips200_show_string(140, 265, "OY:");
        ips200_show_string(140, 287, "SY:");
    }

    // ==========================================
    // 【动态刷新区】优先显示 Core A 实际生效参数
    // ==========================================
    const float *p = screen_get_param_values();
    screen_show_param_items(k_page3_items, ARRAY_SIZE(k_page3_items), p);
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