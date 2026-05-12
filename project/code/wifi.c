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
#include "screen_display.h"

// 宏定义和缓冲区
#define WIFI_RX_BUF_SIZE    256
#define WIFI_MAX_RETRY      5
#define WIFI_BOOT_MAX_RETRY 1
#define WIFI_SEND_FAIL_LIMIT        10
#define WIFI_RECONNECT_COOLDOWN_TICKS 100
uint8 wifi_spi_receive_data[WIFI_RX_BUF_SIZE];
static uint32 data_length;

// ================= 全局状态变量 =================
wifi_mode_t current_wifi_mode = WIFI_MODE_SILENT;
uint8 wave_format = 1;
uint8 channel_show[5] = {1, 1, 1, 0, 0};
uint8 wifi_wave_selected_count = 0;
uint8 wifi_wave_selected[WIFI_WAVE_MAX_SELECTED] = {
    WIFI_WAVE_EMPTY_SLOT,
    WIFI_WAVE_EMPTY_SLOT,
    WIFI_WAVE_EMPTY_SLOT,
    WIFI_WAVE_EMPTY_SLOT,
    WIFI_WAVE_EMPTY_SLOT,
    WIFI_WAVE_EMPTY_SLOT
};
volatile uint8 WIFI_Send_flag = 0;

uint8 wifi_is_connected = 0; // 标记 WiFi 是否通关成功 (0:未连接, 1:已连接)
static uint8 wifi_run_without_connection = 0; // 1 表示用户允许未连接 WiFi 也允许运行
static uint8 wifi_reconnect_enabled = 0;      // 1 表示允许后台重连尝试
static volatile uint8 wifi_boot_skip_requested = 0;
static wifi_boot_state_t wifi_boot_state = WIFI_BOOT_STATE_NONE;

// ================= 断线重连与防粘包专用变量 =================
static uint16 wifi_error_count = 0;       // 连续发送失败计数器
static uint16 wifi_reconnect_cooldown = 0;// 重连冷却倒计时

static uint8 wifi_fifo_buffer[512];       // WiFi 专属底层 FIFO 数组
fifo_struct wifi_data_fifo;               // WiFi 专属 FIFO 结构体
static uint8 wifi_parse_buffer[512];      // 拼装完的完整数据包缓冲区

static void wifi_mark_disconnected(const char *reason)
{
    wifi_is_connected = 0;
    wifi_run_without_connection = 1;
    wifi_reconnect_enabled = 1;
    wifi_error_count = 0;
    wifi_reconnect_cooldown = WIFI_RECONNECT_COOLDOWN_TICKS;
    fifo_clear(&wifi_data_fifo);
    printf("\r\n[SYS] WiFi Connection Lost: %s. Entering Auto-Reconnect Mode...\r\n", reason);
}
static void wifi_skip_connection(void)
{
    wifi_is_connected = 0;
    wifi_run_without_connection = 1;
    wifi_reconnect_enabled = 0;
    wifi_error_count = 0;
    wifi_reconnect_cooldown = 0;
    fifo_clear(&wifi_data_fifo);
}

static uint8 wifi_boot_abort_check(void)
{
    if (screen_boot_skip_is_down())
    {
        wifi_boot_skip_requested = 1;
        return 1;
    }
    return 0;
}

static void wifi_set_boot_abort_hook(uint8 enable)
{
    wifi_boot_skip_requested = 0;
    wifi_spi_set_abort_callback(enable ? wifi_boot_abort_check : NULL);
}

static uint8 wifi_boot_skip_requested_or_pressed(uint8 allow_skip)
{
    return (allow_skip && (wifi_boot_skip_requested || screen_boot_skip_pressed())) ? 1 : 0;
}
static const char *const k_wifi_wave_var_names[WIFI_WAVE_VAR_COUNT] = {
    "Roll", "Pitch", "Yaw",
    "L_Spd", "R_Spd", "L_PWM", "R_PWM",
    "SpdO_L", "SpdO_R", "AngO_L", "AngO_R", "GyrO_L", "GyrO_R",
    "TurnO", "LegO", "LegTilt", "LegXOff", "LegXTar", "LegTick", "Battery",
    "NavX", "NavY", "NavV", "NavW", "NavYaw", "NavOK",
    "LegXGain", "LegXLim", "LegXStep", "LegXHit",
    "ZeroSt", "ZeroMs", "ZeroRx", "ZeroSpd", "ZeroStart", "ZeroTx", "ZeroTask", "ZeroRxCnt",
    "TargetVel", "TargetAng", "TargetStand", "XCurrent", "YCurrent"
};

const char *wifi_wave_var_name(wifi_wave_var_t id)
{
    if ((uint8)id >= WIFI_WAVE_VAR_COUNT)
    {
        return "Unknown";
    }
    return k_wifi_wave_var_names[id];
}

float wifi_wave_get_value(wifi_wave_var_t id)
{
    switch (id)
    {
        case WIFI_WAVE_VAR_ROLL:           return core_a_status.roll;
        case WIFI_WAVE_VAR_PITCH:          return core_a_status.pitch;
        case WIFI_WAVE_VAR_YAW:            return core_a_status.yaw;
        case WIFI_WAVE_VAR_LEFT_SPEED:     return (float)core_a_status.left_wheel_speed;
        case WIFI_WAVE_VAR_RIGHT_SPEED:    return (float)core_a_status.right_wheel_speed;
        case WIFI_WAVE_VAR_LEFT_PWM:       return (float)core_a_status.left_pwm_duty;
        case WIFI_WAVE_VAR_RIGHT_PWM:      return (float)core_a_status.right_pwm_duty;
        case WIFI_WAVE_VAR_SPD_OUT_L:      return core_a_status.pid_out_speed_l;
        case WIFI_WAVE_VAR_SPD_OUT_R:      return core_a_status.pid_out_speed_r;
        case WIFI_WAVE_VAR_ANG_OUT_L:      return core_a_status.pid_out_angle_l;
        case WIFI_WAVE_VAR_ANG_OUT_R:      return core_a_status.pid_out_angle_r;
        case WIFI_WAVE_VAR_GYR_OUT_L:      return core_a_status.pid_out_gyro_l;
        case WIFI_WAVE_VAR_GYR_OUT_R:      return core_a_status.pid_out_gyro_r;
        case WIFI_WAVE_VAR_TURN_OUT:       return core_a_status.pid_out_turn;
        case WIFI_WAVE_VAR_LEG_OUT:        return core_a_status.pid_out_leg;
        case WIFI_WAVE_VAR_LEG_SPEED_TILT: return core_a_status.leg_dbg_speed_tilt;
        case WIFI_WAVE_VAR_LEG_X_OFFSET:   return core_a_status.leg_dbg_x_offset;
        case WIFI_WAVE_VAR_LEG_X_TARGET:   return core_a_status.leg_dbg_x_target;
        case WIFI_WAVE_VAR_LEG_TICK:       return core_a_status.leg_dbg_tick;
        case WIFI_WAVE_VAR_BATTERY:        return core_a_status.battery_voltage;
        case WIFI_WAVE_VAR_TARGET_VELOCITY: return core_a_status.target_velocity_status;
        case WIFI_WAVE_VAR_TARGET_ANGLE:    return core_a_status.target_angle_status;
        case WIFI_WAVE_VAR_TARGET_STAND:    return core_a_status.target_motor_stand_status;
        case WIFI_WAVE_VAR_X_CURRENT:       return core_a_status.x_current_status;
        case WIFI_WAVE_VAR_Y_CURRENT:       return core_a_status.y_current_status;
        case WIFI_WAVE_VAR_NAV_X:          return core_a_status.nav_x;
        case WIFI_WAVE_VAR_NAV_Y:          return core_a_status.nav_y;
        case WIFI_WAVE_VAR_NAV_V:          return core_a_status.nav_v;
        case WIFI_WAVE_VAR_NAV_W:          return core_a_status.nav_w;
        case WIFI_WAVE_VAR_NAV_YAW:        return core_a_status.nav_yaw;
        case WIFI_WAVE_VAR_NAV_VALID:      return core_a_status.nav_valid;
        case WIFI_WAVE_VAR_LEG_X_GAIN_USED:     return core_a_status.leg_dbg_x_gain_used;
        case WIFI_WAVE_VAR_LEG_X_LIMIT_USED:    return core_a_status.leg_dbg_x_limit_used;
        case WIFI_WAVE_VAR_LEG_X_STEP_USED:     return core_a_status.leg_dbg_x_step_used;
        case WIFI_WAVE_VAR_LEG_X_LIMIT_HIT:     return core_a_status.leg_dbg_x_limit_hit;
        case WIFI_WAVE_VAR_MOTOR_ZERO_STATE:    return (float)core_a_status.motor_zero_state;
        case WIFI_WAVE_VAR_MOTOR_ZERO_ELAPSED:  return core_a_status.motor_zero_elapsed_ms;
        case WIFI_WAVE_VAR_MOTOR_ZERO_RX:       return core_a_status.motor_zero_rx_seen;
        case WIFI_WAVE_VAR_MOTOR_ZERO_SPEED:    return core_a_status.motor_zero_speed_seen;
        case WIFI_WAVE_VAR_MOTOR_ZERO_START:    return core_a_status.motor_zero_start_count;
        case WIFI_WAVE_VAR_MOTOR_ZERO_TX:       return core_a_status.motor_zero_tx_count;
        case WIFI_WAVE_VAR_MOTOR_ZERO_TASK:     return core_a_status.motor_zero_task_count;
        case WIFI_WAVE_VAR_MOTOR_ZERO_RX_COUNT: return core_a_status.motor_zero_rx_count;
        default:                           return 0.0f;
    }
}

uint8 wifi_wave_is_selected(uint8 id)
{
    uint8 i;
    for (i = 0; i < wifi_wave_selected_count && i < WIFI_WAVE_MAX_SELECTED; i++)
    {
        if (wifi_wave_selected[i] == id)
        {
            return 1;
        }
    }
    return 0;
}

uint8 wifi_wave_toggle_selected(uint8 id)
{
    uint8 i;
    uint8 j;

    if (id >= WIFI_WAVE_VAR_COUNT)
    {
        return 0;
    }

    for (i = 0; i < wifi_wave_selected_count && i < WIFI_WAVE_MAX_SELECTED; i++)
    {
        if (wifi_wave_selected[i] == id)
        {
            for (j = i; j + 1 < wifi_wave_selected_count; j++)
            {
                wifi_wave_selected[j] = wifi_wave_selected[j + 1];
            }
            if (wifi_wave_selected_count > 0)
            {
                wifi_wave_selected_count--;
            }
            wifi_wave_selected[wifi_wave_selected_count] = WIFI_WAVE_EMPTY_SLOT;
            return 1;
        }
    }

    if (wifi_wave_selected_count >= WIFI_WAVE_MAX_SELECTED)
    {
        return 0;
    }

    wifi_wave_selected[wifi_wave_selected_count] = id;
    wifi_wave_selected_count++;
    return 1;
}

void wifi_wave_enter_mode(void)
{
    wave_format = 1;
    current_wifi_mode = WIFI_MODE_WAVE;
}
void wifi_wave_send_var_map(void)
{
    char line[220];
    char item[36];
    uint8 i;
    uint8 col;

    LOG_Printf("\r\nWAVE VAR MAP:\r\n");
    for (i = 0; i < WIFI_WAVE_VAR_COUNT; i += 5)
    {
        line[0] = '\0';
        for (col = 0; col < 5 && (i + col) < WIFI_WAVE_VAR_COUNT; col++)
        {
            sprintf(item, "%02d:%s  ", (int)(i + col + 1), wifi_wave_var_name((wifi_wave_var_t)(i + col)));
            strcat(line, item);
        }
        LOG_Printf("%s\r\n", line);
    }
    LOG_Printf("Use: WAVE 1 4 6  or  WAVE 1,4,6  (max 6 vars)\r\n");
    LOG_Printf("Use: WAVE? show this map. WAVE OFF enter silent mode.\r\n");
}

uint8 wifi_wave_set_selected_ids(const uint8 *ids, uint8 count)
{
    uint8 i;
    uint8 j;
    uint8 unique_count = 0;

    if (ids == NULL || count == 0 || count > WIFI_WAVE_MAX_SELECTED)
    {
        return 0;
    }

    for (i = 0; i < WIFI_WAVE_MAX_SELECTED; i++)
    {
        wifi_wave_selected[i] = WIFI_WAVE_EMPTY_SLOT;
    }

    for (i = 0; i < count; i++)
    {
        uint8 id = ids[i];
        uint8 exists = 0;
        if (id == 0 || id > WIFI_WAVE_VAR_COUNT)
        {
            wifi_wave_selected_count = 0;
            return 0;
        }

        for (j = 0; j < unique_count; j++)
        {
            if (wifi_wave_selected[j] == (uint8)(id - 1))
            {
                exists = 1;
                break;
            }
        }

        if (!exists)
        {
            if (unique_count >= WIFI_WAVE_MAX_SELECTED)
            {
                wifi_wave_selected_count = 0;
                return 0;
            }
            wifi_wave_selected[unique_count] = (uint8)(id - 1);
            unique_count++;
        }
    }

    wifi_wave_selected_count = unique_count;
    if (wifi_wave_selected_count == 0)
    {
        return 0;
    }

    wifi_wave_enter_mode();
    return wifi_wave_selected_count;
}

static void wifi_wave_send_selected_text(void)
{
    char text_buffer[192] = {0};
    char temp[32];
    uint8 i;
    uint8 sent = 0;

    if (wifi_wave_selected_count == 0)
    {
        return;
    }

    IPC_Pull_Status_To_CoreB();

    for (i = 0; i < wifi_wave_selected_count && i < WIFI_WAVE_MAX_SELECTED; i++)
    {
        uint8 id = wifi_wave_selected[i];
        if (id >= WIFI_WAVE_VAR_COUNT)
        {
            continue;
        }
        if (sent == 0)
        {
            sprintf(temp, "%.3f", wifi_wave_get_value((wifi_wave_var_t)id));
        }
        else
        {
            sprintf(temp, ",%.3f", wifi_wave_get_value((wifi_wave_var_t)id));
        }
        strcat(text_buffer, temp);
        sent++;
    }

    if (sent > 0)
    {
        strcat(text_buffer, "\n");
        WIFI_Send_Buffer_Checked((uint8*)text_buffer, strlen(text_buffer), 1);
    }
}
static uint8 wifi_wait_retry_or_skip(uint8 allow_skip, uint16 wait_ms)
{
    while (wait_ms > 0)
    {
        if (wifi_boot_skip_requested_or_pressed(allow_skip))
        {
            wifi_skip_connection();
            return 1;
        }
        system_delay_ms(10);
        wait_ms = (wait_ms > 10) ? (wait_ms - 10) : 0;
    }
    return 0;
}

uint8 wifi_control_is_ready(void)
{
    return (wifi_is_connected || wifi_run_without_connection) ? 1 : 0;
}

wifi_boot_state_t wifi_get_boot_state(void)
{
    return wifi_boot_state;
}

const char *wifi_get_boot_state_text(void)
{
    switch (wifi_boot_state)
    {
        case WIFI_BOOT_STATE_CONNECTED: return "OK";
        case WIFI_BOOT_STATE_SKIPPED:   return "SKIP";
        case WIFI_BOOT_STATE_FAILED:    return "OFFLINE";
        default:                        return "WAIT";
    }
}

void wifi_request_reconnect(void)
{
    wifi_is_connected = 0;
    wifi_run_without_connection = 1;
    wifi_reconnect_enabled = 1;
    wifi_error_count = 0;
    wifi_reconnect_cooldown = 0;
    wifi_boot_state = WIFI_BOOT_STATE_NONE;
    fifo_clear(&wifi_data_fifo);
    printf("\r\n[WIFI] Manual reconnect requested.\r\n");
}

uint8 WIFI_Send_Buffer_Checked(const uint8 *data, uint32 len, uint8 flush_now)
{
    if (data == NULL || len == 0)
    {
        return 1;
    }

    if (wifi_is_connected == 0)
    {
        return 1;
    }

    if (wifi_spi_send_buffer((uint8 *)data, len) != 0)
    {
        wifi_error_count++;
        if (wifi_error_count >= WIFI_SEND_FAIL_LIMIT)
        {
            wifi_mark_disconnected("send failed");
        }
        return 1;
    }

    wifi_error_count = 0;
#if (WIFI_PROTOCOL_MODE == 1)
    if (flush_now)
    {
        wifi_spi_udp_send_now();
    }
#endif
    return 0;
}

/**
 * @brief WiFi low-rate health check
 * @note Silent mode has no periodic send, so this probe is needed for hot-plug reconnect.
 */
void wifi_health_check_task(void)
{
    // Disabled: INT low and UDP send-now can be normal while WiFi-SPI is connected.
}
/**
 * @brief WiFi模块初始化并连接，允许启动阶段按 P20_2 跳过。
 */
uint8 wifi_init_with_skip(uint8 allow_skip)
{
    uint8 wifi_spi_test_buffer[] = "Wheel-Leg Robot Online!\r\n";
    uint8 retry_count = 0;
    uint8 result = 0;
    uint8 max_retry = allow_skip ? WIFI_BOOT_MAX_RETRY : WIFI_MAX_RETRY;

    fifo_init(&wifi_data_fifo, FIFO_DATA_8BIT, wifi_fifo_buffer, sizeof(wifi_fifo_buffer));
    wifi_is_connected = 0;
    wifi_run_without_connection = 0;
    wifi_reconnect_enabled = 0;
    wifi_boot_state = WIFI_BOOT_STATE_NONE;
    wifi_set_boot_abort_hook(allow_skip);

    while(1)
    {
        screen_boot_show_wifi_attempt((uint8)(retry_count + 1), max_retry);
        if (wifi_boot_skip_requested_or_pressed(allow_skip))
        {
            printf("\r\n WiFi skipped by user before init.\r\n");
            wifi_boot_state = WIFI_BOOT_STATE_SKIPPED;
            wifi_skip_connection();
            goto wifi_init_exit;
        }

        if(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST) == 0)
        {
            break;
        }

        if (wifi_boot_skip_requested_or_pressed(allow_skip))
        {
            printf("\r\n WiFi skipped by user during init.\r\n");
            wifi_boot_state = WIFI_BOOT_STATE_SKIPPED;
            wifi_skip_connection();
            goto wifi_init_exit;
        }

        retry_count++;
        printf("\r\n connect wifi failed. retry: %d \r\n", retry_count);

        if(retry_count >= max_retry)
        {
            printf("\r\n WiFi Init Timeout! Run without WiFi. \r\n");
            wifi_boot_state = WIFI_BOOT_STATE_FAILED;
            wifi_skip_connection();
            goto wifi_init_exit;
        }
        if (wifi_wait_retry_or_skip(allow_skip, 100))
        {
            printf("\r\n WiFi skipped by user.\r\n");
            wifi_boot_state = WIFI_BOOT_STATE_SKIPPED;
            goto wifi_init_exit;
        }
    }

    printf("\r\n module version:%s", wifi_spi_version);
    printf("\r\n module mac    :%s", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s", wifi_spi_ip_addr_port);

    retry_count = 0;
    if(0 == WIFI_SPI_AUTO_CONNECT)
    {
        while(1)
        {
            screen_boot_show_wifi_attempt((uint8)(retry_count + 1), max_retry);
            if (wifi_boot_skip_requested_or_pressed(allow_skip))
            {
                printf("\r\n WiFi socket skipped by user before connect.\r\n");
                wifi_boot_state = WIFI_BOOT_STATE_SKIPPED;
                wifi_skip_connection();
                goto wifi_init_exit;
            }

            if(wifi_spi_socket_connect(WIFI_PROTOCOL_STR, WIFI_TARGET_IP, WIFI_TARGET_PORT, WIFI_LOCAL_PORT) == 0)
            {
                break;
            }

            if (wifi_boot_skip_requested_or_pressed(allow_skip))
            {
                printf("\r\n WiFi socket skipped by user during connect.\r\n");
                wifi_boot_state = WIFI_BOOT_STATE_SKIPPED;
                wifi_skip_connection();
                goto wifi_init_exit;
            }

            retry_count++;
            printf("\r\n Connect %s Servers error, try again.", WIFI_PROTOCOL_STR);

            if(retry_count >= max_retry)
            {
                printf("\r\n %s Socket Timeout! Run without WiFi.\r\n", WIFI_PROTOCOL_STR);
                wifi_boot_state = WIFI_BOOT_STATE_FAILED;
                wifi_skip_connection();
                goto wifi_init_exit;
            }
            if (wifi_wait_retry_or_skip(allow_skip, 100))
            {
                printf("\r\n WiFi socket skipped by user.\r\n");
                wifi_boot_state = WIFI_BOOT_STATE_SKIPPED;
                goto wifi_init_exit;
            }
        }
    }

    if(!wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer) - 1))
    {
        printf("\r\n wifi init & send success.\r\n");
        wifi_error_count = 0;
        wifi_is_connected = 1;
        wifi_run_without_connection = 0;
        wifi_reconnect_enabled = 0;
        wifi_boot_state = WIFI_BOOT_STATE_CONNECTED;
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    }
    else
    {
        wifi_boot_state = WIFI_BOOT_STATE_FAILED;
        wifi_skip_connection();
    }

wifi_init_exit:
    if ((wifi_boot_state == WIFI_BOOT_STATE_NONE) && (wifi_is_connected == 0))
    {
        wifi_boot_state = WIFI_BOOT_STATE_FAILED;
    }
    result = wifi_is_connected;
    wifi_set_boot_abort_hook(0);
    return result;
}
void wifi_init(void)
{
    (void)wifi_init_with_skip(0);
}

/**
 * @brief WiFi数据处理轮询 (防粘包拼图版)
 */
void wifi_process_loop(void)
{
    if (wifi_is_connected == 0)
    {
        return;
    }

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
        VOFA_Set_Param_Rx_Source(VOFA_PARAM_RX_SRC_WIFI);
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
    if (wifi_reconnect_enabled == 0) return;

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
            wifi_run_without_connection = 0;
            wifi_reconnect_enabled = 0;

            wifi_boot_state = WIFI_BOOT_STATE_CONNECTED;

            // ========================================================
            // 【新增体验优化】重连成功后，主动把当前 RAM 里的真实参数推给电脑！
            // 坚决不读 Flash，而是把刚调好的参数同步给 VOFA+
            // ========================================================
            IPC_Pull_Status_To_CoreB();
            VOFA_Send_Params_To_Wifi(core_a_status.act_params);
            printf("[WIFI] Current RAM Params Synced to PC!\r\n");
            // ========================================================

            return;
        }
    }

    // 重连失败，进入长冷却期 (大约半秒到1秒，取决于你的主循环速度)
    printf("\r\n[WIFI] Reconnect Failed. Cooldown for next attempt...\r\n");
    wifi_boot_state = WIFI_BOOT_STATE_FAILED;
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
                wifi_wave_send_selected_text();
                break;

            case WIFI_MODE_IMAGE:
                WIFI_Send_Buffer_Checked((uint8*)"IMAGE MODE NOW\r\n", 16, 1);
                break;
            case WIFI_MODE_LOG:
                WIFI_Send_Buffer_Checked((uint8*)"System Running OK...\r\n", 22, 1);
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
        WIFI_Send_Buffer_Checked((uint8*)log_buf, strlen(log_buf), 1);
    }
}




