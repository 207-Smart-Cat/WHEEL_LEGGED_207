#include "vofa_protocol.h"
#include "param.h"
#include "small_driver_uart_control.h"
#include "wifi.h"
#include "ipc_shared_data.h" // 必须包含 IPC 头文件
#include "runtime_status.h"
#include "jump_control.h"
#include "vehicle_supervisor.h"

// 引入定义在 wifi.c 中的全局状态变量
extern wifi_mode_t current_wifi_mode;
extern uint8 channel_show[5];

#define VOFA_PARAM_FRAME_BUF_SIZE   (256)
#define VOFA_PARAM_MAX_COUNT        IPC_PARAM_MAX_COUNT    // 受 256 字节帧缓冲和 64-bit update_mask 限制，最多建议 63 个参数

typedef char vofa_param_count_must_not_exceed_63[(PARAM_COUNT <= VOFA_PARAM_MAX_COUNT) ? 1 : -1];
typedef char vofa_param_frame_buffer_must_fit[(PARAM_COUNT * sizeof(float) + 3 <= VOFA_PARAM_FRAME_BUF_SIZE) ? 1 : -1];
static uint8 VOFA_Text_Is_Space(uint8 ch)
{
    return (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') ? 1 : 0;
}
static uint8 VOFA_Text_To_Upper(uint8 ch)
{
    if (ch >= 'a' && ch <= 'z')
    {
        return (uint8)(ch - 32);
    }
    return ch;
}

static uint8 VOFA_Text_Match_Wave(const uint8 *rx_buffer, uint32 data_length, uint32 *pos)
{
    uint32 i = 0;
    while (i < data_length && VOFA_Text_Is_Space(rx_buffer[i]))
    {
        i++;
    }

    if ((i + 4) > data_length)
    {
        return 0;
    }

    if (VOFA_Text_To_Upper(rx_buffer[i]) != 'W' || VOFA_Text_To_Upper(rx_buffer[i + 1]) != 'A' || VOFA_Text_To_Upper(rx_buffer[i + 2]) != 'V' || VOFA_Text_To_Upper(rx_buffer[i + 3]) != 'E')
    {
        return 0;
    }

    *pos = i + 4;
    return 1;
}

static uint8 VOFA_Parse_Wave_Text_Command(uint8 *rx_buffer, uint32 data_length)
{
    uint32 pos;
    uint8 ids[WIFI_WAVE_MAX_SELECTED + 1];
    uint8 count = 0;

    if (!VOFA_Text_Match_Wave(rx_buffer, data_length, &pos))
    {
        return 0;
    }

    while (pos < data_length && VOFA_Text_Is_Space(rx_buffer[pos]))
    {
        pos++;
    }

    if (pos < data_length && rx_buffer[pos] == '?')
    {
        wifi_wave_send_var_map();
        return 1;
    }

    if ((pos + 3) <= data_length && VOFA_Text_To_Upper(rx_buffer[pos]) == 'O' && VOFA_Text_To_Upper(rx_buffer[pos + 1]) == 'F' && VOFA_Text_To_Upper(rx_buffer[pos + 2]) == 'F')
    {
        current_wifi_mode = WIFI_MODE_SILENT;
        LOG_Printf("[WAVE] OFF, enter silent mode\r\n");
        return 1;
    }

    while (pos < data_length)
    {
        uint16 value = 0;
        uint8 has_digit = 0;

        while (pos < data_length && (VOFA_Text_Is_Space(rx_buffer[pos]) || rx_buffer[pos] == ','))
        {
            pos++;
        }

        if (pos >= data_length || rx_buffer[pos] == '\0')
        {
            break;
        }

        if (rx_buffer[pos] == '\r' || rx_buffer[pos] == '\n')
        {
            break;
        }

        while (pos < data_length && rx_buffer[pos] >= '0' && rx_buffer[pos] <= '9')
        {
            has_digit = 1;
            value = (uint16)(value * 10 + (rx_buffer[pos] - '0'));
            pos++;
        }

        if (!has_digit)
        {
            LOG_Printf("[WAVE] invalid command. Use WAVE? for help\r\n");
            return 1;
        }

        if (count >= WIFI_WAVE_MAX_SELECTED)
        {
            LOG_Printf("[WAVE] too many vars, max %d\r\n", WIFI_WAVE_MAX_SELECTED);
            return 1;
        }

        ids[count++] = (uint8)value;
    }

    if (count == 0)
    {
        wifi_wave_send_var_map();
        return 1;
    }

    count = wifi_wave_set_selected_ids(ids, count);
    if (count == 0)
    {
        LOG_Printf("[WAVE] invalid id. Use WAVE? for map\r\n");
        return 1;
    }

    LOG_Printf("[WAVE] selected %d vars, enter wave mode\r\n", count);
    return 1;
}

typedef union {
    float f_val;
    uint8 b_val[4];
} FloatConverter_t;



static uint8 g_param_log_detail_enable = 0;
static vofa_param_rx_source_t g_param_rx_source = VOFA_PARAM_RX_SRC_UART;

static const char *VOFA_Param_Rx_Source_Name(vofa_param_rx_source_t source)
{
    switch (source)
    {
        case VOFA_PARAM_RX_SRC_WIFI:    return "WIFI";
        case VOFA_PARAM_RX_SRC_SCREEN:  return "SCREEN";
        case VOFA_PARAM_RX_SRC_FLASH:   return "FLASH";
        case VOFA_PARAM_RX_SRC_DEFAULT: return "DEFAULT";
        case VOFA_PARAM_RX_SRC_UART:
        default:                        return "UART";
    }
}

void VOFA_Set_Param_Rx_Source(vofa_param_rx_source_t source)
{
    g_param_rx_source = source;
}

void VOFA_Set_Param_Log_Detail(uint8 enable)
{
    g_param_log_detail_enable = (enable != 0) ? 1 : 0;
}

uint8 VOFA_Get_Param_Log_Detail(void)
{
    return g_param_log_detail_enable;
}

void VOFA_Log_Param_Update(uint8 param_id, const char *name, float value, const char *cmd)
{
    if (g_param_log_detail_enable)
    {
        LOG_Printf("[Core 1] Param '%s' set to %.4f        src=%s cmd=%s id=%d\r\n",
                   name, value, VOFA_Param_Rx_Source_Name(g_param_rx_source), cmd, param_id);
    }
    else
    {
        LOG_Printf("[Core 1] Param '%s' set to %.4f\r\n", name, value);
    }
}

void VOFA_Log_Param_Bulk(const char *message, const char *cmd, uint16 count)
{
    if (g_param_log_detail_enable)
    {
        LOG_Printf("%s        src=%s cmd=%s count=%d\r\n",
                   message, VOFA_Param_Rx_Source_Name(g_param_rx_source), cmd, count);
    }
    else
    {
        LOG_Printf("%s\r\n", message);
    }
}

void VOFA_Log_Param_Command(const char *message, const char *cmd)
{
    if (g_param_log_detail_enable)
    {
        LOG_Printf("%s        src=%s cmd=%s\r\n",
                   message, VOFA_Param_Rx_Source_Name(g_param_rx_source), cmd);
    }
    else
    {
        LOG_Printf("%s\r\n", message);
    }
}

static const char *VOFA_Runtime_Module_Name(runtime_module_t module)
{
    switch (module)
    {
        case RUNTIME_MODULE_MOTOR:        return "Motor";
        case RUNTIME_MODULE_BALANCE:      return "Balance";
        case RUNTIME_MODULE_SERVO:        return "Servo";
        case RUNTIME_MODULE_REMOTE:       return "Remote";
        case RUNTIME_MODULE_NAVIGATION:   return "Navigation";
        case RUNTIME_MODULE_DEBUG_OUTPUT: return "DebugOutput";
        default:                          return "Unknown";
    }
}

static const char *VOFA_Jump_State_Name(uint8 state)
{
    switch ((JumpState)state)
    {
        case JUMP_FREE:        return "FREE";
        case JUMP_PREPARE:     return "PREPARE";
        case JUMP_BURST:       return "BURST";
        case JUMP_AIR_RETRACT: return "AIR_RETRACT";
        case JUMP_EXE_BUFFER:  return "BUFFER";
        case JUMP_RECOVER:     return "RECOVER";
        default:               return "UNKNOWN";
    }
}

static const char *VOFA_Jump_Block_Name(uint8 reason)
{
    switch ((JumpTriggerBlockReason)reason)
    {
        case JUMP_BLOCK_NONE:           return "NONE";
        case JUMP_BLOCK_STARTED:        return "STARTED";
        case JUMP_BLOCK_BUSY:           return "BUSY";
        case JUMP_BLOCK_REMOTE_OFF:     return "REMOTE_OFF";
        case JUMP_BLOCK_REMOTE_LOST:    return "REMOTE_LOST";
        case JUMP_BLOCK_REMOTE_STANDBY: return "REMOTE_STANDBY";
        case JUMP_BLOCK_NOT_ARMED:      return "NOT_ARMED";
        case JUMP_BLOCK_NO_EDGE:        return "NO_EDGE";
        default:                        return "UNKNOWN";
    }
}

static void VOFA_Log_Runtime_Status(void)
{
    IPC_Pull_Status_To_CoreB();
    Runtime_Sync_From_IPC();
    LOG_Printf("[RUNTIME] mode=%d wifi=%d mask=0x%02X\r\n",
               Runtime_Get_Vehicle_Mode(), Runtime_Get_Wifi_Connected(), (unsigned int)g_runtime_status.module_enable_mask);
    LOG_Printf("[RUNTIME] reason motor=%s balance=%s servo=%s remote=%s\r\n",
               Runtime_Reason_Name((runtime_reason_t)core_a_status.motor_reason),
               Runtime_Reason_Name((runtime_reason_t)core_a_status.balance_reason),
               Runtime_Reason_Name((runtime_reason_t)core_a_status.servo_reason),
               Runtime_Reason_Name((runtime_reason_t)core_a_status.remote_reason));
    LOG_Printf("[RUNTIME] jump state=%s elapsed=%d trigger=%lu block=%s\r\n",
               VOFA_Jump_State_Name(jump_dbg_state),
               jump_dbg_elapsed_ms,
               jump_dbg_trigger_count,
               VOFA_Jump_Block_Name(jump_dbg_trigger_block_reason));
    for (uint8 module = 0; module < RUNTIME_MODULE_COUNT; module++)
    {
        LOG_Printf("[RUNTIME] id=%d %-12s %s\r\n",
                   module,
                   VOFA_Runtime_Module_Name((runtime_module_t)module),
                   Runtime_Is_Module_Enabled((runtime_module_t)module) ? "ON" : "OFF");
    }
}

static uint8 VOFA_Calc_Checksum(const uint8 *data, uint32 len)
{
    uint8 sum = 0;
    uint32 i;
    for (i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}


static uint32 VOFA_Build_Params_Frame(const float *params, uint8 *tx_buf, uint32 tx_buf_size)
{
    uint32 payload_len = PARAM_COUNT * sizeof(float);
    uint32 frame_len = payload_len + 3;

    if (params == NULL || tx_buf == NULL || tx_buf_size < frame_len)
    {
        return 0;
    }

    tx_buf[0] = 0xAA;
    tx_buf[1] = 0xC4;
    memcpy(&tx_buf[2], params, payload_len);
    tx_buf[2 + payload_len] = VOFA_Calc_Checksum(&tx_buf[2], payload_len);
    return frame_len;
}

uint8 VOFA_Send_Params_To_Wifi(const float *params)
{
    uint8 tx_buf[VOFA_PARAM_FRAME_BUF_SIZE];
    uint32 frame_len = VOFA_Build_Params_Frame(params, tx_buf, sizeof(tx_buf));

    if (frame_len == 0)
    {
        return 1;
    }

    return WIFI_Send_Buffer_Checked(tx_buf, frame_len, 1);
}

void VOFA_Save_Params_To_Flash(void)
{
    IPC_Save_Params_To_Flash();
}

void VOFA_Load_Params_From_Flash(void)
{
    IPC_Load_Params_From_Flash();
}

void VOFA_Upload_Params_To_UI(void)
{
    IPC_Pull_Status_To_CoreB();
    VOFA_Send_Params_To_Wifi(core_a_status.act_params);
}
/**
 * @brief 终极通用协议解析器
 */
void VOFA_Protocol_Parse(uint8 *rx_buffer, uint32 data_length)
{
    if(data_length == 0 || rx_buffer == NULL) return;

    if (VOFA_Parse_Wave_Text_Command(rx_buffer, data_length)) return;

    for(uint32 i = 0; i < data_length; i++)
    {
        // ========================================================
        // 协议 1: 模式切换 (AA EE) - 保持不变
        // ========================================================
        if((i + 2) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xEE)
        {
            uint8 target_mode = rx_buffer[i+2];
            if(target_mode <= 3)
            {
                if(target_mode == WIFI_MODE_WAVE)
                {
                    wifi_wave_enter_mode();
                    LOG_Printf("\r\n >>> Enter WAVE Mode <<< \r\n");
                }
                else current_wifi_mode = (wifi_mode_t)target_mode;
            }
            i += 2; continue;
        }

        // ========================================================
        // 协议 1.5: 显隐切换 (AA FF) - 保持不变
        // ========================================================
        else if((i + 2) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xFF)
        {
            uint8 ch = rx_buffer[i+2];
            if(ch >= 1 && ch <= 5) channel_show[ch - 1] ^= 1;
            i += 2; continue;
        }

        // ========================================================
        // 协议 1.6: 参数日志详细模式切换 (AA E1 00/01/02)
        // 00 简洁模式, 01 详细模式, 02 切换模式
        // ========================================================
        else if((i + 2) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xE1)
        {
            uint8 mode = rx_buffer[i + 2];
            if (mode == 0x00)
            {
                VOFA_Set_Param_Log_Detail(0);
            }
            else if (mode == 0x01)
            {
                VOFA_Set_Param_Log_Detail(1);
            }
            else if (mode == 0x02)
            {
                VOFA_Set_Param_Log_Detail(VOFA_Get_Param_Log_Detail() ? 0 : 1);
            }

            VOFA_Log_Param_Command(VOFA_Get_Param_Log_Detail() ? "[PARAM LOG] detail mode" : "[PARAM LOG] brief mode", "E1_LOG");
            i += 2; continue;
        }


        // ========================================================
        // 协议 1.7: 运行状态控制 (AA E2)
        // AA E2 00              查询全部运行状态
        // AA E2 01 id enable    设置模块开关，id=0..5，enable=0/1
        // AA E2 02 mode         设置小车模式，mode=0..3
        // AA E2 03 id           翻转模块开关，id=0..5
        // ========================================================
        else if((i + 2) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xE2)
        {
            uint8 cmd = rx_buffer[i + 2];

            if (cmd == 0x00)
            {
                VOFA_Log_Runtime_Status();
                i += 2; continue;
            }
            else if (cmd == 0x01 && (i + 4) < data_length)
            {
                uint8 module = rx_buffer[i + 3];
                uint8 enable = rx_buffer[i + 4] ? 1 : 0;
                if (module < RUNTIME_MODULE_COUNT)
                {
                    Runtime_Set_Module_Enabled((runtime_module_t)module, enable);
                    LOG_Printf("[RUNTIME] %s set to %s\r\n",
                               VOFA_Runtime_Module_Name((runtime_module_t)module),
                               enable ? "ON" : "OFF");
                }
                else
                {
                    LOG_Printf("[RUNTIME] invalid module id=%d\r\n", module);
                }
                i += 4; continue;
            }
            else if (cmd == 0x02 && (i + 3) < data_length)
            {
                uint8 mode = rx_buffer[i + 3];
                if (mode <= 3)
                {
                    Runtime_Set_Vehicle_Mode(mode);
                    LOG_Printf("[RUNTIME] vehicle mode set to %d\r\n", mode);
                }
                else
                {
                    LOG_Printf("[RUNTIME] invalid vehicle mode=%d\r\n", mode);
                }
                i += 3; continue;
            }
            else if (cmd == 0x03 && (i + 3) < data_length)
            {
                uint8 module = rx_buffer[i + 3];
                if (module < RUNTIME_MODULE_COUNT)
                {
                    Runtime_Toggle_Module((runtime_module_t)module);
                    LOG_Printf("[RUNTIME] %s toggled to %s\r\n",
                               VOFA_Runtime_Module_Name((runtime_module_t)module),
                               Runtime_Is_Module_Enabled((runtime_module_t)module) ? "ON" : "OFF");
                }
                else
                {
                    LOG_Printf("[RUNTIME] invalid module id=%d\r\n", module);
                }
                i += 3; continue;
            }

            LOG_Printf("[RUNTIME] invalid E2 command\r\n");
            i += 2; continue;
        }
        // ========================================================
        // 协议 2: 动作控制 (AA C1) - 保持不变
        // ========================================================
        else if((i + 5) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC1)
        {
            uint8 sum_check = 0;
            for(uint8 j = 0; j < 5; j++) sum_check += rx_buffer[i + j];
            if(sum_check == rx_buffer[i + 5])
            {
                uint8 cmd_id = rx_buffer[i + 2];
                if(cmd_id == 0x01)
                {
                    Vehicle_Emergency_Stop(VEHICLE_EVENT_SOURCE_WIFI);
                    LOG_Printf("\r\n [VOFA] KILL SWITCH! \r\n");
                }
                else if(cmd_id == 0x02) LOG_Printf("\r\n [VOFA] STAY STILL! \r\n");
                i += 5; continue;
            }
        }

        // ========================================================
        // 协议 3: VOFA+ 极速 HEX 调参 (AA C2)
        // 修改点：使用掩码机制，实现精准单项更新，不牵一发而动全身
        // ========================================================
        else if((i + 6) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC2)
        {
            uint8 param_id = rx_buffer[i + 2]; // VOFA 下发的是 1 到 PARAM_COUNT

            // 安全检查：如果 ID 超出了我们现有的参数总数，直接丢弃
            if (param_id == 0 || param_id > PARAM_COUNT) {
                i += 6; continue;
            }

            FloatConverter_t temp_float;
            temp_float.b_val[0] = rx_buffer[i + 3];
            temp_float.b_val[1] = rx_buffer[i + 4];
            temp_float.b_val[2] = rx_buffer[i + 5];
            temp_float.b_val[3] = rx_buffer[i + 6];

            // 把从 1 开始的 ID 转换为从 0 开始的数组下标
            uint8 index = param_id - 1;

            IPC_Request_Param_Update((ParamID_e)index, temp_float.f_val);

            VOFA_Log_Param_Update(param_id, g_param_names[index], temp_float.f_val, "C2_SINGLE");

            i += 6; continue;
        }
        // ========================================================
        // 协议 4: 系统级指令 - 一键保存参数到 Flash (AA C3)
        // 格式: AA C3 88 55
        // ========================================================
        else if((i + 3) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC3 &&
                rx_buffer[i+2] == 0x88 && rx_buffer[i+3] == 0x55)
        {
            VOFA_Log_Param_Command("\r\n[VOFA] Received SAVE TO FLASH command!", "C3_SAVE");

            SCB_CleanInvalidateDCache_by_Addr(&core_a_status, sizeof(core_a_status));

            if ( abs(core_a_status.left_pwm_duty) > 1000 || abs(core_a_status.right_pwm_duty) > 1000 )
            {
                LOG_Printf("[ERROR] Motor is running! PLEASE STOP CAR FIRST!\r\n");
            }
            else
            {
                LOG_Printf("Writing real params from Core A to Flash...\r\n");
                // 【修改这里】直接调用 ipc_shared_data 里的函数
                IPC_Save_Params_To_Flash();
                LOG_Printf("Save Complete!\r\n");
            }

            i += 3; continue;
        }
        // ========================================================
        // 协议 5: 批量下发所有参数 (AA C4)
        // 格式: AA C4 [PARAM_COUNT个float] [1字节校验和]
        // ========================================================
        else if((i + 2 + PARAM_COUNT * 4 + 1) <= data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC4)
        {
            // 1. 计算校验和 (仅校验 float 数据区)
            uint8 sum_check = 0;
            for(int j = 0; j < PARAM_COUNT * 4; j++) {
                sum_check += rx_buffer[i + 2 + j];
            }

            // 2. 校验通过，执行批量更新
            if(sum_check == rx_buffer[i + 2 + PARAM_COUNT * 4])
            {
                LOG_Printf("\r\n[VOFA] Received BULK PARAM UPDATE (AA C4)!\r\n");

                __disable_irq();
                // 神级操作：利用内存拷贝，直接把 PARAM_COUNT 个 float 灌入 Core B 的参数数组中
                memcpy(core_b_cmd.params, &rx_buffer[i + 2], PARAM_COUNT * sizeof(float));

                // 触发全量更新掩码 (0xFFFFFFFF 表示所有位都是 1)
                core_b_cmd.update_mask = IPC_Get_All_Param_Mask();
                core_b_cmd.param_update_flag = 1;

                SCB_CleanInvalidateDCache_by_Addr(&core_b_cmd, sizeof(core_b_cmd));
                __enable_irq();

                char bulk_msg[64];
                sprintf(bulk_msg, "[VOFA] All %d Parameters Updated Successfully!", PARAM_COUNT);
                VOFA_Log_Param_Bulk(bulk_msg, "C4_BULK", PARAM_COUNT);
            }
            else
            {
                LOG_Printf("[ERROR] Bulk Update Checksum Failed!\r\n");
            }

            i += (2 + PARAM_COUNT * 4 + 1); // 跳过整帧：帧头 + 参数区 + 校验和
            continue;
        }

        // ========================================================
        // 协议 6: 上位机请求读取当前参数 (AA C5)
        // 格式: AA C5 88 55
        // ========================================================
        else if((i + 3) < data_length && rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xC5 &&
                rx_buffer[i+2] == 0x88 && rx_buffer[i+3] == 0x55)
        {
            VOFA_Log_Param_Command("\r\n[VOFA] Received PARAM REQUEST (AA C5)! Sending...", "C5_READ");

            uint8 tx_buf[VOFA_PARAM_FRAME_BUF_SIZE];
            uint32 total_len;

            // 1. 获取最新真值
            IPC_Pull_Status_To_CoreB();

            // 2. 构建并发送二进制返回包 (AA C4)
            total_len = VOFA_Build_Params_Frame(core_a_status.act_params, tx_buf, sizeof(tx_buf));
            if (total_len > 0)
            {
                WIFI_Send_Buffer_Checked(tx_buf, total_len, 1);
            }

            // 3. 打印给人类看的清单（采用防对齐 Bug 宏）
            #define F_S(f) ((f) < 0 ? "-" : "")
            #define F_I(f) (int)((f) < 0 ? -(f) : (f))
            #define F_D(f) (int)((((f) < 0 ? -(f) : (f)) - (int)((f) < 0 ? -(f) : (f))) * 10000)

            LOG_Printf("\r\n============= CURRENT PARAMS =============\r\n");
            LOG_Printf(" Ang_P: %s%d.%04d | Ang_D: %s%d.%04d \r\n",
                   F_S(core_a_status.act_params[P_ANGLE_P]), F_I(core_a_status.act_params[P_ANGLE_P]), F_D(core_a_status.act_params[P_ANGLE_P]),
                   F_S(core_a_status.act_params[P_ANGLE_D]), F_I(core_a_status.act_params[P_ANGLE_D]), F_D(core_a_status.act_params[P_ANGLE_D]));

            LOG_Printf(" Spd_P: %s%d.%04d | Spd_I: %s%d.%04d | Spd_D: %s%d.%04d\r\n",
                   F_S(core_a_status.act_params[P_SPEED_P]), F_I(core_a_status.act_params[P_SPEED_P]), F_D(core_a_status.act_params[P_SPEED_P]),
                   F_S(core_a_status.act_params[P_SPEED_I]), F_I(core_a_status.act_params[P_SPEED_I]), F_D(core_a_status.act_params[P_SPEED_I]),
                   F_S(core_a_status.act_params[P_SPEED_D]), F_I(core_a_status.act_params[P_SPEED_D]), F_D(core_a_status.act_params[P_SPEED_D]));

            LOG_Printf(" Gyr_P: %s%d.%04d | Gyr_I: %s%d.%04d | Gyr_D: %s%d.%04d\r\n",
                   F_S(core_a_status.act_params[P_GYRO_P]), F_I(core_a_status.act_params[P_GYRO_P]), F_D(core_a_status.act_params[P_GYRO_P]),
                   F_S(core_a_status.act_params[P_GYRO_I]), F_I(core_a_status.act_params[P_GYRO_I]), F_D(core_a_status.act_params[P_GYRO_I]),
                   F_S(core_a_status.act_params[P_GYRO_D]), F_I(core_a_status.act_params[P_GYRO_D]), F_D(core_a_status.act_params[P_GYRO_D]));
            LOG_Printf(" Air_P: %s%d.%04d | Air_I: %s%d.%04d | Air_D: %s%d.%04d\r\n",
                   F_S(core_a_status.act_params[P_AIR_ROLL_P]), F_I(core_a_status.act_params[P_AIR_ROLL_P]), F_D(core_a_status.act_params[P_AIR_ROLL_P]),
                   F_S(core_a_status.act_params[P_AIR_ROLL_I]), F_I(core_a_status.act_params[P_AIR_ROLL_I]), F_D(core_a_status.act_params[P_AIR_ROLL_I]),
                   F_S(core_a_status.act_params[P_AIR_ROLL_D]), F_I(core_a_status.act_params[P_AIR_ROLL_D]), F_D(core_a_status.act_params[P_AIR_ROLL_D]));

            LOG_Printf(" Dir_P: %s%d.%04d | Dir_I: %s%d.%04d | Dir_D: %s%d.%04d\r\n",
                   F_S(core_a_status.act_params[P_DIR_P]), F_I(core_a_status.act_params[P_DIR_P]), F_D(core_a_status.act_params[P_DIR_P]),
                   F_S(core_a_status.act_params[P_DIR_I]), F_I(core_a_status.act_params[P_DIR_I]), F_D(core_a_status.act_params[P_DIR_I]),
                   F_S(core_a_status.act_params[P_DIR_D]), F_I(core_a_status.act_params[P_DIR_D]), F_D(core_a_status.act_params[P_DIR_D]));
            LOG_Printf(" Mag_Off: X %s%d.%04d | Y %s%d.%04d \r\n",
                   F_S(core_a_status.act_params[P_MAG_OFFSET_X]), F_I(core_a_status.act_params[P_MAG_OFFSET_X]), F_D(core_a_status.act_params[P_MAG_OFFSET_X]),
                   F_S(core_a_status.act_params[P_MAG_OFFSET_Y]), F_I(core_a_status.act_params[P_MAG_OFFSET_Y]), F_D(core_a_status.act_params[P_MAG_OFFSET_Y]));
            LOG_Printf(" Mag_Scl: X %s%d.%04d | Y %s%d.%04d \r\n",
                   F_S(core_a_status.act_params[P_MAG_SCALE_X]), F_I(core_a_status.act_params[P_MAG_SCALE_X]), F_D(core_a_status.act_params[P_MAG_SCALE_X]),
                   F_S(core_a_status.act_params[P_MAG_SCALE_Y]), F_I(core_a_status.act_params[P_MAG_SCALE_Y]), F_D(core_a_status.act_params[P_MAG_SCALE_Y]));
            LOG_Printf("=============================================\r\n");

            // 4. 分 4 段安全拼合打印 AA C4 参数包（防丢包/溢出）
            LOG_Printf("\r\n>>> VOFA+ COPY & PASTE COMMAND (AA C4) <<<\r\n");

            char hex_str[160];


            // 第 1 段：0 ~ 39 字节
            memset(hex_str, 0, sizeof(hex_str));
            for(int k = 0; k < 40 && k < (int)total_len; k++) sprintf(hex_str + strlen(hex_str), "%02X ", tx_buf[k]);
            LOG_Printf("%s", hex_str);

            // 第 2 段：40 ~ 79 字节
            memset(hex_str, 0, sizeof(hex_str));
            for(int k = 40; k < 80 && k < (int)total_len; k++) sprintf(hex_str + strlen(hex_str), "%02X ", tx_buf[k]);
            LOG_Printf("%s", hex_str);

            // 第 3 段：80 ~ 119 字节
            memset(hex_str, 0, sizeof(hex_str));
            for(int k = 80; k < 120 && k < (int)total_len; k++) sprintf(hex_str + strlen(hex_str), "%02X ", tx_buf[k]);
            LOG_Printf("%s", hex_str);

            // 第 4 段：120 ~ 末尾
            memset(hex_str, 0, sizeof(hex_str));
            for(int k = 120; k < (int)total_len; k++) sprintf(hex_str + strlen(hex_str), "%02X ", tx_buf[k]);
            LOG_Printf("%s\r\n", hex_str);

            LOG_Printf(">>> END OF COMMAND <<<\r\n\r\n");

            i += 3; continue;
        }
    }
}

// ====================================================================
// UART 初始化与后台处理函数（保持你原来的逻辑，无需修改）
// ====================================================================
#define UART_INDEX      (DEBUG_UART_INDEX)
#define UART_BAUDRATE   (115200)
#define UART_TX_PIN     (DEBUG_UART_TX_PIN)
#define UART_RX_PIN     (DEBUG_UART_RX_PIN)

static uint8 uart_get_data[256];
static uint8 fifo_get_data[256];
fifo_struct uart_data_fifo;

void VOFA_UART_Init(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, sizeof(uart_get_data));
    system_delay_ms(15);
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
    uart_rx_interrupt(UART_INDEX, 1);
}

void VOFA_UART_Process(void)
{
    uint32 fifo_data_count = fifo_used(&uart_data_fifo);
    if(fifo_data_count >= 3)
    {
        system_delay_ms(2);
        fifo_data_count = fifo_used(&uart_data_fifo);
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
        VOFA_Set_Param_Rx_Source(VOFA_PARAM_RX_SRC_UART);
        VOFA_Protocol_Parse(fifo_get_data, fifo_data_count);
    }
}


