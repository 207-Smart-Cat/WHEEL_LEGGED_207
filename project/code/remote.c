#include "remote.h"
#include "imu.h"
#include "param.h"
#include "runtime_status.h"
#include "jump_control.h"
#include "vehicle_supervisor.h"
#include "navigation_tracking.h"
#include "navigation_action.h"
extern IMU_t IMU_data;            // IMU数据
extern float target_angle;        // 目标角度
extern float target_velocity;
#define REMOTE_CH3_EMERGENCY_THRESHOLD 1000      // CH3: 192/1792，两档切换，超过该阈值视为高位
#define REMOTE_CH3_EMERGENCY_RECOVER_FRAMES 300
#define REMOTE_CH6_JUMP_THRESHOLD 1000           // CH6: 192/1792，两档切换，超过该阈值视为高位
#define REMOTE_CH6_JUMP_ARM_FRAMES 5
#define REMOTE_CH4_MID_THRESHOLD 800
#define REMOTE_CH4_HIGH_THRESHOLD 1400
// ------------------- 内部结构体定义 -------------------
// 将数据结构体定义在 .c 文件中，实现对外隐藏（封装）
typedef struct
{
    Remote_Status status;                // 连接状态
    int32_t channel[REMOTE_CHANNEL_NUM]; // 通道数据缓存
} Remote_CtrlData_t;

// 实例化一个私有的遥控器数据对象
static Remote_CtrlData_t s_RemoteData = {
    .status = REMOTE_DISCONNECTED,
    .channel = {REMOTE_SAFE_VALUE_CH1, REMOTE_SAFE_VALUE_CH2, REMOTE_SAFE_VALUE_OTHER,
                REMOTE_SAFE_VALUE_OTHER, REMOTE_SAFE_VALUE_OTHER, REMOTE_SAFE_VALUE_OTHER}};

static bool remote_drive_active = false;
static uint8 remote_ch3_initialized = 0;
static uint8 remote_ch3_last_high = 0;
static uint16 remote_ch3_emergency_frames = 0;
static uint8 remote_ch6_initialized = 0;
static uint8 remote_ch6_last_high = 0;
static uint8 remote_ch6_stable_count = 0;
static uint8 remote_jump_armed = 0;
static uint8 remote_record_initialized = 0;
static uint8 remote_record_last_high = 0;
static uint8 remote_record_stable_count = 0;
static uint8 remote_record_armed = 0;

float remote_dbg_connected = 0.0f;
float remote_dbg_ch1 = REMOTE_SAFE_VALUE_CH1;
float remote_dbg_ch2 = REMOTE_SAFE_VALUE_CH2;
float remote_dbg_ch3 = REMOTE_SAFE_VALUE_OTHER;
float remote_dbg_ch4 = REMOTE_SAFE_VALUE_OTHER;
float remote_dbg_ch5 = REMOTE_SAFE_VALUE_OTHER;
float remote_dbg_ch6 = REMOTE_SAFE_VALUE_OTHER;
float remote_dbg_frame_count = 0.0f;
float remote_dbg_raw_state = 0.0f;
float remote_dbg_uart4_isr_count = 0.0f;

static void Remote_CheckEmergencyStop(void);
static void Remote_ResetEmergencyTrigger(void);
static void Remote_CheckJumpTrigger(uint8 remote_drive_enabled);
static void Remote_ResetJumpTrigger(void);
static uint8 Remote_Is_Navi_Record_Mode(void);
static void Remote_CheckRecordTrigger(uint8 remote_drive_enabled);
static void Remote_ResetRecordTrigger(void);
static WayPoint_Type Remote_GetRecordPointType(void);

static void Remote_UpdateDebugValues(void)
{
    remote_dbg_connected = (s_RemoteData.status == REMOTE_CONNECTED) ? 1.0f : 0.0f;
    remote_dbg_ch1 = (float)s_RemoteData.channel[0];
    remote_dbg_ch2 = (float)s_RemoteData.channel[1];
    remote_dbg_ch3 = (float)s_RemoteData.channel[2];
    remote_dbg_ch4 = (float)s_RemoteData.channel[3];
    remote_dbg_ch5 = (float)s_RemoteData.channel[4];
    remote_dbg_ch6 = (float)s_RemoteData.channel[5];
    remote_dbg_raw_state = (float)uart_receiver.state;
}
// ------------------- 接口实现 -------------------

void Remote_Init(void)
{
    // 调用逐飞库底层的 sbus 接收机初始化
    uart_receiver_init();

    // 初始化本地缓存为安全状态
    s_RemoteData.status = REMOTE_DISCONNECTED;
    s_RemoteData.channel[0] = REMOTE_SAFE_VALUE_CH1;
    s_RemoteData.channel[1] = REMOTE_SAFE_VALUE_CH2;
    s_RemoteData.channel[2] = REMOTE_SAFE_VALUE_OTHER;
    s_RemoteData.channel[3] = REMOTE_SAFE_VALUE_OTHER;
    s_RemoteData.channel[4] = REMOTE_SAFE_VALUE_OTHER;
    s_RemoteData.channel[5] = REMOTE_SAFE_VALUE_OTHER;
    Remote_UpdateDebugValues();
    Remote_ResetJumpTrigger();
    Remote_ResetRecordTrigger();
}

void Remote_Update(void)
{
    // 检查底层库是否完成了一帧数据的解析
    if (1 == uart_receiver.finsh_flag)
    {
        remote_dbg_frame_count += 1.0f;
        // 判断遥控器状态
        if (1 == uart_receiver.state)
        {
            s_RemoteData.status = REMOTE_CONNECTED;

            // 安全拷贝底层通道数据到我们的私有结构体中
            for (int i = 0; i < REMOTE_CHANNEL_NUM; i++)
            {
                s_RemoteData.channel[i] = uart_receiver.channel[i];
            }

            // 调试打印 (可视情况注释掉，避免拖慢主循环运行速度)
            // printf("Remote Connected. CH1-CH6: %d %d %d %d %d %d\r\n",
            //        s_RemoteData.channel[0], s_RemoteData.channel[1], s_RemoteData.channel[2],
            //        s_RemoteData.channel[3], s_RemoteData.channel[4], s_RemoteData.channel[5]);
        }
        else
        {
            // 遥控器失控处理：将状态置为断开，并将所有通道数据赋为安全值
            s_RemoteData.status = REMOTE_DISCONNECTED;
            s_RemoteData.channel[0] = REMOTE_SAFE_VALUE_CH1;
            s_RemoteData.channel[1] = REMOTE_SAFE_VALUE_CH2;
            s_RemoteData.channel[2] = REMOTE_SAFE_VALUE_OTHER;
            s_RemoteData.channel[3] = REMOTE_SAFE_VALUE_OTHER;
            s_RemoteData.channel[4] = REMOTE_SAFE_VALUE_OTHER;
            s_RemoteData.channel[5] = REMOTE_SAFE_VALUE_OTHER;
            // Keep ISR path non-blocking. Do not print here.
        }

        Remote_UpdateDebugValues();

        // 必须清零完成标志位，等待下一次接收
        uart_receiver.finsh_flag = 0;
    }
}

Remote_Status Remote_GetStatus(void)
{
    return s_RemoteData.status;
}

int32_t Remote_GetChannelData(uint8_t ch_index) // 通道序号，用于外部访问
{
    // 安全性：防止数组越界访问
    // 注意：外部调用的通道号习惯是 1~6，所以映射到数组需要减 1
    if (ch_index >= 1 && ch_index <= REMOTE_CHANNEL_NUM)
    {
        // 安全性：如果遥控器断开连接，强制返回安全值，避免小车获取到最后的错误缓存而暴走
        if (s_RemoteData.status == REMOTE_CONNECTED)
        {
            return s_RemoteData.channel[ch_index - 1];
        }
    }

    // 越界或失控时，返回安全值
    switch (ch_index)
    {
    case 1:
        return REMOTE_SAFE_VALUE_CH1;
        break;
    case 2:
        return REMOTE_SAFE_VALUE_CH2;
        break;
    case 3:
        return REMOTE_SAFE_VALUE_OTHER;
        break;
    case 4:
        return REMOTE_SAFE_VALUE_OTHER;
        break;
    case 5:
        return REMOTE_SAFE_VALUE_OTHER;
        break;
    case 6:
        return REMOTE_SAFE_VALUE_OTHER;
        break;
    default:
        return REMOTE_SAFE_VALUE_OTHER;
    }
}

static void Remote_ResetEmergencyTrigger(void)
{
    remote_ch3_initialized = 0;
    remote_ch3_last_high = 0;
    remote_ch3_emergency_frames = 0;
}

static void Remote_CheckEmergencyStop(void)
{
    uint8 ch3_high = (Remote_GetChannelData(3) > REMOTE_CH3_EMERGENCY_THRESHOLD) ? 1 : 0;

    if (!remote_ch3_initialized)
    {
        remote_ch3_last_high = ch3_high;
        remote_ch3_initialized = 1;
        remote_ch3_emergency_frames = 0;
        return;
    }

    if (Vehicle_Is_Emergency_Stop())
    {
        if (remote_ch3_emergency_frames < REMOTE_CH3_EMERGENCY_RECOVER_FRAMES)
        {
            remote_ch3_emergency_frames++;
        }
    }
    else
    {
        remote_ch3_emergency_frames = 0;
    }

    if (ch3_high != remote_ch3_last_high)
    {
        remote_ch3_last_high = ch3_high;
        if (Vehicle_Is_Emergency_Stop())
        {
            if (remote_ch3_emergency_frames >= REMOTE_CH3_EMERGENCY_RECOVER_FRAMES)
            {
                Vehicle_Emergency_Recover(VEHICLE_EVENT_SOURCE_REMOTE);
                remote_ch3_emergency_frames = 0;
            }
        }
        else
        {
            Vehicle_Emergency_Stop(VEHICLE_EVENT_SOURCE_REMOTE);
            remote_ch3_emergency_frames = 0;
        }
    }
}
static void Remote_ResetJumpTrigger(void)
{
    remote_ch6_initialized = 0;
    remote_ch6_last_high = 0;
    remote_ch6_stable_count = 0;
    remote_jump_armed = 0;
    jump_set_trigger_block_reason(JUMP_BLOCK_NOT_ARMED);
}

static uint8 Remote_Is_Navi_Record_Mode(void)
{
    return ((uint8)vofa_mode_driver == 2 || navi_ctrl.navi_mode_driver == 2) ? 1 : 0;
}

static void Remote_ResetRecordTrigger(void)
{
    remote_record_initialized = 0;
    remote_record_last_high = 0;
    remote_record_stable_count = 0;
    remote_record_armed = 0;
}

static WayPoint_Type Remote_GetRecordPointType(void)
{
    uint8 mode = Runtime_Get_Vehicle_Mode();

    if (mode == VEHICLE_MODE_COURSE_2)
    {
        int32_t ch4 = Remote_GetChannelData(4);

        if (ch4 > REMOTE_CH4_HIGH_THRESHOLD)
        {
            return WP_TYPE_JUMP;
            // return WP_TYPE_BUMP; // 颠簸路段打点：与上一行二选一，切换注释即可
        }
        if (ch4 > REMOTE_CH4_MID_THRESHOLD)
        {
            return WP_TYPE_MINE_SWEEP;
        }
    }

    // Course 3: CH4 low/mid/high records normal/bridge/stair waypoints.
    // CH6 still triggers the record, and the navigation layer creates HOME first.
    if (mode == VEHICLE_MODE_COURSE_3)
    {
        int32_t ch4 = Remote_GetChannelData(4);

        if (ch4 < REMOTE_CH4_MID_THRESHOLD)
        {
            return WP_TYPE_NORMAL;
        }
        if (ch4 < REMOTE_CH4_HIGH_THRESHOLD)
        {
            return WP_TYPE_BRIDGE;
        }
        return WP_TYPE_JUMP;
        // return WP_TYPE_BUMP; // 颠簸路段打点：与上一行二选一，切换注释即可
    }

    return WP_TYPE_NORMAL;
}

//static WayPoint_Type Remote_GetRecordPointType(void)
//{
//    // 读取 CH4 (旋钮/多段开关) 通道数据
//    // 遥控器通道下限一般在 300 左右，上限在 1700 左右
//    int32_t ch4_val = Remote_GetChannelData(4);
//    
//    // 将通道值域分割为 6 段，对应 6 种类型
//    if      (ch4_val < 500)  return WP_TYPE_NORMAL;      // 普通循迹点 (极低位)
//    else if (ch4_val < 800)  return WP_TYPE_MINE_SWEEP    ;   // 定点排雷    (低位)
//    else if (ch4_val < 1100) return WP_TYPE_CONE_CONE;  //   绕圆锥桶(中低位)
//    else if (ch4_val < 1400) return WP_TYPE_BRIDGE;      // 单边桥     (中高位)
//    else if (ch4_val < 1700) return WP_TYPE_JUMP;        // 跳跃台阶   (高位)
//    else                     return WP_TYPE_STOP;        // 终点停车   (极高位)
//}

static void Remote_CheckRecordTrigger(uint8 remote_drive_enabled)
{
    uint8 ch6_high = (Remote_GetChannelData(6) > REMOTE_CH6_JUMP_THRESHOLD) ? 1 : 0;

    if (!remote_record_initialized)
    {
        remote_record_last_high = ch6_high;
        remote_record_initialized = 1;
        return;
    }

    if (ch6_high == remote_record_last_high)
    {
        if (remote_record_stable_count < REMOTE_CH6_JUMP_ARM_FRAMES)
        {
            remote_record_stable_count++;
        }
        if (remote_record_stable_count >= REMOTE_CH6_JUMP_ARM_FRAMES)
        {
            remote_record_armed = 1;
        }
        return;
    }

    if (remote_drive_enabled && remote_record_armed)
    {
        wifi_remote_type = Remote_GetRecordPointType();
        wifi_in_action = 0.0f;
        vofa_trigger_record = 2.0f;
        navi_ctrl.trigger_record = 0;
        remote_record_armed = 0;
    }

    remote_record_last_high = ch6_high;
    remote_record_stable_count = 0;
}
static void Remote_CheckJumpTrigger(uint8 remote_drive_enabled)
{
    uint8 ch6_high = (Remote_GetChannelData(6) > REMOTE_CH6_JUMP_THRESHOLD) ? 1 : 0;

    if (!remote_ch6_initialized)
    {
        remote_ch6_last_high = ch6_high;
        remote_ch6_initialized = 1;
        jump_set_trigger_block_reason(JUMP_BLOCK_NOT_ARMED);
        return;
    }

    if (jump_is_active() || Navi_Action_Remote_Test_Active())
    {
        remote_ch6_last_high = ch6_high;
        jump_set_trigger_block_reason(JUMP_BLOCK_BUSY);
        return;
    }

    if (ch6_high == remote_ch6_last_high)
    {
        if (remote_ch6_stable_count < REMOTE_CH6_JUMP_ARM_FRAMES)
        {
            remote_ch6_stable_count++;
        }
        if (remote_ch6_stable_count >= REMOTE_CH6_JUMP_ARM_FRAMES)
        {
            remote_jump_armed = 1;
        }
    }
    else
    {
        if (remote_drive_enabled && remote_jump_armed)
        {
            int32_t ch4 = Remote_GetChannelData(4);

            /* ==================== LEGACY CH6 JUMP PATH - DISABLED ====================
             * Original logic: CH6 triggered the jump_control.c manual jump FSM.
             * Kept for rollback; do not delete.
             */
            // (void)jump_start();

            /* ==================== NEW CH6 NAVIGATION JUMP PATH ==================== */
            Runtime_Set_Module_Enabled(RUNTIME_MODULE_NAVIGATION, 1U);
            if (ch4 < REMOTE_CH4_MID_THRESHOLD)
            {
                Navi_Action_Request_Remote_Jump();
            }
            else if (ch4 > REMOTE_CH4_HIGH_THRESHOLD)
            {
                Navi_Action_Request_Remote_Bump();
            }
            else
            {
                /* CH4 中位是测试安全空挡。 */
            }
            /* ==================== END NEW CH6 NAVIGATION JUMP PATH ==================== */
            remote_jump_armed = 0;
        }
        else if (!remote_drive_enabled)
        {
            jump_set_trigger_block_reason(JUMP_BLOCK_REMOTE_STANDBY);
        }
        else
        {
            jump_set_trigger_block_reason(JUMP_BLOCK_NOT_ARMED);
        }

        remote_ch6_last_high = ch6_high;
        remote_ch6_stable_count = 0;
        return;
    }

    if (!remote_drive_enabled)
    {
        jump_set_trigger_block_reason(JUMP_BLOCK_REMOTE_STANDBY);
    }
    else if (!remote_jump_armed)
    {
        jump_set_trigger_block_reason(JUMP_BLOCK_NOT_ARMED);
    }
    else
    {
        jump_set_trigger_block_reason(JUMP_BLOCK_NO_EDGE);
    }
}

void Remote_control_callback(void)
{
    float yaw_stick;

    Remote_Update();
    if (!Runtime_Is_Module_Enabled(RUNTIME_MODULE_REMOTE))
    {
        Runtime_Set_Remote_Reason(RUNTIME_REASON_REMOTE_OFF);
        Remote_ResetJumpTrigger();
        Remote_ResetRecordTrigger();
        Remote_ResetEmergencyTrigger();
        jump_set_trigger_block_reason(JUMP_BLOCK_REMOTE_OFF);
        if (remote_drive_active)
        {
            remote_drive_active = false;
            target_velocity = 0.0f;
        }
        return;
    }

    if (Remote_GetStatus() == REMOTE_CONNECTED)
    {
        uint8 remote_drive_enabled = (Remote_GetChannelData(5) > 1000) ? 1 : 0;
        Remote_CheckEmergencyStop();
        if (Vehicle_Is_Emergency_Stop())
        {
            target_velocity = 0.0f;
            remote_drive_active = false;
            return;
        }
        if (Remote_Is_Navi_Record_Mode())
        {
            Remote_ResetJumpTrigger();
            Remote_CheckRecordTrigger(remote_drive_enabled);
        }
        else
        {
            Remote_ResetRecordTrigger();
            Remote_CheckJumpTrigger(remote_drive_enabled);
        }

        /* 遥控动作由 5 ms 导航任务接管，禁止普通摇杆覆盖动作命令。 */
        if (Navi_Action_Remote_Test_Active())
        {
            return;
        }

        if (remote_drive_enabled)
        {
            Runtime_Set_Remote_Reason(RUNTIME_REASON_NORMAL);
            if (!remote_drive_active)
            {
                remote_drive_active = true;
            }

            yaw_stick = (Remote_GetChannelData(1) - REMOTE_SAFE_VALUE_CH1) / 332.0f;
            if (yaw_stick > 0.05f || yaw_stick < -0.05f)
            {
                target_angle += yaw_stick * 1.0f;
                while (target_angle > 180.0f)  target_angle -= 360.0f;
                while (target_angle < -180.0f) target_angle += 360.0f;
            }

            target_velocity = (Remote_GetChannelData(2) - REMOTE_SAFE_VALUE_CH2) / 689.0f * 800.0f;
        }
        else
        {
            Runtime_Set_Remote_Reason(RUNTIME_REASON_REMOTE_STANDBY);
            if (remote_drive_active)
            {
                remote_drive_active = false;
                target_velocity = 0.0f;
            }
        }
    }
    else
    {
        Runtime_Set_Remote_Reason(RUNTIME_REASON_REMOTE_LOST);
        Remote_ResetJumpTrigger();
        Remote_ResetRecordTrigger();
        Remote_ResetEmergencyTrigger();
        jump_set_trigger_block_reason(JUMP_BLOCK_REMOTE_LOST);
        if (remote_drive_active)
        {
            remote_drive_active = false;
            target_velocity = 0.0f;
        }
    }
}

