#include "remote.h"
#include "kalman_rm.h"
#include "param.h"
extern Attitude_3D_Kalman filter; // 卡尔曼滤波器
extern IMU_t IMU_data;            // IMU数据
extern float target_angle;        // 目标角度
extern float target_velocity;
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
    .channel = {REMOTE_SAFE_VALUE_CH1, REMOTE_SAFE_VALUE_CH2, REMOTE_SAFE_VALUE_CHother,
                REMOTE_SAFE_VALUE_CHother, REMOTE_SAFE_VALUE_CHother, REMOTE_SAFE_VALUE_CHother}};

static bool remote_drive_active = false;

// ------------------- 接口实现 -------------------

void Remote_Init(void)
{
    // 调用逐飞库底层的 sbus 接收机初始化
    uart_receiver_init();

    // 初始化本地缓存为安全状态
    s_RemoteData.status = REMOTE_DISCONNECTED;
    s_RemoteData.channel[0] = REMOTE_SAFE_VALUE_CH1;
    s_RemoteData.channel[1] = REMOTE_SAFE_VALUE_CH2;
    s_RemoteData.channel[2] = REMOTE_SAFE_VALUE_CHother;
    s_RemoteData.channel[3] = REMOTE_SAFE_VALUE_CHother;
    s_RemoteData.channel[4] = REMOTE_SAFE_VALUE_CHother;
    s_RemoteData.channel[5] = REMOTE_SAFE_VALUE_CHother;
    target_angle = IMU_data.filter_result.yaw;
}

void Remote_Update(void)
{
    // 检查底层库是否完成了一帧数据的解析
    if (1 == uart_receiver.finsh_flag)
    {
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
            s_RemoteData.channel[2] = REMOTE_SAFE_VALUE_CHother;
            s_RemoteData.channel[3] = REMOTE_SAFE_VALUE_CHother;
            s_RemoteData.channel[4] = REMOTE_SAFE_VALUE_CHother;
            s_RemoteData.channel[5] = REMOTE_SAFE_VALUE_CHother;
            // Keep ISR path non-blocking. Do not print here.
        }

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
        return REMOTE_SAFE_VALUE_CHother;
        break;
    case 4:
        return REMOTE_SAFE_VALUE_CHother;
        break;
    case 5:
        return REMOTE_SAFE_VALUE_CHother;
        break;
    case 6:
        return REMOTE_SAFE_VALUE_CHother;
        break;
    default:
        return REMOTE_SAFE_VALUE_CHother;
    }
}

void Remote_control_callback(void)
{
    float yaw_stick;

    Remote_Update();

    if (Remote_GetStatus() == REMOTE_CONNECTED)
    {
        if (Remote_GetChannelData(5) > 1000)
        {
            if (!remote_drive_active)
            {
                remote_drive_active = true;
                target_angle = IMU_data.filter_result.yaw;
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
        else if (remote_drive_active)
        {
            remote_drive_active = false;
            target_velocity = 0.0f;
            target_angle = IMU_data.filter_result.yaw;
        }
    }
    else if (remote_drive_active)
    {
        remote_drive_active = false;
        target_velocity = 0.0f;
        target_angle = IMU_data.filter_result.yaw;
    }
}

