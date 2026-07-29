#ifndef _REMOTE_H_
#define _REMOTE_H_

#include "zf_common_headfile.h"
// 定义遥控器通道数量 (6个)
#define REMOTE_CHANNEL_NUM  6

// 遥控器通道标定值，来自实测 SBUS 解码结果。
// CH1 默认 888，CH2 默认 1000；CH3/5/6 为两档开关；CH4 为三档开关。
// 通道用途说明：
// CH1：方向调节，用于遥控接管时调整 target_angle。
// CH2：油门/速度调节，用于遥控接管时调整 target_velocity。
// CH3：科目三采集模式记录普通点，其他模式用于紧急制动/恢复。
// CH4：科目三采集模式选择单边桥/颠簸/台阶斜坡；科目二保持原类型选择。
// CH5：两档开关，用于选择遥控是否接管小车。
// CH6：两档开关；正常遥控时触发跳跃，导航打点模式下触发打点。
#define REMOTE_CH1_CENTER        888
#define REMOTE_CH2_CENTER        1000
#define REMOTE_SWITCH_LOW        192
#define REMOTE_SWITCH_MID        992
#define REMOTE_SWITCH_HIGH       1792
#define REMOTE_CH3_LOW           REMOTE_SWITCH_LOW
#define REMOTE_CH3_HIGH          REMOTE_SWITCH_HIGH
#define REMOTE_CH4_LOW           REMOTE_SWITCH_LOW
#define REMOTE_CH4_MID           REMOTE_SWITCH_MID
#define REMOTE_CH4_HIGH          REMOTE_SWITCH_HIGH
#define REMOTE_CH5_LOW           REMOTE_SWITCH_LOW
#define REMOTE_CH5_HIGH          REMOTE_SWITCH_HIGH
#define REMOTE_CH6_LOW           REMOTE_SWITCH_LOW
#define REMOTE_CH6_HIGH          REMOTE_SWITCH_HIGH

// 遥控器失控时的安全默认值。
#define REMOTE_SAFE_VALUE_CH1    REMOTE_CH1_CENTER
#define REMOTE_SAFE_VALUE_CH2    REMOTE_CH2_CENTER
#define REMOTE_SAFE_VALUE_OTHER  REMOTE_SWITCH_LOW
// 遥控器连接状态枚举
typedef enum {
    REMOTE_DISCONNECTED = 0, // 失控/断开
    REMOTE_CONNECTED    = 1  // 正常连接
} Remote_Status;

// ------------------- API 函数声明 -------------------

/**
 * @brief  遥控器模块初始化
 * @note   在 main 函数的硬件初始化阶段调用
 */
void Remote_Init(void);

/**
 * @brief  遥控器数据更新任务
 * @note   放在 main 函数的 while(1) 循环中持续调用
 */
void Remote_Update(void);

/**
 * @brief  获取遥控器连接状态
 * @retval 1:连接正常, 0:断开/失控
 */
Remote_Status Remote_GetStatus(void);

void Remote_control_callback(void);
int32_t Remote_GetChannelData(uint8_t ch_index);

extern float remote_dbg_connected;
extern float remote_dbg_ch1;
extern float remote_dbg_ch2;
extern float remote_dbg_ch3;
extern float remote_dbg_ch4;
extern float remote_dbg_ch5;
extern float remote_dbg_ch6;
extern float remote_dbg_frame_count;
extern float remote_dbg_raw_state;
extern float remote_dbg_uart4_isr_count;
#endif /* _REMOTE_H_ */
