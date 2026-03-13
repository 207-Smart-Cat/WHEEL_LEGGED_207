#ifndef _REMOTE_H_
#define _REMOTE_H_

#include "zf_common_headfile.h"
// 定义遥控器通道数量 (6个)
#define REMOTE_CHANNEL_NUM  6

// 定义遥控器失控时的安全默认值 (通常摇杆回中值为1500，取决于具体遥控器)
#define REMOTE_SAFE_VALUE_CH1   888
#define REMOTE_SAFE_VALUE_CH2   1000
#define REMOTE_SAFE_VALUE_CHother   192
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
#endif /* _REMOTE_H_ */