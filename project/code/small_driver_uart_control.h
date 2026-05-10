#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_2        )

#define SMALL_DRIVER_BAUDRATE                   (460800        )

#define SMALL_DRIVER_RX                         (UART2_TX_P10_1)

#define SMALL_DRIVER_TX                         (UART2_RX_P10_0)


/*
typedef struct
{
    uint8 send_data_buffer[7];                  // 发送缓冲数组

    uint8 receive_data_buffer[7];               // 接收缓冲数组

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_left_speed_data;              // 接收到的左侧电机速度数据

    int16 receive_right_speed_data;             // 接收到的右侧电机速度数据
    
    int32 receive_left_encoder_data;            // 左侧编码器数据 (建议用int32防止溢出)
    
    int32 receive_right_encoder_data;           // 右侧编码器数据

}small_device_value_struct;
*/


typedef struct
{
    uint8 send_data_buffer[7];                  // 发送缓冲数组

    uint8 receive_data_buffer[7];               // 接收缓冲数组

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_left_speed_data;              // 接收到的左侧电机速度数据

    int16 receive_right_speed_data;             // 接收到的右侧电机速度数据

}small_device_value_struct;

extern small_device_value_struct motor_value;
extern volatile uint16 motor_zero_dbg_elapsed_ms;
extern volatile uint8 motor_zero_dbg_rx_seen;
extern volatile uint8 motor_zero_dbg_speed_seen;
extern volatile uint32 motor_zero_dbg_start_count;
extern volatile uint32 motor_zero_dbg_tx_count;
extern volatile uint32 motor_zero_dbg_task_count;
extern volatile uint32 motor_zero_dbg_rx_count;

typedef enum
{
    MOTOR_ZERO_STATE_IDLE = 0,
    MOTOR_ZERO_STATE_WAIT_REPLY,
    MOTOR_ZERO_STATE_SETTLE,
    MOTOR_ZERO_STATE_DONE,
    MOTOR_ZERO_STATE_TIMEOUT
} motor_zero_state_t;

void uart_control_callback(void);                                   // 无刷驱动 串口接收回调函数

void small_driver_set_duty(int16 left_duty, int16 right_duty);      // 无刷驱动 设置电机占空比

void small_driver_get_speed(void);                                  // 无刷驱动 获取速度信息

void small_driver_uart_init(void);                                  // 无刷驱动 串口通讯初始化
void small_driver_zero_calibration_start(void);                      // 发送 SET-ZERO 并启动零点校准状态机
void small_driver_zero_calibration_task(void);                       // 1ms 调用，推进零点校准状态机
uint8 small_driver_zero_calibration_is_active(void);                 // 查询零点校准期间是否需要强制电机零输出
motor_zero_state_t small_driver_zero_calibration_state(void);        // 查询当前零点校准状态
void small_driver_request_startup_ramp_reset(void);                  // 请求下一次电机输出从斜坡起步

/*
// 编码器字符串通讯相关函数声明
void small_driver_request_encoder(void);
void uart_encoder_string_callback(void);
void small_driver_uart_init_encoder(void);
*/

#endif
