#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART_1                       (UART_5        )

#define SMALL_DRIVER_BAUDRATE_1                   (460800        )

#define SMALL_DRIVER_RX_1                         (UART5_TX_P02_1)

#define SMALL_DRIVER_TX_1                         (UART5_RX_P02_0)

#define SMALL_DRIVER_UART_2                       (UART_6        )

#define SMALL_DRIVER_BAUDRATE_2                   (460800        )

#define SMALL_DRIVER_RX_2                         (UART6_TX_P18_1)

#define SMALL_DRIVER_TX_2                         (UART6_RX_P18_0)

typedef struct
{
    uint8 send_data_buffer[7];                  // 发送缓冲数组

    uint8 receive_data_buffer[7];               // 接收缓冲数组

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_left_speed_data;              // 接收到的左侧电机速度数据

    int16 receive_right_speed_data;             // 接收到的右侧电机速度数据

}small_device_value_struct;

typedef enum
{
    SMALL_DRIVER_1,
    SMALL_DRIVER_2,
}small_device_enum;


extern small_device_value_struct motor_value_1; 
extern small_device_value_struct motor_value_2; 

void uart_control_callback(small_device_enum device_num);                                               // 无刷驱动 串口接收回调函数

void small_driver_set_duty(small_device_enum device_num, int16 left_duty, int16 right_duty);            // 无刷驱动 设置电机占空比

void small_driver_get_speed(small_device_enum device_num);                                              // 无刷驱动 获取速度信息

void small_driver_uart_init(small_device_enum device_num);                                              // 无刷驱动 串口通讯初始化

#endif
