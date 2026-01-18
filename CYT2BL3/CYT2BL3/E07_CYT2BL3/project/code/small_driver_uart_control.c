#include "small_driver_uart_control.h"

small_device_value_struct motor_value_1;      // 定义通讯参数结构体
small_device_value_struct motor_value_2;      // 定义通讯参数结构体

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback(1000, -1000);
// 备注信息     用于解析接收到的速度数据  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(small_device_enum device_num)
{
    uint8 receive_data;                                                                     // 定义临时变量
    uint8 receive_flag;                                                                     
    small_device_value_struct *motor_value_temp;
  
    if(device_num == SMALL_DRIVER_1)
    {
        motor_value_temp = &motor_value_1;
        receive_flag = uart_query_byte(SMALL_DRIVER_UART_1, &receive_data);
    }
    else
    {
        motor_value_temp = &motor_value_2;
        receive_flag = uart_query_byte(SMALL_DRIVER_UART_2, &receive_data);
    }
    
    if(receive_flag)                                   // 接收串口数据
    {
        if(receive_data == 0xA5 && motor_value_temp->receive_data_buffer[0] != 0xA5)              // 判断是否收到帧头 并且 当前接收内容中是否正确包含帧头
        {
            motor_value_temp->receive_data_count = 0;                                             // 未收到帧头或者未正确包含帧头则重新接收
        }

        motor_value_temp->receive_data_buffer[motor_value_temp->receive_data_count ++] = receive_data;  // 保存串口数据

        if(motor_value_temp->receive_data_count >= 7)                                             // 判断是否接收到指定数量的数据
        {
            if(motor_value_temp->receive_data_buffer[0] == 0xA5)                                  // 判断帧头是否正确
            {

                motor_value_temp->sum_check_data = 0;                                             // 清除校验位数据

                for(int i = 0; i < 6; i ++)
                {
                    motor_value_temp->sum_check_data += motor_value_temp->receive_data_buffer[i];       // 重新计算校验位
                }

                if(motor_value_temp->sum_check_data == motor_value_temp->receive_data_buffer[6])        // 校验数据准确性
                {

                    if(motor_value_temp->receive_data_buffer[1] == 0x02)                          // 判断是否正确接收到 速度输出 功能字
                    {
                        motor_value_temp->receive_left_speed_data  = (((int)motor_value_temp->receive_data_buffer[2] << 8) | (int)motor_value_temp->receive_data_buffer[3]);  // 拟合左侧电机转速数据

                        motor_value_temp->receive_right_speed_data = (((int)motor_value_temp->receive_data_buffer[4] << 8) | (int)motor_value_temp->receive_data_buffer[5]);  // 拟合右侧电机转速数据
                    }

                    motor_value_temp->receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value_temp->receive_data_buffer, 0, 7);                          // 清除缓冲区数据
                }
                else
                {
                    motor_value_temp->receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value_temp->receive_data_buffer, 0, 7);                          // 清除缓冲区数据
                }
            }
            else
            {
                motor_value_temp->receive_data_count = 0;                                         // 清除缓冲区计数值

                memset(motor_value_temp->receive_data_buffer, 0, 7);                              // 清除缓冲区数据
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置电机占空比
// 参数说明     left_duty       左侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 参数说明     right_duty      右侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 返回参数     void
// 使用示例     small_driver_set_duty(1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(small_device_enum device_num, int16 left_duty, int16 right_duty)
{
    small_device_value_struct *motor_value_temp;
  
    if(device_num == SMALL_DRIVER_1)
    {
        motor_value_temp = &motor_value_1;
    }
    else
    {
        motor_value_temp = &motor_value_2;
    }
    
    motor_value_temp->send_data_buffer[0] = 0xA5;                                         // 配置帧头
    
    motor_value_temp->send_data_buffer[1] = 0X01;                                         // 配置功能字
    
    motor_value_temp->send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);           // 拆分 左侧占空比 的高八位
    
    motor_value_temp->send_data_buffer[3] = (uint8)(left_duty & 0x00FF);                  // 拆分 左侧占空比 的低八位
    
    motor_value_temp->send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);          // 拆分 右侧占空比 的高八位
    
    motor_value_temp->send_data_buffer[5] = (uint8)(right_duty & 0x00FF);                 // 拆分 右侧占空比 的低八位
    
    motor_value_temp->send_data_buffer[6] = 0;                                            // 和校验清除

    for(int i = 0; i < 6; i ++)
    {
        motor_value_temp->send_data_buffer[6] += motor_value_temp->send_data_buffer[i];         // 计算校验位
    }

    if(device_num == SMALL_DRIVER_1)
    {
        uart_write_buffer(SMALL_DRIVER_UART_1, motor_value_temp->send_data_buffer, 7);                     // 发送获取转速数据的 字节包 数据
    }
    else
    {
        uart_write_buffer(SMALL_DRIVER_UART_2, motor_value_temp->send_data_buffer, 7);                     // 发送获取转速数据的 字节包 数据
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     仅需发送一次 驱动将周期发出速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(small_device_enum device_num)
{  
    small_device_value_struct *motor_value_temp;
  
    if(device_num == SMALL_DRIVER_1)
    {
        motor_value_temp = &motor_value_1;
    }
    else
    {
        motor_value_temp = &motor_value_2;
    }
    
    motor_value_temp->send_data_buffer[0] = 0xA5;                                         // 配置帧头
    
    motor_value_temp->send_data_buffer[1] = 0X02;                                         // 配置功能字
    
    motor_value_temp->send_data_buffer[2] = 0x00;                                         // 数据位清空
    
    motor_value_temp->send_data_buffer[3] = 0x00;                                         // 数据位清空
    
    motor_value_temp->send_data_buffer[4] = 0x00;                                         // 数据位清空
    
    motor_value_temp->send_data_buffer[5] = 0x00;                                         // 数据位清空
    
    motor_value_temp->send_data_buffer[6] = 0xA7;                                         // 配置校验位

    if(device_num == SMALL_DRIVER_1)
    {
        uart_write_buffer(SMALL_DRIVER_UART_1, motor_value_temp->send_data_buffer, 7);                     // 发送获取转速数据的 字节包 数据
    }
    else
    {
        uart_write_buffer(SMALL_DRIVER_UART_2, motor_value_temp->send_data_buffer, 7);                     // 发送获取转速数据的 字节包 数据
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(small_device_enum device_num)
{
    small_device_value_struct *motor_value_temp;
  
    if(device_num == SMALL_DRIVER_1)
    {
        motor_value_temp = &motor_value_1;
    }
    else
    {
        motor_value_temp = &motor_value_2;
    }
    
    memset(motor_value_temp->send_data_buffer, 0, 7);                             // 清除缓冲区数据

    memset(motor_value_temp->receive_data_buffer, 0, 7);                          // 清除缓冲区数据

    motor_value_temp->receive_data_count          = 0;

    motor_value_temp->sum_check_data              = 0;

    motor_value_temp->receive_right_speed_data    = 0;

    motor_value_temp->receive_left_speed_data     = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(small_device_enum device_num)
{
    if(device_num == SMALL_DRIVER_1)
    {
        uart_init(SMALL_DRIVER_UART_1, SMALL_DRIVER_BAUDRATE_1, SMALL_DRIVER_RX_1, SMALL_DRIVER_TX_1);          // 串口初始化

        uart_rx_interrupt(SMALL_DRIVER_UART_1, 1);                                                              // 使能串口接收中断

        small_driver_init(device_num);                                                                          // 结构体参数初始化
  
        small_driver_set_duty(device_num, 0, 0);                                                                // 设置0占空比

        small_driver_get_speed(device_num);                                                                     // 获取实时速度数据
    }
    else
    {
        uart_init(SMALL_DRIVER_UART_2, SMALL_DRIVER_BAUDRATE_2, SMALL_DRIVER_RX_2, SMALL_DRIVER_TX_2);          // 串口初始化

        uart_rx_interrupt(SMALL_DRIVER_UART_2, 1);                                                              // 使能串口接收中断

        small_driver_init(device_num);                                                                          // 结构体参数初始化
  
        small_driver_set_duty(device_num, 0, 0);                                                                // 设置0占空比

        small_driver_get_speed(device_num);                                                                     // 获取实时速度数据        
    }
}














