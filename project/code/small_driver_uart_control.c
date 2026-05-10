#include "small_driver_uart_control.h"
#include "ipc_shared_data.h"
#include "param.h"
#include "runtime_status.h"
#define MOTOR_STARTUP_DUTY_STEP      (120)
#define MOTOR_DUTY_MAX_ABS           (MAX_DUTY * (PWM_DUTY_MAX / 100))
#define MOTOR_ZERO_WAIT_MIN_MS       (2000U)
#define MOTOR_ZERO_WAIT_TIMEOUT_MS   (5000U)
#define MOTOR_ZERO_SETTLE_MS         (200U)

static volatile motor_zero_state_t motor_zero_state = MOTOR_ZERO_STATE_IDLE;
static volatile uint8 motor_zero_rx_seen = 0;
static volatile uint8 motor_zero_speed_seen = 0;
static volatile uint8 motor_startup_ramp_reset_request = 0;
static uint16 motor_zero_elapsed_ms = 0;

volatile uint16 motor_zero_dbg_elapsed_ms = 0;
volatile uint8 motor_zero_dbg_rx_seen = 0;
volatile uint8 motor_zero_dbg_speed_seen = 0;
volatile uint32 motor_zero_dbg_start_count = 0;
volatile uint32 motor_zero_dbg_tx_count = 0;
volatile uint32 motor_zero_dbg_task_count = 0;
volatile uint32 motor_zero_dbg_rx_count = 0;

static uint8 motor_output_is_allowed(void)
{
    if (IPC_CoreB_Wifi_Is_Connected() == 0)
    {
        Runtime_Set_Motor_Reason(RUNTIME_REASON_WIFI_OFF);
        return 0;
    }

    Runtime_Set_Motor_Reason(RUNTIME_REASON_NORMAL);
    return 1;
}

static int16 motor_duty_abs_limit(int16 target, int16 limit_abs)
{
    if (target > limit_abs)
    {
        return limit_abs;
    }
    if (target < -limit_abs)
    {
        return (int16)(-limit_abs);
    }
    return target;
}

static void motor_safety_reset(uint8 *enabled, int16 *startup_limit)
{
    *enabled = 0;
    *startup_limit = 0;
}

static void motor_startup_ramp(uint8 *enabled, int16 *startup_limit)
{
    if (*enabled == 0)
    {
        *enabled = 1;
        *startup_limit = 0;
    }

    if (*startup_limit < MOTOR_DUTY_MAX_ABS)
    {
        *startup_limit += MOTOR_STARTUP_DUTY_STEP;
        if (*startup_limit > MOTOR_DUTY_MAX_ABS)
        {
            *startup_limit = MOTOR_DUTY_MAX_ABS;
        }
    }
}

void small_driver_request_startup_ramp_reset(void)
{
    motor_startup_ramp_reset_request = 1;
}

uint8 small_driver_zero_calibration_is_active(void)
{
    return (motor_zero_state == MOTOR_ZERO_STATE_WAIT_REPLY || motor_zero_state == MOTOR_ZERO_STATE_SETTLE) ? 1 : 0;
}

motor_zero_state_t small_driver_zero_calibration_state(void)
{
    return motor_zero_state;
}

void small_driver_zero_calibration_start(void)
{
    uint8 set_zero_cmd[7];
    uint8 set_zero_text[] = "SET-ZERO\n";

    motor_zero_dbg_start_count++;

    if (small_driver_zero_calibration_is_active())
    {
        return;
    }

    motor_zero_rx_seen = 0;
    motor_zero_speed_seen = 0;
    motor_zero_elapsed_ms = 0;
    motor_zero_state = MOTOR_ZERO_STATE_WAIT_REPLY;
    IPC_Update_Motor_Zero_State_From_Core0((uint8)motor_zero_state);

    small_driver_set_duty(0, 0);

    set_zero_cmd[0] = 0xA5;
    set_zero_cmd[1] = 0x03;
    set_zero_cmd[2] = 0x00;
    set_zero_cmd[3] = 0x00;
    set_zero_cmd[4] = 0x00;
    set_zero_cmd[5] = 0x00;
    set_zero_cmd[6] = set_zero_cmd[0] + set_zero_cmd[1] + set_zero_cmd[2] + set_zero_cmd[3] + set_zero_cmd[4] + set_zero_cmd[5];
    uart_write_buffer(SMALL_DRIVER_UART, set_zero_cmd, 7);
    uart_write_buffer(SMALL_DRIVER_UART, set_zero_text, sizeof(set_zero_text) - 1);
    motor_zero_dbg_tx_count++;

    small_driver_get_speed();
}

void small_driver_zero_calibration_task(void)
{
    if (motor_zero_state == MOTOR_ZERO_STATE_IDLE || motor_zero_state == MOTOR_ZERO_STATE_DONE || motor_zero_state == MOTOR_ZERO_STATE_TIMEOUT)
    {
        return;
    }

    if (motor_zero_elapsed_ms < 0xFFFFU)
    {
        motor_zero_elapsed_ms++;
    }
    motor_zero_dbg_task_count++;
    motor_zero_dbg_elapsed_ms = motor_zero_elapsed_ms;
    motor_zero_dbg_rx_seen = motor_zero_rx_seen;
    motor_zero_dbg_speed_seen = motor_zero_speed_seen;

    if (motor_zero_state == MOTOR_ZERO_STATE_WAIT_REPLY)
    {
        if (motor_zero_elapsed_ms >= MOTOR_ZERO_WAIT_MIN_MS && motor_zero_rx_seen && motor_zero_speed_seen)
        {
            motor_zero_elapsed_ms = 0;
            motor_zero_state = MOTOR_ZERO_STATE_SETTLE;
        }
        else if (motor_zero_elapsed_ms >= MOTOR_ZERO_WAIT_TIMEOUT_MS)
        {
            motor_zero_state = MOTOR_ZERO_STATE_TIMEOUT;
            small_driver_request_startup_ramp_reset();
        }
    }
    else if (motor_zero_state == MOTOR_ZERO_STATE_SETTLE)
    {
        if (motor_zero_elapsed_ms >= MOTOR_ZERO_SETTLE_MS)
        {
            motor_zero_state = MOTOR_ZERO_STATE_DONE;
            small_driver_request_startup_ramp_reset();
        }
    }

    motor_zero_dbg_elapsed_ms = motor_zero_elapsed_ms;
    motor_zero_dbg_rx_seen = motor_zero_rx_seen;
    motor_zero_dbg_speed_seen = motor_zero_speed_seen;
    IPC_Update_Motor_Zero_State_From_Core0((uint8)motor_zero_state);
}

small_device_value_struct motor_value;      // 定义通讯参数结构体


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback(1000, -1000);
// 备注信息     用于解析接收到的速度数据  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data;                                                                     // 定义临时变量

    if(uart_query_byte(SMALL_DRIVER_UART, &receive_data))                                   // 接收串口数据
    {
        motor_zero_dbg_rx_count++;
        if (small_driver_zero_calibration_is_active())
        {
            motor_zero_rx_seen = 1;
            motor_zero_dbg_rx_seen = 1;
        }
        if(receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)              // 判断是否收到帧头 并且 当前接收内容中是否正确包含帧头
        {
            motor_value.receive_data_count = 0;                                             // 未收到帧头或者未正确包含帧头则重新接收
        }

        motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;  // 保存串口数据

        if(motor_value.receive_data_count >= 7)                                             // 判断是否接收到指定数量的数据
        {
            if(motor_value.receive_data_buffer[0] == 0xA5)                                  // 判断帧头是否正确
            {

                motor_value.sum_check_data = 0;                                             // 清除校验位数据

                for(int i = 0; i < 6; i ++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];       // 重新计算校验位
                }

                if(motor_value.sum_check_data == motor_value.receive_data_buffer[6])        // 校验数据准确性
                {

                    if(motor_value.receive_data_buffer[1] == 0x02)                          // 判断是否正确接收到 速度输出 功能字
                    {
                        motor_value.receive_left_speed_data  = (((int)motor_value.receive_data_buffer[2] << 8) | (int)motor_value.receive_data_buffer[3]);  // 拟合左侧电机转速数据

                        motor_value.receive_right_speed_data = (((int)motor_value.receive_data_buffer[4] << 8) | (int)motor_value.receive_data_buffer[5]);  // 拟合右侧电机转速数据

                        motor_zero_speed_seen = 1;
                        motor_zero_dbg_speed_seen = 1;
                    }

                    motor_value.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value.receive_data_buffer, 0, 7);                          // 清除缓冲区数据
                }
                else
                {
                    motor_value.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value.receive_data_buffer, 0, 7);                          // 清除缓冲区数据
                }
            }
            else
            {
                motor_value.receive_data_count = 0;                                         // 清除缓冲区计数值

                memset(motor_value.receive_data_buffer, 0, 7);                              // 清除缓冲区数据
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
void small_driver_set_duty(int16 left_duty, int16 right_duty)
{
    static uint8 motor_output_enabled = 0;
    static int16 startup_duty_limit = 0;

    if (motor_startup_ramp_reset_request)
    {
        motor_safety_reset(&motor_output_enabled, &startup_duty_limit);
        motor_startup_ramp_reset_request = 0;
    }

    if (small_driver_zero_calibration_is_active())
    {
        left_duty = 0;
        right_duty = 0;
        motor_safety_reset(&motor_output_enabled, &startup_duty_limit);
    }
    else if (motor_output_is_allowed() == 0)
    {
        left_duty = 0;
        right_duty = 0;
        motor_safety_reset(&motor_output_enabled, &startup_duty_limit);
    }
    else
    {
        motor_startup_ramp(&motor_output_enabled, &startup_duty_limit);
        if (startup_duty_limit < MOTOR_DUTY_MAX_ABS)
        {
            Runtime_Set_Motor_Reason(RUNTIME_REASON_STARTUP_RAMP);
        }
        left_duty = motor_duty_abs_limit(left_duty, startup_duty_limit);
        right_duty = motor_duty_abs_limit(right_duty, startup_duty_limit);
    }

    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X01;                                         // 配置功能字

    motor_value.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);           // 拆分 左侧占空比 的高八位

    motor_value.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);                  // 拆分 左侧占空比 的低八位

    motor_value.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);          // 拆分 右侧占空比 的高八位

    motor_value.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);                 // 拆分 右侧占空比 的低八位

    motor_value.send_data_buffer[6] = 0;                                            // 和校验清除

    for(int i = 0; i < 6; i ++)
    {
        motor_value.send_data_buffer[6] += motor_value.send_data_buffer[i];         // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);                     // 发送设置占空比的 字节包 数据
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     仅需发送一次 驱动将周期发出速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X02;                                         // 配置功能字

    motor_value.send_data_buffer[2] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[3] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[4] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[5] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[6] = 0xA7;                                         // 配置校验位

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);                     // 发送获取转速数据的 字节包 数据
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 7);                             // 清除缓冲区数据

    memset(motor_value.receive_data_buffer, 0, 7);                          // 清除缓冲区数据

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;

    motor_value.receive_right_speed_data    = 0;

    motor_value.receive_left_speed_data     = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体参数初始化

    small_driver_set_duty(0, 0);                                                                // 设置0占空比

    small_driver_get_speed();                                                                   // 获取实时速度数据
}




//**************************新增*************************************
/*
// ==============================================================================
// 智能车助手新增：编码器字符串通讯支持代码
// ==============================================================================

char  encoder_rx_buffer[32];           // 字符串接收缓存区
uint8 encoder_rx_count = 0;            // 字符串接收计数器

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      无刷驱动 请求编码器信息 (字符串模式)
// 返回参数      void
//-------------------------------------------------------------------------------------------------------------------
void small_driver_request_encoder(void)
{
    char *cmd = "GET-ENCODER\n";
    uart_write_buffer(SMALL_DRIVER_UART, (uint8 *)cmd, strlen(cmd)); // 发送字符串指令
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      无刷驱动 字符串接收回调函数 (解析 "123,456" 格式)
// 备注信息      需要在对应的串口接收中断中调用，替换原有的 uart_control_callback
//-------------------------------------------------------------------------------------------------------------------
void uart_encoder_string_callback(void)
{
    uint8 receive_data;

    // 循环读取串口接收寄存器中的所有数据
    while(uart_query_byte(SMALL_DRIVER_UART, &receive_data))       
    {
        if(receive_data == '\n')                                // 遇到回车，说明一帧数据接收完毕
        {
            encoder_rx_buffer[encoder_rx_count] = '\0';         // 封口，变成标准 C 语言字符串

            // 寻找逗号的位置
            char *comma_ptr = strchr(encoder_rx_buffer, ',');
            if(comma_ptr != NULL)
            {
                *comma_ptr = '\0';                              // 将逗号替换为结束符 '\0'，把字符串劈成两半
                
                // 将字符串转为数字，并存入我们结构体的新变量中
                motor_value.receive_left_encoder_data = atoi(encoder_rx_buffer);       
                motor_value.receive_right_encoder_data = atoi(comma_ptr + 1);          
            }

            encoder_rx_count = 0;                               // 解析完清零计数器
            memset(encoder_rx_buffer, 0, sizeof(encoder_rx_buffer)); // 清空缓存区
        }
        else if(receive_data != '\r')                           // 过滤掉可能存在的 '\r' 字符
        {
            if(encoder_rx_count < 30)                           // 数组防溢出保护
            {
                encoder_rx_buffer[encoder_rx_count ++] = receive_data; // 保存有效字符
            }
            else
            {
                encoder_rx_count = 0;                           // 超长则认为是错误数据，直接丢弃
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      无刷驱动 串口通讯初始化 (获取编码器专用版本)
// 备注信息      在 main 函数中调用此函数，替代原有的 small_driver_uart_init
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init_encoder(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化
    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体参数初始化
    
    // 清空编码器相关变量
    motor_value.receive_left_encoder_data = 0;
    motor_value.receive_right_encoder_data = 0;
    encoder_rx_count = 0;
    memset(encoder_rx_buffer, 0, sizeof(encoder_rx_buffer));

    small_driver_set_duty(0, 0);                                                                // 设置0占空比防暴走
    small_driver_request_encoder();                                                             // 发送请求编码器指令
}
//********************************************************************



*/












