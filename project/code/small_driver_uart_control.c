#include "small_driver_uart_control.h"
#include "ipc_shared_data.h"
#include "param.h"
#include "runtime_status.h"
#define MOTOR_STARTUP_DUTY_STEP      (1)
#define MOTOR_DUTY_MAX_ABS           (MAX_DUTY * (PWM_DUTY_MAX / 100))
#define MOTOR_ZERO_WAIT_MIN_MS       (5500U)
#define MOTOR_ZERO_WAIT_TIMEOUT_MS   (8000U)
#define MOTOR_ZERO_SETTLE_MS         (500U)

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

static uint8 small_driver_try_write_packet(const uint8 *buffer, uint32 length)
{
    volatile stc_SCB_t *uart_base = get_scb_module(SMALL_DRIVER_UART);
    uint32 free_count = Cy_SCB_GetFifoSize(uart_base) - Cy_SCB_UART_GetNumInTxFifo(uart_base);

    if (free_count < length)
    {
        return 0;
    }

    Cy_SCB_UART_PutArray(uart_base, (void *)buffer, length);
    return 1;
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

static void small_driver_send_zero_duty_direct(void)
{
    uint8 zero_duty_cmd[7] = {0xA5, 0x01, 0x00, 0x00, 0x00, 0x00, 0xA6};

    uart_write_buffer(SMALL_DRIVER_UART, zero_duty_cmd, sizeof(zero_duty_cmd));
}

static void small_driver_stop_send(void)
{
    uint8 stop_send_cmd[] = "STOP-SEND\n";

    uart_write_buffer(SMALL_DRIVER_UART, stop_send_cmd, sizeof(stop_send_cmd) - 1);
}

static void small_driver_send_set_zero(void)
{
    uint8 set_zero_cmd[] = "SET-ZERO\n";

    uart_write_buffer(SMALL_DRIVER_UART, set_zero_cmd, sizeof(set_zero_cmd) - 1);
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

    small_driver_send_zero_duty_direct();
    small_driver_stop_send();

    small_driver_send_set_zero();
    motor_zero_dbg_tx_count++;
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
        if (motor_zero_elapsed_ms >= MOTOR_ZERO_WAIT_MIN_MS)
        {
            small_driver_get_speed();
            motor_zero_state = MOTOR_ZERO_STATE_DONE;
            small_driver_request_startup_ramp_reset();
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

small_device_value_struct motor_value;      // Legacy comment removed: original encoding was damaged.


//-------------------------------------------------------------------------------------------------------------------
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data;                                                                     // Legacy comment removed: original encoding was damaged.

    while(uart_query_byte(SMALL_DRIVER_UART, &receive_data))                                // Legacy comment removed: original encoding was damaged.
    {
        motor_zero_dbg_rx_count++;
        if (small_driver_zero_calibration_is_active())
        {
            motor_zero_rx_seen = 1;
            motor_zero_dbg_rx_seen = 1;
        }
        if(receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)              // Legacy comment removed: original encoding was damaged.
        {
            motor_value.receive_data_count = 0;                                             // Legacy comment removed: original encoding was damaged.
        }

        motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;  // Legacy comment removed: original encoding was damaged.

        if(motor_value.receive_data_count >= 7)                                             // Legacy comment removed: original encoding was damaged.
        {
            if(motor_value.receive_data_buffer[0] == 0xA5)                                  // Legacy comment removed: original encoding was damaged.
            {

                motor_value.sum_check_data = 0;                                             // Legacy comment removed: original encoding was damaged.

                for(int i = 0; i < 6; i ++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];       // Legacy comment removed: original encoding was damaged.
                }

                if(motor_value.sum_check_data == motor_value.receive_data_buffer[6])        // Legacy comment removed: original encoding was damaged.
                {

                    if(motor_value.receive_data_buffer[1] == 0x02)                          // Legacy comment removed: original encoding was damaged.
                    {
                        motor_value.receive_left_speed_data  = (((int)motor_value.receive_data_buffer[2] << 8) | (int)motor_value.receive_data_buffer[3]);  // Legacy comment removed: original encoding was damaged.

                        motor_value.receive_right_speed_data = (((int)motor_value.receive_data_buffer[4] << 8) | (int)motor_value.receive_data_buffer[5]);  // Legacy comment removed: original encoding was damaged.

                        motor_zero_speed_seen = 1;
                        motor_zero_dbg_speed_seen = 1;
                    }

                    motor_value.receive_data_count = 0;                                     // Legacy comment removed: original encoding was damaged.

                    memset(motor_value.receive_data_buffer, 0, 7);                          // Legacy comment removed: original encoding was damaged.
                }
                else
                {
                    motor_value.receive_data_count = 0;                                     // Legacy comment removed: original encoding was damaged.

                    memset(motor_value.receive_data_buffer, 0, 7);                          // Legacy comment removed: original encoding was damaged.
                }
            }
            else
            {
                motor_value.receive_data_count = 0;                                         // Legacy comment removed: original encoding was damaged.

                memset(motor_value.receive_data_buffer, 0, 7);                              // Legacy comment removed: original encoding was damaged.
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
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
        motor_safety_reset(&motor_output_enabled, &startup_duty_limit);
        return;
    }
    else if (motor_output_is_allowed() == 0)
    {
        left_duty = 0;
        right_duty = 0;
        motor_safety_reset(&motor_output_enabled, &startup_duty_limit);
    }
    else
    {
        // Temporarily bypass startup ramp limiting so commanded duty reaches
        // the normal control-path limit immediately during obstacle crossing.
        (void)motor_output_enabled;
        (void)startup_duty_limit;
        left_duty = motor_duty_abs_limit(left_duty, MOTOR_DUTY_MAX_ABS);
        right_duty = motor_duty_abs_limit(right_duty, MOTOR_DUTY_MAX_ABS);
    }

    motor_value.send_data_buffer[0] = 0xA5;                                         // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[1] = 0X01;                                         // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);           // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);                  // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);          // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);                 // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[6] = 0;                                            // Legacy comment removed: original encoding was damaged.

    for(int i = 0; i < 6; i ++)
    {
        motor_value.send_data_buffer[6] += motor_value.send_data_buffer[i];         // Legacy comment removed: original encoding was damaged.
    }

    (void)small_driver_try_write_packet(motor_value.send_data_buffer, 7);                    // Legacy comment removed: original encoding was damaged.
}

//-------------------------------------------------------------------------------------------------------------------
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[1] = 0X02;                                         // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[2] = 0x00;                                         // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[3] = 0x00;                                         // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[4] = 0x00;                                         // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[5] = 0x00;                                         // Legacy comment removed: original encoding was damaged.

    motor_value.send_data_buffer[6] = 0xA7;                                         // Legacy comment removed: original encoding was damaged.

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);                     // Legacy comment removed: original encoding was damaged.
}


//-------------------------------------------------------------------------------------------------------------------
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 7);                             // Legacy comment removed: original encoding was damaged.

    memset(motor_value.receive_data_buffer, 0, 7);                          // Legacy comment removed: original encoding was damaged.

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;

    motor_value.receive_right_speed_data    = 0;

    motor_value.receive_left_speed_data     = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // Legacy comment removed: original encoding was damaged.

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // Legacy comment removed: original encoding was damaged.

    small_driver_init();                                                                        // Legacy comment removed: original encoding was damaged.

    small_driver_set_duty(0, 0);                                                                // Legacy comment removed: original encoding was damaged.

    small_driver_get_speed();                                                                   // Legacy comment removed: original encoding was damaged.
}




// Legacy comment removed: original encoding was damaged.
/*
// ==============================================================================
// Legacy comment removed: original encoding was damaged.
// ==============================================================================

char  encoder_rx_buffer[32];           // Legacy comment removed: original encoding was damaged.
uint8 encoder_rx_count = 0;            // Legacy comment removed: original encoding was damaged.

//-------------------------------------------------------------------------------------------------------------------
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
//-------------------------------------------------------------------------------------------------------------------
void small_driver_request_encoder(void)
{
    char *cmd = "GET-ENCODER\n";
    uart_write_buffer(SMALL_DRIVER_UART, (uint8 *)cmd, strlen(cmd)); // Legacy comment removed: original encoding was damaged.
}

//-------------------------------------------------------------------------------------------------------------------
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
//-------------------------------------------------------------------------------------------------------------------
void uart_encoder_string_callback(void)
{
    uint8 receive_data;

    // Legacy comment removed: original encoding was damaged.
    while(uart_query_byte(SMALL_DRIVER_UART, &receive_data))       
    {
        if(receive_data == '\n')                                // Legacy comment removed: original encoding was damaged.
        {
            encoder_rx_buffer[encoder_rx_count] = '\0';         // Legacy comment removed: original encoding was damaged.

            // Legacy comment removed: original encoding was damaged.
            char *comma_ptr = strchr(encoder_rx_buffer, ',');
            if(comma_ptr != NULL)
            {
                *comma_ptr = '\0';                              // Legacy comment removed: original encoding was damaged.
                
                // Legacy comment removed: original encoding was damaged.
                motor_value.receive_left_encoder_data = atoi(encoder_rx_buffer);       
                motor_value.receive_right_encoder_data = atoi(comma_ptr + 1);          
            }

            encoder_rx_count = 0;                               // Legacy comment removed: original encoding was damaged.
            memset(encoder_rx_buffer, 0, sizeof(encoder_rx_buffer)); // Legacy comment removed: original encoding was damaged.
        }
        else if(receive_data != '\r')                           // Legacy comment removed: original encoding was damaged.
        {
            if(encoder_rx_count < 30)                           // Legacy comment removed: original encoding was damaged.
            {
                encoder_rx_buffer[encoder_rx_count ++] = receive_data; // Legacy comment removed: original encoding was damaged.
            }
            else
            {
                encoder_rx_count = 0;                           // Legacy comment removed: original encoding was damaged.
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// Legacy comment removed: original encoding was damaged.
// Legacy comment removed: original encoding was damaged.
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init_encoder(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // Legacy comment removed: original encoding was damaged.
    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // Legacy comment removed: original encoding was damaged.

    small_driver_init();                                                                        // Legacy comment removed: original encoding was damaged.
    
    // Legacy comment removed: original encoding was damaged.
    motor_value.receive_left_encoder_data = 0;
    motor_value.receive_right_encoder_data = 0;
    encoder_rx_count = 0;
    memset(encoder_rx_buffer, 0, sizeof(encoder_rx_buffer));

    small_driver_set_duty(0, 0);                                                                // Legacy comment removed: original encoding was damaged.
    small_driver_request_encoder();                                                             // Legacy comment removed: original encoding was damaged.
}
// End of legacy serial-string helper section.
*/











