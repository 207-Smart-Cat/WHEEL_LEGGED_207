#include "jump_control.h"

#include "FiveBarLinkageData.h"
#include "engine.h"
#include "imu.h"
#include "small_driver_uart_control.h"

#define JUMP_PREPARE_X       (0.00f)
#define JUMP_PREPARE_Y       (0.03f)
#define JUMP_BURST_PWM_1     (1050)
#define JUMP_BURST_PWM_2     (1050)
#define JUMP_AIR_RETRACT_X   (0.00f)
#define JUMP_AIR_RETRACT_Y   (0.03f)
#define JUMP_PRE_BUFFER_X    (0.00f)
#define JUMP_PRE_BUFFER_Y    (0.05f)
#define JUMP_EXE_BUFFER_X    (-0.01f)
#define JUMP_EXE_BUFFER_Y    (0.05f)
#define JUMP_RECOVER_X       (0.00f)
#define JUMP_RECOVER_Y       (0.04f)

#define JUMP_PREPARE_MS      (50U)
#define JUMP_BURST_MS        (130U)
#define JUMP_AIR_RETRACT_MS  (80U)
#define JUMP_PRE_BUFFER_MS   (50U)
#define JUMP_LANDING_MAX_MS  (600U)
#define JUMP_RECOVER_MS      (200U)
#define JUMP_LAND_ACCEL_G    (3.0f)

volatile JumpState jump_state = JUMP_FREE;
volatile uint8 jump_engine_suspend = 0;
volatile uint8 jump_encoder_suspend = 0;

static uint16 jump_state_elapsed_ms = 0;

static void jump_set_suspend(uint8 engine_suspend, uint8 encoder_suspend)
{
    jump_engine_suspend = engine_suspend;
    jump_encoder_suspend = encoder_suspend;
    jump_stop = engine_suspend;
}

static void jump_set_state(JumpState next_state)
{
    jump_state = next_state;
    jump_state_elapsed_ms = 0;
}

static void jump_drive_leg(float x, float y)
{
    int left_1;
    int left_2;
    int right_1;
    int right_2;

    servo_control(SERVO_LEG_LEFT, x, y, &left_1, &left_2);
    servo_control(SERVO_LEG_RIGHT, x, y, &right_1, &right_2);
    engine_left_maintain(left_1, left_2);
    engine_right_maintain(right_1, right_2);
}

void jump_start(void)
{
    if (jump_state == JUMP_FREE)
    {
        jump_position = 1;
        jump_set_suspend(1, 1);
        small_driver_set_duty(0, 0);
        jump_set_state(JUMP_PREPARE);
    }
}

uint8 jump_is_active(void)
{
    return (jump_state != JUMP_FREE);
}

uint8 jump_should_suspend_engine(void)
{
    return jump_engine_suspend;
}

uint8 jump_should_suspend_encoder(void)
{
    return jump_encoder_suspend;
}

void jump_process_control(float *current_x, float *current_y)
{
    if (jump_state == JUMP_FREE)
    {
        return;
    }

    jump_state_elapsed_ms++;

    switch (jump_state)
    {
        case JUMP_PREPARE:
            *current_x = JUMP_PREPARE_X;
            *current_y = JUMP_PREPARE_Y;
            jump_drive_leg(*current_x, *current_y);
            if (jump_state_elapsed_ms >= JUMP_PREPARE_MS)
            {
                jump_set_state(JUMP_BURST);
            }
            break;

        case JUMP_BURST:
            engine_left_maintain(JUMP_BURST_PWM_1, JUMP_BURST_PWM_2);
            engine_right_maintain(JUMP_BURST_PWM_1, JUMP_BURST_PWM_2);
            if (jump_state_elapsed_ms >= JUMP_BURST_MS)
            {
                jump_set_state(JUMP_AIR_RETRACT);
            }
            break;

        case JUMP_AIR_RETRACT:
            *current_x = JUMP_AIR_RETRACT_X;
            *current_y = JUMP_AIR_RETRACT_Y;
            jump_drive_leg(*current_x, *current_y);
            if (jump_state_elapsed_ms >= JUMP_AIR_RETRACT_MS)
            {
                jump_set_state(JUMP_PRE_BUFFER);
            }
            break;

        case JUMP_PRE_BUFFER:
            *current_x = JUMP_PRE_BUFFER_X;
            *current_y = JUMP_PRE_BUFFER_Y;
            jump_drive_leg(*current_x, *current_y);
            if (jump_state_elapsed_ms >= JUMP_PRE_BUFFER_MS)
            {
                jump_set_state(JUMP_EXE_BUFFER);
            }
            break;

        case JUMP_EXE_BUFFER:
            *current_x = JUMP_EXE_BUFFER_X;
            *current_y = JUMP_EXE_BUFFER_Y;
            jump_drive_leg(*current_x, *current_y);
            if (IMU_data.accel[2] >= JUMP_LAND_ACCEL_G || jump_state_elapsed_ms >= JUMP_LANDING_MAX_MS)
            {
                jump_set_state(JUMP_RECOVER);
            }
            break;

        case JUMP_RECOVER:
            *current_x = JUMP_RECOVER_X;
            *current_y = JUMP_RECOVER_Y;
            jump_drive_leg(*current_x, *current_y);
            if (jump_state_elapsed_ms >= JUMP_RECOVER_MS)
            {
                jump_set_suspend(0, 0);
                jump_position = 0;
                jump_set_state(JUMP_FREE);
            }
            break;

        case JUMP_FREE:
        default:
            jump_set_suspend(0, 0);
            jump_position = 0;
            jump_set_state(JUMP_FREE);
            break;
    }
}

void jump_abort(void)
{
    jump_drive_leg(JUMP_RECOVER_X, JUMP_RECOVER_Y);
    jump_set_suspend(0, 0);
    jump_position = 0;
    jump_set_state(JUMP_FREE);
}
