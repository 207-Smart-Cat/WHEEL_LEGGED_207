#include "jump_control.h"

#include "FiveBarLinkageData.h"
#include "engine.h"
#include "imu.h"
#include "small_driver_uart_control.h"

#define JUMP_PREPARE_X       (0.00f)
#define JUMP_PREPARE_Y       (0.03f)
#define JUMP_BURST_PWM       (1200)
#define JUMP_SERVO_SUM       (1500)
#define JUMP_SERVO_MIN_PWM   (300)
#define JUMP_SERVO_MAX_PWM   (1200)
#define JUMP_AIR_RETRACT_X   (0.01f)
#define JUMP_AIR_RETRACT_Y   (0.03f)
#define JUMP_EXE_BUFFER_X    (+0.05f)
#define JUMP_EXE_BUFFER_Y    (0.05f)
#define JUMP_RECOVER_X       (0.00f)
#define JUMP_RECOVER_Y       (0.04f)

#define JUMP_PREPARE_MS      (50U)
#define JUMP_BURST_MS        (130U)
#define JUMP_AIR_RETRACT_MS  (80U)
#define JUMP_RECOVER_MS      (50U)
#define JUMP_LANDING_MAX_MS  (600U)
#define JUMP_LAND_ACCEL_G    (1.0f)

volatile JumpState jump_state = JUMP_FREE;
volatile uint8 jump_engine_suspend = 0;
volatile uint8 jump_encoder_suspend = 0;
volatile uint8 jump_dbg_state = JUMP_FREE;
volatile uint16 jump_dbg_elapsed_ms = 0;
volatile uint8 jump_dbg_trigger_block_reason = JUMP_BLOCK_NONE;
volatile uint32 jump_dbg_trigger_count = 0;

static uint16 jump_state_elapsed_ms = 0;

static void jump_set_suspend(uint8 engine_suspend, uint8 encoder_suspend);

static void jump_restore_control(void)
{
    jump_set_suspend(0, 0);
    jump_position = 0;
}

static void jump_set_suspend(uint8 engine_suspend, uint8 encoder_suspend)
{
    jump_engine_suspend = engine_suspend;
    jump_encoder_suspend = encoder_suspend;
    jump_stop = engine_suspend;
}

static void jump_set_state(JumpState next_state)
{
    jump_state = next_state;
    jump_dbg_state = (uint8)next_state;
    jump_state_elapsed_ms = 0;
    jump_dbg_elapsed_ms = 0;
}

static int jump_limit_pwm1(int pwm1)
{
    if (pwm1 < JUMP_SERVO_MIN_PWM)
    {
        return JUMP_SERVO_MIN_PWM;
    }
    if (pwm1 > JUMP_SERVO_MAX_PWM)
    {
        return JUMP_SERVO_MAX_PWM;
    }
    return pwm1;
}

static void jump_drive_symmetric_pwm(int pwm1)
{
    int pwm2;

    pwm1 = jump_limit_pwm1(pwm1);
    pwm2 = JUMP_SERVO_SUM - pwm1;

    engine_left_maintain(pwm1, pwm2);
    engine_right_maintain(pwm1, pwm2);
}

static void jump_drive_symmetric_xy(float x, float y)
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

uint8 jump_start(void)
{
    if (jump_state != JUMP_FREE)
    {
        jump_set_trigger_block_reason(JUMP_BLOCK_BUSY);
        return 0;
    }

    jump_position = 1;
    jump_set_suspend(1, 1);
    small_driver_set_duty(0, 0);
    jump_dbg_trigger_count++;
    jump_set_trigger_block_reason(JUMP_BLOCK_STARTED);
    jump_set_state(JUMP_PREPARE);
    return 1;
}

uint8 jump_is_active(void)
{
    return (jump_state != JUMP_FREE) ? 1 : 0;
}

uint8 jump_should_suspend_engine(void)
{
    return jump_engine_suspend;
}

uint8 jump_should_suspend_encoder(void)
{
    return jump_encoder_suspend;
}

void jump_set_trigger_block_reason(JumpTriggerBlockReason reason)
{
    jump_dbg_trigger_block_reason = (uint8)reason;
}

void jump_process_control(float *current_x, float *current_y)
{
    if (jump_state == JUMP_FREE)
    {
        jump_dbg_state = JUMP_FREE;
        jump_dbg_elapsed_ms = 0;
        return;
    }

    jump_state_elapsed_ms++;
    jump_dbg_elapsed_ms = jump_state_elapsed_ms;

    switch (jump_state)
    {
        case JUMP_PREPARE:
            *current_x = JUMP_PREPARE_X;
            *current_y = JUMP_PREPARE_Y;
            jump_drive_symmetric_xy(*current_x, *current_y);
            if (jump_state_elapsed_ms >= JUMP_PREPARE_MS)
            {
                jump_set_state(JUMP_BURST);
            }
            break;

        case JUMP_BURST:
            jump_drive_symmetric_pwm(JUMP_BURST_PWM);
            if (jump_state_elapsed_ms >= JUMP_BURST_MS)
            {
                jump_set_state(JUMP_AIR_RETRACT);
            }
            break;

        case JUMP_AIR_RETRACT:
            *current_x = JUMP_AIR_RETRACT_X;
            *current_y = JUMP_AIR_RETRACT_Y;
            jump_drive_symmetric_xy(*current_x, *current_y);
            if (jump_state_elapsed_ms >= JUMP_AIR_RETRACT_MS)
            {
                jump_set_state(JUMP_EXE_BUFFER);
            }
            break;

        case JUMP_EXE_BUFFER:
            *current_x = JUMP_EXE_BUFFER_X;
            *current_y = JUMP_EXE_BUFFER_Y;
            jump_drive_symmetric_xy(*current_x, *current_y);
            if (IMU_data.accel[2] >= JUMP_LAND_ACCEL_G || jump_state_elapsed_ms >= JUMP_LANDING_MAX_MS)
            {
                jump_set_state(JUMP_RECOVER);
            }
            break;

        case JUMP_RECOVER:
            *current_x = JUMP_RECOVER_X;
            *current_y = JUMP_RECOVER_Y;
            jump_drive_symmetric_xy(*current_x, *current_y);
            if (jump_state_elapsed_ms >= JUMP_RECOVER_MS)
            {
                jump_restore_control();
                jump_set_state(JUMP_FREE);
            }
            break;

        case JUMP_FREE:
        default:
            jump_restore_control();
            jump_set_state(JUMP_FREE);
            break;
    }
}

void jump_abort(void)
{
    jump_drive_symmetric_xy(JUMP_RECOVER_X, JUMP_RECOVER_Y);
    jump_restore_control();
    jump_set_state(JUMP_FREE);
}

void jump_force_idle(void)
{
    jump_restore_control();
    jump_set_state(JUMP_FREE);
    jump_set_trigger_block_reason(JUMP_BLOCK_NONE);
}
