#include "jump_control.h"

#include "engine.h"
#include "imu.h"

#define JUMP_PREPARE_PWM        (370)                 // 独立跳跃流程默认预压PWM，导航FSM另行传入
#define JUMP_BURST_PWM          (1350)                // 爆发起跳阶段直接输出最大伸腿脉宽
#define JUMP_SERVO_SUM          (1500)                // 左右舵机脉宽总和，用于保持机构同步

#define JUMP_SERVO_MIN_PWM      (150)                 // 跳跃专用最小脉宽
#define JUMP_SERVO_MAX_PWM      (1350)                // 跳跃专用最大脉宽
#define JUMP_PREPARE_START_PWM  (JUMP_SERVO_SUM / 2)  // 预压开始时的初始脉宽

#define JUMP_AIR_RETRACT_PWM    (420)                 // 腾空阶段直接PWM收腿
#define JUMP_EXE_BUFFER_PWM     (520)                 // 落地前直接PWM缓冲
#define JUMP_RECOVER_PWM        (420)                 // 恢复阶段舵机脉宽

#define JUMP_PREPARE_MS         (260U)               // 
#define JUMP_BURST_MS           (180U)                // 爆发起跳阶段持续时间(ms)        延长伸腿发力时间
#define JUMP_AIR_RETRACT_MS     (40U)                // 腾空收腿阶段持续时间(ms)
#define JUMP_RECOVER_MS         (50U)                 // 落地恢复阶段持续时间(ms)
#define JUMP_END_MS             (50U)                 // 跳跃结束缓冲时间(ms)

#define JUMP_LANDING_MAX_MS     (600U)                // 最大允许腾空时间(ms)，超时强制进入落地恢复
#define JUMP_LAND_ACCEL_G       (1.0f)                // 落地判定加速度阈值(g)

volatile JumpState jump_state = JUMP_FREE;
volatile uint8 jump_engine_suspend = 0;
volatile uint8 jump_encoder_suspend = 0;
volatile uint8 jump_dbg_state = JUMP_FREE;
volatile uint16 jump_dbg_elapsed_ms = 0;
volatile uint8 jump_dbg_trigger_block_reason = JUMP_BLOCK_NONE;
volatile uint32 jump_dbg_trigger_count = 0;

static uint16 jump_state_elapsed_ms = 0;

static void jump_clear_motion_suspend(void);

static void jump_restore_control(void)
{
    jump_clear_motion_suspend();
    jump_position = 0;
}

static void jump_clear_motion_suspend(void)
{
    jump_engine_suspend = 0;
    jump_encoder_suspend = 0;
    jump_stop = 0;
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

void jump_drive_symmetric_pwm(int pwm1)
{
    int pwm2;

    pwm1 = jump_limit_pwm1(pwm1);
    pwm2 = JUMP_SERVO_SUM - pwm1;

    engine_jump_maintain(pwm1, pwm2);
}

int jump_calc_prepare_pwm(uint16 elapsed_ms, uint16 prepare_ms, int target_pwm)
{
    float progress;
    float pwm;

    if (prepare_ms == 0U || elapsed_ms >= prepare_ms)
    {
        return target_pwm;
    }

    progress = (float)elapsed_ms / (float)prepare_ms;
    pwm = (float)JUMP_PREPARE_START_PWM +
          ((float)target_pwm - (float)JUMP_PREPARE_START_PWM) * progress;

    if (pwm >= 0.0f)
    {
        return (int)(pwm + 0.5f);
    }
    return (int)(pwm - 0.5f);
}

uint8 jump_start(void)
{
    if (jump_state != JUMP_FREE)
    {
        jump_set_trigger_block_reason(JUMP_BLOCK_BUSY);
        return 0;
    }

    jump_clear_motion_suspend();
    jump_position = 0;
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
    return 0;
}

void jump_set_trigger_block_reason(JumpTriggerBlockReason reason)
{
    jump_dbg_trigger_block_reason = (uint8)reason;
}

void jump_process_control(float *current_x, float *current_y)
{
    (void)current_x;
    (void)current_y;

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
            jump_drive_symmetric_pwm(
                jump_calc_prepare_pwm(jump_state_elapsed_ms,
                                      JUMP_PREPARE_MS,
                                      JUMP_PREPARE_PWM));
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
            jump_drive_symmetric_pwm(JUMP_AIR_RETRACT_PWM);
            if (jump_state_elapsed_ms >= JUMP_AIR_RETRACT_MS)
            {
                jump_set_state(JUMP_EXE_BUFFER);
            }
            break;

        case JUMP_EXE_BUFFER:
            jump_drive_symmetric_pwm(JUMP_EXE_BUFFER_PWM);
            if (IMU_data.accel[2] >= 1.5*JUMP_LAND_ACCEL_G || jump_state_elapsed_ms >= JUMP_LANDING_MAX_MS)
            {
                jump_set_state(JUMP_RECOVER);
            }
            break;

        case JUMP_RECOVER:
            jump_drive_symmetric_pwm(JUMP_RECOVER_PWM);
            if (jump_state_elapsed_ms >= JUMP_RECOVER_MS)
            {
                jump_set_state(JUMP_END);
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
    jump_drive_symmetric_pwm(JUMP_RECOVER_PWM);
    jump_restore_control();
    jump_set_state(JUMP_FREE);
}

void jump_force_idle(void)
{
    jump_restore_control();
    jump_set_state(JUMP_FREE);
    jump_set_trigger_block_reason(JUMP_BLOCK_NONE);
}
