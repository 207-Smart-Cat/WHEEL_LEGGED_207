#include "jump_control.h"

#include "engine.h"
#include "imu.h"
#include "jump_action_profile.h"
#include "landing_detector.h"

#define JUMP_SERVO_SUM          (1500)
#define JUMP_SERVO_MIN_PWM      (150)
#define JUMP_SERVO_MAX_PWM      (1350)
#define JUMP_PREPARE_START_PWM  (JUMP_SERVO_SUM / 2)
#define JUMP_TASK_PERIOD_MS     (5U)
#define JUMP_BASELINE_ALPHA     (0.10f)

volatile JumpState jump_state = JUMP_FREE;
volatile uint8 jump_engine_suspend = 0U;
volatile uint8 jump_encoder_suspend = 0U;
volatile uint8 jump_dbg_state = JUMP_FREE;
volatile uint16 jump_dbg_elapsed_ms = 0U;
volatile uint8 jump_dbg_trigger_block_reason = JUMP_BLOCK_NONE;
volatile uint32 jump_dbg_trigger_count = 0U;

static const JumpActionProfileConfig_t *jump_profile = 0;
static LandingDetector_t jump_landing_detector;
static float jump_ground_accel_z_g = 0.0f;
static uint16 jump_state_elapsed_ms = 0U;
static uint8 jump_ground_sample_count = 0U;

static void jump_clear_motion_suspend(void)
{
    /* Wheel feedback and speed/yaw control continue throughout the jump. */
    jump_engine_suspend = 0U;
    jump_encoder_suspend = 0U;
    jump_stop = 0;
    jump_position = 0;
}

static void jump_set_state(JumpState next_state)
{
    jump_state = next_state;
    jump_dbg_state = (uint8)next_state;
    jump_state_elapsed_ms = 0U;
    jump_dbg_elapsed_ms = 0U;
}

static void jump_observe_ground_accel(float accel_z_g)
{
    if (!LandingDetector_IsBaselineValid(accel_z_g))
    {
        return;
    }

    if (jump_ground_sample_count == 0U)
    {
        jump_ground_accel_z_g = accel_z_g;
    }
    else
    {
        jump_ground_accel_z_g += JUMP_BASELINE_ALPHA *
                                 (accel_z_g - jump_ground_accel_z_g);
    }
    if (jump_ground_sample_count < 255U)
    {
        jump_ground_sample_count++;
    }
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
    return (pwm >= 0.0f) ? (int)(pwm + 0.5f) : (int)(pwm - 0.5f);
}

uint8 JumpAction_Start(JumpActionProfile_e profile)
{
    const JumpActionProfileConfig_t *selected_profile;

    if (jump_state != JUMP_FREE)
    {
        jump_set_trigger_block_reason(JUMP_BLOCK_BUSY);
        return 0U;
    }

    jump_observe_ground_accel(IMU_data.accel[2]);
    selected_profile = JumpActionProfile_Get(profile);
    if (!JumpActionProfile_IsValid(selected_profile) ||
        jump_ground_sample_count == 0U ||
        !LandingDetector_IsBaselineValid(jump_ground_accel_z_g))
    {
        jump_set_trigger_block_reason(JUMP_BLOCK_NOT_ARMED);
        return 0U;
    }

    jump_profile = selected_profile;
    jump_clear_motion_suspend();
    jump_dbg_trigger_count++;
    jump_set_trigger_block_reason(JUMP_BLOCK_STARTED);
    jump_set_state(JUMP_PREPARE);
    return 1U;
}

JumpActionResult_e JumpAction_Task5ms(float accel_z_g)
{
    LandingDetectorState_e landing_state;

    if (jump_state == JUMP_FREE)
    {
        jump_observe_ground_accel(accel_z_g);
        jump_dbg_state = JUMP_FREE;
        jump_dbg_elapsed_ms = 0U;
        return JUMP_ACTION_RESULT_NONE;
    }

    jump_clear_motion_suspend();
    jump_state_elapsed_ms = (uint16)(jump_state_elapsed_ms + JUMP_TASK_PERIOD_MS);
    jump_dbg_elapsed_ms = jump_state_elapsed_ms;

    switch (jump_state)
    {
        case JUMP_PREPARE:
            jump_observe_ground_accel(accel_z_g);
            jump_drive_symmetric_pwm(
                jump_calc_prepare_pwm(jump_state_elapsed_ms,
                                      jump_profile->prepare_ms,
                                      jump_profile->prepare_pwm));
            if (jump_state_elapsed_ms >= jump_profile->prepare_ms)
            {
                LandingDetector_Init(&jump_landing_detector,
                                     jump_ground_accel_z_g);
                LandingDetector_BeginAirborne(&jump_landing_detector);
                jump_set_state(JUMP_BURST);
            }
            break;

        case JUMP_BURST:
            jump_drive_symmetric_pwm(jump_profile->burst_pwm);
            (void)LandingDetector_Update(&jump_landing_detector, accel_z_g);
            if (jump_state_elapsed_ms >= jump_profile->burst_ms)
            {
                jump_set_state(JUMP_AIR_RETRACT);
            }
            break;

        case JUMP_AIR_RETRACT:
            jump_drive_symmetric_pwm(jump_profile->retract_pwm);
            (void)LandingDetector_Update(&jump_landing_detector, accel_z_g);
            if (jump_state_elapsed_ms >=
                (uint16)(jump_profile->retract_fast_ms +
                         jump_profile->retract_hold_ms))
            {
                jump_set_state(JUMP_EXE_BUFFER);
            }
            break;

        case JUMP_EXE_BUFFER:
            jump_drive_symmetric_pwm(jump_profile->buffer_pwm);
            landing_state = LandingDetector_Update(&jump_landing_detector,
                                                   accel_z_g);
            if (landing_state == LANDING_DETECTOR_LANDED)
            {
                jump_set_state(JUMP_RECOVER);
                return JUMP_ACTION_RESULT_LANDED;
            }
            if (landing_state == LANDING_DETECTOR_TIMEOUT)
            {
                jump_drive_symmetric_pwm(jump_profile->recover_pwm);
                jump_set_state(JUMP_FREE);
                return JUMP_ACTION_RESULT_FAULT;
            }
            break;

        case JUMP_RECOVER:
            jump_drive_symmetric_pwm(jump_profile->recover_pwm);
            if (jump_state_elapsed_ms >= jump_profile->recover_ms)
            {
                jump_set_state(JUMP_FREE);
                return JUMP_ACTION_RESULT_COMPLETE;
            }
            break;

        case JUMP_END:
        case JUMP_FREE:
        default:
            jump_clear_motion_suspend();
            jump_set_state(JUMP_FREE);
            break;
    }

    return JUMP_ACTION_RESULT_NONE;
}

uint8 JumpAction_BaselineReady(void)
{
    return (jump_ground_sample_count > 0U &&
            LandingDetector_IsBaselineValid(jump_ground_accel_z_g)) ? 1U : 0U;
}

uint8 jump_start(void)
{
    return JumpAction_Start(JUMP_ACTION_PROFILE_FIRST);
}

uint8 jump_is_active(void)
{
    return (jump_state != JUMP_FREE) ? 1U : 0U;
}

uint8 jump_should_suspend_engine(void)
{
    return jump_engine_suspend;
}

uint8 jump_should_suspend_encoder(void)
{
    /* Encoder acquisition remains enabled; only triple-jump distance freezes. */
    return 0U;
}

void jump_set_trigger_block_reason(JumpTriggerBlockReason reason)
{
    jump_dbg_trigger_block_reason = (uint8)reason;
}

void jump_process_control(float *current_x, float *current_y)
{
    static uint8 task_divider = 0U;

    (void)current_x;
    (void)current_y;
    task_divider++;
    if (task_divider >= JUMP_TASK_PERIOD_MS)
    {
        task_divider = 0U;
        (void)JumpAction_Task5ms(IMU_data.accel[2]);
    }
}

void JumpAction_Abort(void)
{
    const JumpActionProfileConfig_t *profile = jump_profile;

    if (profile == 0)
    {
        profile = JumpActionProfile_Get(JUMP_ACTION_PROFILE_FIRST);
    }
    jump_drive_symmetric_pwm(profile->recover_pwm);
    jump_clear_motion_suspend();
    jump_set_state(JUMP_FREE);
}

void jump_abort(void)
{
    JumpAction_Abort();
}

void jump_force_idle(void)
{
    jump_clear_motion_suspend();
    jump_set_state(JUMP_FREE);
    jump_set_trigger_block_reason(JUMP_BLOCK_NONE);
}
