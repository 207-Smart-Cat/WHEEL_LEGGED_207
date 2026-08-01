#include "jump_action_profile.h"

#include <stddef.h>

#define JUMP_ACTION_PWM_MIN (150)
#define JUMP_ACTION_PWM_MAX (1350)

static const JumpActionProfileConfig_t jump_action_profiles[] =
{
    {
        370, 200U,
        1300, 180U,
        420, 80U, 50U,
        450,
        400, 50U
    },
    {
        370, 100U,
        1300, 180U,
        420, 80U, 50U,
        450,
        400, 50U
    }
};

static uint8_t jump_action_pwm_is_valid(int16_t pwm)
{
    return (pwm >= JUMP_ACTION_PWM_MIN && pwm <= JUMP_ACTION_PWM_MAX) ? 1U : 0U;
}

const JumpActionProfileConfig_t *JumpActionProfile_Get(JumpActionProfile_e profile)
{
    if (profile != JUMP_ACTION_PROFILE_FIRST &&
        profile != JUMP_ACTION_PROFILE_FOLLOWUP)
    {
        return NULL;
    }
    return &jump_action_profiles[(uint8_t)profile];
}

uint8_t JumpActionProfile_IsValid(const JumpActionProfileConfig_t *profile)
{
    if (profile == NULL)
    {
        return 0U;
    }
    return (jump_action_pwm_is_valid(profile->prepare_pwm) &&
            profile->prepare_ms > 0U &&
            jump_action_pwm_is_valid(profile->burst_pwm) &&
            profile->burst_ms > 0U &&
            jump_action_pwm_is_valid(profile->retract_pwm) &&
            profile->retract_fast_ms > 0U &&
            profile->retract_hold_ms > 0U &&
            jump_action_pwm_is_valid(profile->buffer_pwm) &&
            jump_action_pwm_is_valid(profile->recover_pwm) &&
            profile->recover_ms > 0U) ? 1U : 0U;
}
