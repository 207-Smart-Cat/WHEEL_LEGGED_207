#ifndef JUMP_ACTION_PROFILE_H
#define JUMP_ACTION_PROFILE_H

#include <stdint.h>

#include "jump_action_types.h"

typedef struct
{
    int16_t prepare_pwm;
    uint16_t prepare_ms;
    int16_t burst_pwm;
    uint16_t burst_ms;
    int16_t retract_pwm;
    uint16_t retract_fast_ms;
    uint16_t retract_hold_ms;
    int16_t buffer_pwm;
    int16_t recover_pwm;
    uint16_t recover_ms;
} JumpActionProfileConfig_t;

const JumpActionProfileConfig_t *JumpActionProfile_Get(JumpActionProfile_e profile);
uint8_t JumpActionProfile_IsValid(const JumpActionProfileConfig_t *profile);

#endif
