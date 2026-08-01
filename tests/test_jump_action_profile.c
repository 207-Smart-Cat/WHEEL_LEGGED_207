#include <assert.h>

#include "jump_action_profile.h"

static void test_first_jump_profile_matches_p3_tuning(void)
{
    const JumpActionProfileConfig_t *profile =
        JumpActionProfile_Get(JUMP_ACTION_PROFILE_FIRST);

    assert(profile != 0);
    assert(profile->prepare_pwm == 370);
    assert(profile->prepare_ms == 200U);
    assert(profile->burst_pwm == 1300);
    assert(profile->burst_ms == 180U);
    assert(profile->retract_pwm == 420);
    assert(profile->retract_fast_ms == 80U);
    assert(profile->retract_hold_ms == 50U);
    assert(profile->buffer_pwm == 450);
    assert(profile->recover_pwm == 400);
    assert(profile->recover_ms == 50U);
    assert(JumpActionProfile_IsValid(profile));
}

static void test_followup_jump_uses_faster_prepare(void)
{
    const JumpActionProfileConfig_t *first =
        JumpActionProfile_Get(JUMP_ACTION_PROFILE_FIRST);
    const JumpActionProfileConfig_t *followup =
        JumpActionProfile_Get(JUMP_ACTION_PROFILE_FOLLOWUP);

    assert(followup != 0);
    assert(followup->prepare_ms == 100U);
    assert(followup->prepare_pwm == first->prepare_pwm);
    assert(followup->burst_pwm == first->burst_pwm);
    assert(followup->burst_ms == first->burst_ms);
    assert(followup->retract_pwm == first->retract_pwm);
    assert(followup->buffer_pwm == first->buffer_pwm);
    assert(followup->recover_pwm == first->recover_pwm);
    assert(JumpActionProfile_IsValid(followup));
}

static void test_invalid_profile_is_rejected(void)
{
    JumpActionProfileConfig_t profile =
        *JumpActionProfile_Get(JUMP_ACTION_PROFILE_FIRST);

    profile.burst_pwm = 1400;
    assert(!JumpActionProfile_IsValid(&profile));
    profile = *JumpActionProfile_Get(JUMP_ACTION_PROFILE_FIRST);
    profile.prepare_ms = 0U;
    assert(!JumpActionProfile_IsValid(&profile));
    profile = *JumpActionProfile_Get(JUMP_ACTION_PROFILE_FIRST);
    profile.recover_ms = 0U;
    assert(!JumpActionProfile_IsValid(&profile));
}

int main(void)
{
    test_first_jump_profile_matches_p3_tuning();
    test_followup_jump_uses_faster_prepare();
    test_invalid_profile_is_rejected();
    return 0;
}
