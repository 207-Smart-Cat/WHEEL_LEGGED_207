#include <assert.h>

#include "triple_jump_config.h"

static void test_defaults_are_safe_and_valid(void)
{
    TripleJumpConfig_t config;

    TripleJumpConfig_SetDefaults(&config);
    assert(config.x1_m == 0.50f);
    assert(config.x2_m == 0.15f);
    assert(config.x3_m == 0.15f);
    assert(config.speed == 0.0f);
    assert(TripleJump_ConfigIsValid(&config));
}

static void test_crc_changes_with_each_parameter(void)
{
    TripleJumpConfig_t config;
    uint32_t original_crc;

    TripleJumpConfig_SetDefaults(&config);
    original_crc = TripleJumpConfig_CalculateCrc(&config);
    config.x1_m += 0.01f;
    assert(TripleJumpConfig_CalculateCrc(&config) != original_crc);
    TripleJumpConfig_SetDefaults(&config);
    config.x2_m += 0.01f;
    assert(TripleJumpConfig_CalculateCrc(&config) != original_crc);
    TripleJumpConfig_SetDefaults(&config);
    config.x3_m += 0.01f;
    assert(TripleJumpConfig_CalculateCrc(&config) != original_crc);
    TripleJumpConfig_SetDefaults(&config);
    config.speed += 10.0f;
    assert(TripleJumpConfig_CalculateCrc(&config) != original_crc);
}

int main(void)
{
    test_defaults_are_safe_and_valid();
    test_crc_changes_with_each_parameter();
    return 0;
}
