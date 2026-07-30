#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "course3_tuning.h"
#include "vision_control.h"

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static void test_single_filter_pixel_mapping(void)
{
    VisionControlState_t state;

    vision_control_reset(&state);
    assert_close(vision_control_update(&state, 5),
                 VISION_SIGN * 5.0f * VISION_PIXEL_TO_ANGLE_P_DEG_PER_PX);
    assert_close(state.filtered_offset_deg, state.raw_offset_deg);
    assert_close(vision_control_update(&state, -5), 1.5f);

    vision_control_reset(&state);
    assert_close(vision_control_update(&state, 1000), -VISION_MAX_ANGLE_OFFSET_DEG);
    assert_close(state.raw_offset_deg, -VISION_MAX_ANGLE_OFFSET_DEG);
    assert_close(state.filtered_offset_deg, -VISION_MAX_ANGLE_OFFSET_DEG);
}

static void test_yaw_rate_direction_and_damping(void)
{
    VisionTurnControlState_t state;
    float vision_yaw_rate;

    vision_turn_control_reset(&state);
    vision_yaw_rate = COURSE3_VISION_YAW_RATE_SIGN * -6.0f;
    assert(vision_yaw_rate > 0.0f);
    (void)vision_turn_control_update(&state, 0.0f, vision_yaw_rate,
                                     COURSE3_VISION_CONTROL_DT_S, 1U);
    assert_close(state.d_output, -COURSE3_VISION_DIRECTION_D * vision_yaw_rate);
    if (COURSE3_VISION_DIRECTION_D > 0.0f)
    {
        assert(state.output < 0.0f);
    }

    vision_yaw_rate = COURSE3_VISION_YAW_RATE_SIGN * 6.0f;
    assert(vision_yaw_rate < 0.0f);
    (void)vision_turn_control_update(&state, 0.0f, vision_yaw_rate,
                                     COURSE3_VISION_CONTROL_DT_S, 1U);
    assert_close(state.d_output, -COURSE3_VISION_DIRECTION_D * vision_yaw_rate);
    if (COURSE3_VISION_DIRECTION_D > 0.0f)
    {
        assert(state.output > 0.0f);
    }
}

static void test_conditional_integral(void)
{
    VisionTurnControlState_t state;
    VisionTurnControlState_t coarse_state;
    int i;

    vision_turn_control_reset(&state);
    assert_close(vision_turn_control_update(&state, 3.0f, 0.0f,
                                            COURSE3_VISION_CONTROL_DT_S, 1U),
                 COURSE3_VISION_DIRECTION_P * 3.0f);
    assert_close(state.integral_deg_s, 0.0f);
    assert_close(state.i_output, 0.0f);

    vision_turn_control_reset(&state);
    for (i = 0; i < 1000; ++i)
    {
        (void)vision_turn_control_update(&state, 0.3f, 0.0f, 0.001f, 1U);
    }
    assert_close(state.integral_deg_s, 0.3f);
    assert_close(state.i_output, COURSE3_VISION_DIRECTION_I * 0.3f);

    vision_turn_control_reset(&coarse_state);
    for (i = 0; i < 100; ++i)
    {
        (void)vision_turn_control_update(&coarse_state, 0.3f, 0.0f, 0.010f, 1U);
    }
    assert_close(coarse_state.integral_deg_s, state.integral_deg_s);
    assert_close(coarse_state.i_output, state.i_output);

    (void)vision_turn_control_update(&state, -0.3f, 0.0f, 0.001f, 1U);
    assert_close(state.integral_deg_s, 0.0f);
    assert_close(state.i_output, 0.0f);
    assert(state.output < 0.0f);

    (void)vision_turn_control_update(&state, 0.3f, 0.0f, 0.001f, 1U);
    (void)vision_turn_control_update(&state, 0.1f, 0.0f, 0.001f, 1U);
    assert_close(state.integral_deg_s, 0.0f);

    (void)vision_turn_control_update(&state, 0.3f, 0.0f, 0.001f, 1U);
    (void)vision_turn_control_update(&state, 0.3f, 9.0f, 0.001f, 1U);
    assert_close(state.integral_deg_s, 0.0f);
    assert_close(state.output,
                 COURSE3_VISION_DIRECTION_P * 0.3f -
                 COURSE3_VISION_DIRECTION_D * 9.0f);

    (void)vision_turn_control_update(&state, 0.3f, 0.0f, 0.001f, 1U);
    assert(state.integral_deg_s > 0.0f);
    assert_close(vision_turn_control_update(&state, 0.3f, 0.0f, 0.001f, 0U), 0.0f);
    assert_close(state.integral_deg_s, 0.0f);
}

static void test_integral_and_total_limits(void)
{
    VisionTurnControlState_t state;
    int i;

    vision_turn_control_reset(&state);
    for (i = 0; i < 10000; ++i)
    {
        (void)vision_turn_control_update(&state, 1.5f, 0.0f, 0.001f, 1U);
    }
    assert(fabsf(state.i_output) <= COURSE3_VISION_I_OUTPUT_LIMIT);
    assert(fabsf(state.output) <= COURSE3_VISION_TURN_PWM_LIMIT);

    assert_close(vision_turn_control_update(&state, 100.0f, 0.0f, 0.001f, 1U),
                 COURSE3_VISION_TURN_PWM_LIMIT);
    assert_close(state.integral_deg_s, 0.0f);
}

int main(void)
{
    test_single_filter_pixel_mapping();
    test_yaw_rate_direction_and_damping();
    test_conditional_integral();
    test_integral_and_total_limits();

    assert_close(vision_control_pd_output(10.0f, 5.0f, 50.0f, 0.875f, 2200.0f), 495.625f);
    assert_close(vision_control_pd_output(300.0f, 0.0f, 50.0f, 0.875f, 2200.0f), 2200.0f);
    assert_close(vision_control_pd_output(-300.0f, 0.0f, 50.0f, 0.875f, 2200.0f), -2200.0f);

    puts("vision control tests passed");
    return 0;
}
