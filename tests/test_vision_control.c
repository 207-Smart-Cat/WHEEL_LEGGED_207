#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "vision_control.h"

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    VisionControlState_t state;

    vision_control_reset(&state);
    assert_close(vision_control_update(&state, 5), 0.0f);
    assert_close(vision_control_update(&state, 10), -3.76f);

    vision_control_reset(&state);
    assert_close(vision_control_update(&state, 100), -25.6f);

    vision_control_reset(&state);
    state.filtered_offset_deg = 17.0f;
    assert_close(vision_control_update(&state, 100), -22.2f);

    vision_control_reset(&state);
    assert_close(vision_control_update(&state, 1000), -256.0f);

    vision_control_reset(&state);
    assert_close(vision_control_update(&state, 50), -12.8f);
    assert_close(vision_control_update(&state, 30), -7.84f);

    assert_close(vision_control_pd_output(10.0f, 5.0f, 50.0f, 0.875f, 2200.0f), 495.625f);
    assert_close(vision_control_pd_output(300.0f, 0.0f, 50.0f, 0.875f, 2200.0f), 2200.0f);
    assert_close(vision_control_pd_output(-300.0f, 0.0f, 50.0f, 0.875f, 2200.0f), -2200.0f);

    puts("vision_control tests passed");
    return 0;
}
