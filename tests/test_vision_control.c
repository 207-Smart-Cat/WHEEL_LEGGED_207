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
    assert_close(vision_control_update(&state, 10), -1.44f);

    vision_control_reset(&state);
    assert_close(vision_control_update(&state, 100), -14.4f);

    vision_control_reset(&state);
    state.filtered_offset_deg = 17.0f;
    assert_close(vision_control_update(&state, 100), -11.0f);

    vision_control_reset(&state);
    assert_close(vision_control_update(&state, 1000), -144.0f);

    puts("vision_control tests passed");
    return 0;
}
