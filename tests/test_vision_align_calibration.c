#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "vision_align_calibration.h"

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static void stable_wait_requires_full_window(void)
{
    VisionAlignCal_t cal;
    uint32 frame = 1U;

    VisionAlignCal_Reset(&cal);

    for (uint8 i = 0U; i < (VISION_ALIGN_STABLE_WINDOW_FRAMES - 1U); i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, 29, 10.0f);
        assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_STABLE_WAIT);
        assert(VisionAlignCal_GetStableCount(&cal) == (uint8)(i + 1U));
    }

    VisionAlignCal_Update(&cal, 1U, 1U, frame++, 31, 10.0f);
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_STABLE_WAIT);
    assert(VisionAlignCal_GetStableCount(&cal) == 0U);

    for (uint8 i = 0U; i < VISION_ALIGN_STABLE_WINDOW_FRAMES; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, -30, 10.0f);
    }
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_SAMPLING);
    assert(VisionAlignCal_GetStableCount(&cal) == VISION_ALIGN_STABLE_WINDOW_FRAMES);
}

static void sampling_uses_center_window_and_wrap_safe_average(void)
{
    VisionAlignCal_t cal;
    uint32 frame = 1U;

    VisionAlignCal_Reset(&cal);
    for (uint8 i = 0U; i < VISION_ALIGN_STABLE_WINDOW_FRAMES; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, 12, 170.0f);
    }

    VisionAlignCal_Update(&cal, 1U, 1U, frame++, 8, 178.0f);
    assert(VisionAlignCal_GetSampleCount(&cal) == 0U);

    for (uint8 i = 0U; i < VISION_ALIGN_SIDE_SAMPLE_TARGET; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, -3, 179.0f);
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, 3, -179.0f);
    }

    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_DONE);
    assert(VisionAlignCal_ResultValid(&cal) == 1U);
    assert_close(VisionAlignCal_GetResultYaw(&cal), 180.0f);
}

static void sampling_requires_five_samples_on_each_side(void)
{
    VisionAlignCal_t cal;
    uint32 frame = 1U;

    VisionAlignCal_Reset(&cal);
    for (uint8 i = 0U; i < VISION_ALIGN_STABLE_WINDOW_FRAMES; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, 10, 30.0f);
    }

    for (uint8 i = 0U; i < VISION_ALIGN_SAMPLE_COUNT_TARGET; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, -2, 40.0f);
    }
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_SAMPLING);
    assert(VisionAlignCal_GetSampleCount(&cal) == VISION_ALIGN_SIDE_SAMPLE_TARGET);
    assert(VisionAlignCal_ResultValid(&cal) == 0U);

    for (uint8 i = 0U; i < (VISION_ALIGN_SIDE_SAMPLE_TARGET - 1U); i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, 2, 50.0f);
    }
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_SAMPLING);
    assert(VisionAlignCal_GetSampleCount(&cal) == (VISION_ALIGN_SAMPLE_COUNT_TARGET - 1U));

    VisionAlignCal_Update(&cal, 1U, 1U, frame++, 2, 50.0f);
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_DONE);
    assert(VisionAlignCal_GetSampleCount(&cal) == VISION_ALIGN_SAMPLE_COUNT_TARGET);
    assert_close(VisionAlignCal_GetResultYaw(&cal), 45.0f);
}

int main(void)
{
    stable_wait_requires_full_window();
    sampling_uses_center_window_and_wrap_safe_average();
    sampling_requires_five_samples_on_each_side();
    puts("vision_align_calibration tests passed");
    return 0;
}
