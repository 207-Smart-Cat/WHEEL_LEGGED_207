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
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, VISION_ALIGN_STABLE_ERROR_PX, 10.0f, 10U);
        assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_STABLE_WAIT);
        assert(VisionAlignCal_GetStableCount(&cal) == (uint8)(i + 1U));
    }

    VisionAlignCal_Update(&cal, 1U, 1U, frame++, VISION_ALIGN_STABLE_ERROR_PX + 1, 10.0f, 10U);
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_STABLE_WAIT);
    assert(VisionAlignCal_GetStableCount(&cal) == 0U);

    for (uint8 i = 0U; i < VISION_ALIGN_STABLE_WINDOW_FRAMES; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, -VISION_ALIGN_STABLE_ERROR_PX, 10.0f, 10U);
    }
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_SAMPLING);
    assert(VisionAlignCal_GetStableCount(&cal) == VISION_ALIGN_STABLE_WINDOW_FRAMES);
}

static void sampling_accepts_consecutive_centered_frames_from_one_side(void)
{
    VisionAlignCal_t cal;
    uint32 frame = 1U;

    assert(VISION_ALIGN_SAMPLE_COUNT_TARGET == 14U);

    VisionAlignCal_Reset(&cal);
    for (uint8 i = 0U; i < VISION_ALIGN_STABLE_WINDOW_FRAMES; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, 12, 170.0f, 10U);
    }

    VisionAlignCal_Update(&cal, 1U, 1U, frame++, VISION_ALIGN_SAMPLE_ERROR_PX + 1, 178.0f, 10U);
    assert(VisionAlignCal_GetSampleCount(&cal) == 0U);

    for (uint8 i = 0U; i < VISION_ALIGN_SAMPLE_COUNT_TARGET; i++)
    {
        float yaw = (i & 1U) ? -179.0f : 179.0f;
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, -3, yaw, 10U);
    }

    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_DONE);
    assert(VisionAlignCal_ResultValid(&cal) == 1U);
    assert(VisionAlignCal_GetLeftSampleCount(&cal) == VISION_ALIGN_SAMPLE_COUNT_TARGET);
    assert(VisionAlignCal_GetRightSampleCount(&cal) == 0U);
    assert_close(VisionAlignCal_GetResultYaw(&cal), 180.0f);
}

static void sampling_requires_consecutive_centered_frames(void)
{
    VisionAlignCal_t cal;
    uint32 frame = 1U;

    VisionAlignCal_Reset(&cal);
    for (uint8 i = 0U; i < VISION_ALIGN_STABLE_WINDOW_FRAMES; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, 10, 30.0f, 10U);
    }

    for (uint8 i = 0U; i < 5U; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, -2, 40.0f, 10U);
    }
    assert(VisionAlignCal_GetSampleCount(&cal) == 5U);

    VisionAlignCal_Update(&cal, 1U, 1U, frame++, VISION_ALIGN_SAMPLE_ERROR_PX + 1, 45.0f, 10U);
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_SAMPLING);
    assert(VisionAlignCal_GetSampleCount(&cal) == 0U);
    assert(VisionAlignCal_ResultValid(&cal) == 0U);

    for (uint8 i = 0U; i < VISION_ALIGN_SAMPLE_COUNT_TARGET; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, 2, 50.0f, 10U);
    }
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_DONE);
    assert(VisionAlignCal_GetSampleCount(&cal) == VISION_ALIGN_SAMPLE_COUNT_TARGET);
    assert_close(VisionAlignCal_GetResultYaw(&cal), 50.0f);
}

static void timeout_finishes_only_on_a_centered_frame(void)
{
    VisionAlignCal_t cal;
    uint32 frame = 1U;

    assert(VISION_ALIGN_COMPLETE_TIMEOUT_MS == 800U);
    VisionAlignCal_Reset(&cal);
    for (uint8 i = 0U; i < VISION_ALIGN_STABLE_WINDOW_FRAMES; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, 10, 30.0f, 10U);
    }

    for (uint8 i = 0U; i < 60U; i++)
    {
        VisionAlignCal_Update(&cal, 1U, 1U, frame++, VISION_ALIGN_SAMPLE_ERROR_PX + 1, 31.0f, 10U);
    }
    assert(cal.elapsed_ms == VISION_ALIGN_COMPLETE_TIMEOUT_MS);
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_SAMPLING);
    assert(VisionAlignCal_ResultValid(&cal) == 0U);

    VisionAlignCal_Update(&cal, 1U, 1U, frame++, VISION_ALIGN_SAMPLE_ERROR_PX, 33.0f, 10U);
    assert(VisionAlignCal_GetState(&cal) == VISION_ALIGN_CAL_DONE);
    assert(VisionAlignCal_ResultValid(&cal) == 1U);
    assert_close(VisionAlignCal_GetResultYaw(&cal), 33.0f);
}

int main(void)
{
    stable_wait_requires_full_window();
    sampling_accepts_consecutive_centered_frames_from_one_side();
    sampling_requires_consecutive_centered_frames();
    timeout_finishes_only_on_a_centered_frame();
    puts("vision_align_calibration tests passed");
    return 0;
}
