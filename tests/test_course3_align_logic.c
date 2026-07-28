#include <assert.h>
#include <math.h>
#include <stdlib.h>

#include "course3_align_logic.h"

int main(void)
{
    Course3AlignSamples_t samples;
    uint8_t i;

    Course3Align_Reset(&samples);
    for (i = 0U; i < 10U; i++)
    {
        Course3Align_AddSample(&samples, -1, 179.0f);
        Course3Align_AddSample(&samples, 1, -179.0f);
    }
    assert(Course3Align_IsComplete(&samples));
    assert(fabsf(fabsf(Course3Align_ComputeYaw(&samples)) - 180.0f) < 1.0f);

    Course3Align_Reset(&samples);
    Course3Align_AddSample(&samples, 0, 10.0f);
    Course3Align_AddSample(&samples, 0, 20.0f);
    assert(samples.left_count == 1U && samples.right_count == 1U);

    Course3Align_Reset(&samples);
    for (i = 0U; i < 20U; i++)
    {
        Course3Align_AddSample(&samples, -20 + (int16_t)i, 0.0f);
        Course3Align_AddSample(&samples, 20 - (int16_t)i, 0.0f);
    }
    assert(samples.left_count == COURSE3_ALIGN_CANDIDATE_COUNT);
    assert(samples.right_count == COURSE3_ALIGN_CANDIDATE_COUNT);
    assert(abs(samples.left[0].error_px) <= abs(samples.left[4].error_px));
    assert(abs(samples.right[0].error_px) <= abs(samples.right[4].error_px));

    return 0;
}
