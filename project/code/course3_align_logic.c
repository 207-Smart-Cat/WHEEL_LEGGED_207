#include "course3_align_logic.h"

#include <math.h>
#include <string.h>

#define COURSE3_ALIGN_PI (3.14159265358979323846f)

static uint16_t align_abs_error(int16_t error_px)
{
    return (error_px < 0) ? (uint16_t)(-error_px) : (uint16_t)error_px;
}

static void align_insert(Course3AlignSample_t *list, uint8_t *count, int16_t error_px, float yaw_deg)
{
    uint8_t index;
    uint8_t limit = *count;

    if (limit < COURSE3_ALIGN_CANDIDATE_COUNT)
    {
        (*count)++;
    }
    else if (align_abs_error(error_px) >= align_abs_error(list[limit - 1U].error_px))
    {
        return;
    }
    else
    {
        limit--;
    }

    index = limit;
    while (index > 0U && align_abs_error(error_px) < align_abs_error(list[index - 1U].error_px))
    {
        list[index] = list[index - 1U];
        index--;
    }
    list[index].error_px = error_px;
    list[index].yaw_deg = yaw_deg;
}

void Course3Align_Reset(Course3AlignSamples_t *samples)
{
    memset(samples, 0, sizeof(*samples));
    samples->zero_to_left = 1U;
}

void Course3Align_AddSample(Course3AlignSamples_t *samples, int16_t error_px, float yaw_deg)
{
    if (error_px < 0)
    {
        align_insert(samples->left, &samples->left_count, error_px, yaw_deg);
    }
    else if (error_px > 0)
    {
        align_insert(samples->right, &samples->right_count, error_px, yaw_deg);
    }
    else if (samples->left_count < samples->right_count ||
             (samples->left_count == samples->right_count && samples->zero_to_left))
    {
        align_insert(samples->left, &samples->left_count, error_px, yaw_deg);
        samples->zero_to_left = 0U;
    }
    else
    {
        align_insert(samples->right, &samples->right_count, error_px, yaw_deg);
        samples->zero_to_left = 1U;
    }
}

uint8_t Course3Align_IsComplete(const Course3AlignSamples_t *samples)
{
    return ((samples->left_count + samples->right_count >= COURSE3_ALIGN_CANDIDATE_COUNT) &&
            samples->left_count >= COURSE3_ALIGN_SIDE_SAMPLE_COUNT &&
            samples->right_count >= COURSE3_ALIGN_SIDE_SAMPLE_COUNT) ? 1U : 0U;
}

float Course3Align_ComputeYaw(const Course3AlignSamples_t *samples)
{
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    uint8_t i;

    for (i = 0U; i < COURSE3_ALIGN_SIDE_SAMPLE_COUNT; i++)
    {
        float radians = samples->left[i].yaw_deg * COURSE3_ALIGN_PI / 180.0f;
        sin_sum += sinf(radians);
        cos_sum += cosf(radians);
        radians = samples->right[i].yaw_deg * COURSE3_ALIGN_PI / 180.0f;
        sin_sum += sinf(radians);
        cos_sum += cosf(radians);
    }
    return atan2f(sin_sum, cos_sum) * 180.0f / COURSE3_ALIGN_PI;
}
