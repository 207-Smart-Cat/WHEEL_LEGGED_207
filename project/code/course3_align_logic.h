#ifndef COURSE3_ALIGN_LOGIC_H
#define COURSE3_ALIGN_LOGIC_H

#include <stdint.h>
#include "course3_tuning.h"

typedef struct
{
    float yaw_deg;
    int16_t error_px;
} Course3AlignSample_t;

typedef struct
{
    Course3AlignSample_t left[COURSE3_ALIGN_CANDIDATE_COUNT];
    Course3AlignSample_t right[COURSE3_ALIGN_CANDIDATE_COUNT];
    uint8_t left_count;
    uint8_t right_count;
    uint8_t zero_to_left;
} Course3AlignSamples_t;

void Course3Align_Reset(Course3AlignSamples_t *samples);
void Course3Align_AddSample(Course3AlignSamples_t *samples, int16_t error_px, float yaw_deg);
uint8_t Course3Align_IsComplete(const Course3AlignSamples_t *samples);
float Course3Align_ComputeYaw(const Course3AlignSamples_t *samples);

#endif
