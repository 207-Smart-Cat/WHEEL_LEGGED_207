#ifndef VISION_ALIGN_CALIBRATION_H
#define VISION_ALIGN_CALIBRATION_H

#include "zf_common_typedef.h"
#include "course3_tuning.h"

typedef enum
{
    VISION_ALIGN_CAL_IDLE = 0,
    VISION_ALIGN_CAL_ALIGNING,
    VISION_ALIGN_CAL_STABLE_WAIT,
    VISION_ALIGN_CAL_SAMPLING,
    VISION_ALIGN_CAL_DONE
} VisionAlignCalState_t;

typedef struct
{
    VisionAlignCalState_t state;
    uint32 last_frame_seq;
    uint8 stable_count;
    uint8 sample_count;
    uint8 left_sample_count;
    uint8 right_sample_count;
    uint8 result_valid;
    uint8 has_anchor;
    float anchor_yaw;
    float sample_delta_sum;
    float result_yaw;
} VisionAlignCal_t;

void VisionAlignCal_Reset(VisionAlignCal_t *cal);
void VisionAlignCal_Update(VisionAlignCal_t *cal,
                           uint8 enabled,
                           uint8 lane_valid,
                           uint32 frame_seq,
                           int16 lane_error_px,
                           float current_yaw);
VisionAlignCalState_t VisionAlignCal_GetState(const VisionAlignCal_t *cal);
uint8 VisionAlignCal_GetStableCount(const VisionAlignCal_t *cal);
uint8 VisionAlignCal_GetSampleCount(const VisionAlignCal_t *cal);
uint8 VisionAlignCal_GetLeftSampleCount(const VisionAlignCal_t *cal);
uint8 VisionAlignCal_GetRightSampleCount(const VisionAlignCal_t *cal);
uint8 VisionAlignCal_ResultValid(const VisionAlignCal_t *cal);
float VisionAlignCal_GetResultYaw(const VisionAlignCal_t *cal);

#endif
