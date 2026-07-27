#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

typedef struct
{
    float raw_offset_deg;
    float filtered_offset_deg;
} VisionControlState_t;

void vision_control_reset(VisionControlState_t *state);
float vision_control_update(VisionControlState_t *state, int lane_error_px);

#endif
