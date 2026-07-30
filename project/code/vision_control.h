#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

typedef struct
{
    float raw_offset_deg;
    float filtered_offset_deg;
    int last_error_px;
    unsigned char has_last_error;
} VisionControlState_t;

typedef struct
{
    float integral_deg_s;
    float last_error_deg;
    float p_output;
    float i_output;
    float d_output;
    float output;
    unsigned char has_last_error;
} VisionTurnControlState_t;

void vision_control_reset(VisionControlState_t *state);
float vision_control_update(VisionControlState_t *state, int lane_error_px);
float vision_control_pd_output(float yaw_error_deg,
                               float yaw_rate_dps,
                               float kp,
                               float kd,
                               float output_limit);
void vision_turn_control_reset(VisionTurnControlState_t *state);
float vision_turn_control_update(VisionTurnControlState_t *state,
                                 float yaw_error_deg,
                                 float vision_yaw_rate_dps,
                                 float dt_s,
                                 unsigned char control_valid);

#endif
