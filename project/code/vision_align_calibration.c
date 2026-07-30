#include "vision_align_calibration.h"

static int vision_align_abs_i16(int16 value)
{
    return (value < 0) ? -value : value;
}

static float vision_align_wrap_angle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

void VisionAlignCal_Reset(VisionAlignCal_t *cal)
{
    cal->state = VISION_ALIGN_CAL_IDLE;
    cal->last_frame_seq = 0U;
    cal->stable_count = 0U;
    cal->sample_count = 0U;
    cal->left_sample_count = 0U;
    cal->right_sample_count = 0U;
    cal->result_valid = 0U;
    cal->has_anchor = 0U;
    cal->elapsed_ms = 0U;
    cal->anchor_yaw = 0.0f;
    cal->sample_delta_sum = 0.0f;
    cal->result_yaw = 0.0f;
}

void VisionAlignCal_Update(VisionAlignCal_t *cal,
                           uint8 enabled,
                           uint8 lane_valid,
                           uint32 frame_seq,
                           int16 lane_error_px,
                           float current_yaw,
                           uint16 dt_ms)
{
    uint8 within_stable;
    uint8 within_sample;

    if (!enabled)
    {
        VisionAlignCal_Reset(cal);
        return;
    }

    if (cal->state == VISION_ALIGN_CAL_IDLE)
    {
        cal->state = VISION_ALIGN_CAL_ALIGNING;
    }

    if (cal->elapsed_ms < VISION_ALIGN_COMPLETE_TIMEOUT_MS)
    {
        uint16 remaining_ms = (uint16)(VISION_ALIGN_COMPLETE_TIMEOUT_MS - cal->elapsed_ms);
        cal->elapsed_ms += (dt_ms < remaining_ms) ? dt_ms : remaining_ms;
    }

    if (frame_seq == cal->last_frame_seq)
    {
        return;
    }
    cal->last_frame_seq = frame_seq;

    if (cal->state == VISION_ALIGN_CAL_DONE)
    {
        return;
    }

    within_stable = (lane_valid && vision_align_abs_i16(lane_error_px) <= VISION_ALIGN_STABLE_ERROR_PX) ? 1U : 0U;
    within_sample = (lane_valid && vision_align_abs_i16(lane_error_px) <= VISION_ALIGN_SAMPLE_ERROR_PX) ? 1U : 0U;

    if (cal->state == VISION_ALIGN_CAL_ALIGNING)
    {
        cal->state = VISION_ALIGN_CAL_STABLE_WAIT;
    }

    if (cal->state == VISION_ALIGN_CAL_STABLE_WAIT)
    {
        if (within_stable)
        {
            if (cal->stable_count < VISION_ALIGN_STABLE_WINDOW_FRAMES)
            {
                cal->stable_count++;
            }
            if (cal->stable_count >= VISION_ALIGN_STABLE_WINDOW_FRAMES)
            {
                cal->state = VISION_ALIGN_CAL_SAMPLING;
                cal->sample_count = 0U;
                cal->left_sample_count = 0U;
                cal->right_sample_count = 0U;
                cal->has_anchor = 0U;
                cal->sample_delta_sum = 0.0f;
            }
        }
        else
        {
            cal->stable_count = 0U;
        }
        return;
    }

    if (cal->state == VISION_ALIGN_CAL_SAMPLING)
    {
        if (!within_stable)
        {
            cal->state = VISION_ALIGN_CAL_STABLE_WAIT;
            cal->stable_count = 0U;
            cal->sample_count = 0U;
            cal->left_sample_count = 0U;
            cal->right_sample_count = 0U;
            cal->has_anchor = 0U;
            cal->sample_delta_sum = 0.0f;
            return;
        }

        if (!within_sample)
        {
            cal->sample_count = 0U;
            cal->left_sample_count = 0U;
            cal->right_sample_count = 0U;
            cal->has_anchor = 0U;
            cal->sample_delta_sum = 0.0f;
            return;
        }

        if (cal->elapsed_ms >= VISION_ALIGN_COMPLETE_TIMEOUT_MS)
        {
            cal->result_yaw = vision_align_wrap_angle(current_yaw);
            cal->result_valid = 1U;
            cal->state = VISION_ALIGN_CAL_DONE;
            return;
        }

        if (lane_error_px < 0)
        {
            cal->left_sample_count++;
        }
        else if (lane_error_px > 0)
        {
            cal->right_sample_count++;
        }

        if (!cal->has_anchor)
        {
            cal->anchor_yaw = current_yaw;
            cal->has_anchor = 1U;
            cal->sample_delta_sum = 0.0f;
        }
        else
        {
            cal->sample_delta_sum += vision_align_wrap_angle(current_yaw - cal->anchor_yaw);
        }

        if (cal->sample_count < VISION_ALIGN_SAMPLE_COUNT_TARGET)
        {
            cal->sample_count++;
        }

        if (cal->sample_count >= VISION_ALIGN_SAMPLE_COUNT_TARGET)
        {
            cal->result_yaw = vision_align_wrap_angle(cal->anchor_yaw +
                cal->sample_delta_sum / (float)cal->sample_count);
            cal->result_valid = 1U;
            cal->state = VISION_ALIGN_CAL_DONE;
        }
    }
}

VisionAlignCalState_t VisionAlignCal_GetState(const VisionAlignCal_t *cal)
{
    return cal->state;
}

uint8 VisionAlignCal_GetStableCount(const VisionAlignCal_t *cal)
{
    return cal->stable_count;
}

uint8 VisionAlignCal_GetSampleCount(const VisionAlignCal_t *cal)
{
    return cal->sample_count;
}

uint8 VisionAlignCal_GetLeftSampleCount(const VisionAlignCal_t *cal)
{
    return cal->left_sample_count;
}

uint8 VisionAlignCal_GetRightSampleCount(const VisionAlignCal_t *cal)
{
    return cal->right_sample_count;
}

uint8 VisionAlignCal_ResultValid(const VisionAlignCal_t *cal)
{
    return cal->result_valid;
}

float VisionAlignCal_GetResultYaw(const VisionAlignCal_t *cal)
{
    return cal->result_yaw;
}
