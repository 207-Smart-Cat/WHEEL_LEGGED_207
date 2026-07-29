#ifndef NAVIGATION_TOUCH_LOGIC_H
#define NAVIGATION_TOUCH_LOGIC_H

#include <stdint.h>

typedef struct
{
    uint8_t high_sample_count;
    uint8_t low_after_high_count;
    uint8_t speed_established;
} NaviJumpTouchLogic_t;

static inline void Navi_JumpTouchLogic_Reset(NaviJumpTouchLogic_t *logic)
{
    logic->high_sample_count = 0;
    logic->low_after_high_count = 0;
    logic->speed_established = 0;
}

static inline uint8_t Navi_JumpTouchLogic_UpdateSignals(
    NaviJumpTouchLogic_t *logic,
    uint8_t high_speed_confirmed,
    uint8_t low_speed_confirmed,
    uint8_t high_sample_threshold,
    uint8_t low_confirm_threshold)
{
    if (high_speed_confirmed)
    {
        if (logic->high_sample_count < high_sample_threshold)
        {
            logic->high_sample_count++;
        }
        if (logic->high_sample_count >= high_sample_threshold)
        {
            logic->speed_established = 1;
        }
        logic->low_after_high_count = 0;
        return 0U;
    }

    if (!logic->speed_established)
    {
        logic->high_sample_count = 0;
    }

    if (logic->speed_established && low_speed_confirmed)
    {
        if (logic->low_after_high_count < low_confirm_threshold)
        {
            logic->low_after_high_count++;
        }
        if (logic->low_after_high_count >= low_confirm_threshold)
        {
            Navi_JumpTouchLogic_Reset(logic);
            return 1U;
        }
    }
    else
    {
        logic->low_after_high_count = 0;
    }

    return 0U;
}

static inline uint8_t Navi_JumpTouchLogic_Update(NaviJumpTouchLogic_t *logic,
                                                 float avg_speed_abs,
                                                 float high_speed_threshold,
                                                 float low_speed_threshold,
                                                 uint8_t high_sample_threshold,
                                                 uint8_t low_confirm_threshold)
{
    return Navi_JumpTouchLogic_UpdateSignals(
        logic,
        (avg_speed_abs >= high_speed_threshold) ? 1U : 0U,
        (avg_speed_abs <= low_speed_threshold) ? 1U : 0U,
        high_sample_threshold,
        low_confirm_threshold);
}

static inline uint8_t Navi_JumpTouchLogic_UpdateWheels(
    NaviJumpTouchLogic_t *logic,
    float left_speed_abs,
    float right_speed_abs,
    float high_speed_threshold,
    float low_speed_threshold,
    uint8_t high_sample_threshold,
    uint8_t low_confirm_threshold)
{
    return Navi_JumpTouchLogic_UpdateSignals(
        logic,
        (left_speed_abs >= high_speed_threshold &&
         right_speed_abs >= high_speed_threshold) ? 1U : 0U,
        (left_speed_abs <= low_speed_threshold &&
         right_speed_abs <= low_speed_threshold) ? 1U : 0U,
        high_sample_threshold,
        low_confirm_threshold);
}

#endif
