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

static inline uint8_t Navi_JumpTouchLogic_Update(NaviJumpTouchLogic_t *logic,
                                                 float avg_speed_abs,
                                                 float high_speed_threshold,
                                                 float low_speed_threshold,
                                                 uint8_t high_sample_threshold,
                                                 uint8_t low_confirm_threshold)
{
    if (avg_speed_abs >= high_speed_threshold)
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

    if (logic->speed_established && avg_speed_abs <= low_speed_threshold)
    {
        logic->low_after_high_count++;
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

#endif
