#include <stdio.h>

#include "navigation_smooth_logic.h"

static int expect_non_terminal_point_advances_inside_radius(void)
{
    return Navi_Smooth_Should_Advance(0.15f, 0.20f, 0U) == 1U;
}

static int expect_terminal_point_never_advances_early(void)
{
    return Navi_Smooth_Should_Advance(0.15f, 0.20f, 1U) == 0U;
}

static int expect_speed_limit_only_applies_in_smooth_zone(void)
{
    return Navi_Smooth_Resolve_Target_Velocity(260.0f, 240.0f, 180.0f, 1U) == 180.0f &&
           Navi_Smooth_Resolve_Target_Velocity(260.0f, 240.0f, 180.0f, 0U) == 240.0f;
}

int main(void)
{
    if (!expect_non_terminal_point_advances_inside_radius())
    {
        printf("non-terminal point did not advance inside smooth radius\n");
        return 1;
    }

    if (!expect_terminal_point_never_advances_early())
    {
        printf("terminal point advanced early\n");
        return 1;
    }

    if (!expect_speed_limit_only_applies_in_smooth_zone())
    {
        printf("smooth speed limit behavior mismatch\n");
        return 1;
    }

    return 0;
}
