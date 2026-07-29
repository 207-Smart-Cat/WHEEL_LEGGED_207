#include <stdint.h>
#include <stdio.h>

#include "navigation_touch_logic.h"

#define TEST_TOUCH_HIGH_SPEED       (50.0f)
#define TEST_TOUCH_LOW_SPEED        (10.0f)
#define TEST_TOUCH_HIGH_SAMPLES     (2U)
#define TEST_TOUCH_LOW_CONFIRM      (2U)

static NaviJumpTouchLogic_t touch_logic;

static uint8_t update_touch(float avg_speed_abs)
{
    return Navi_JumpTouchLogic_Update(&touch_logic,
                                      avg_speed_abs,
                                      TEST_TOUCH_HIGH_SPEED,
                                      TEST_TOUCH_LOW_SPEED,
                                      TEST_TOUCH_HIGH_SAMPLES,
                                      TEST_TOUCH_LOW_CONFIRM);
}

static uint8_t update_touch_wheels(float left_speed_abs,
                                   float right_speed_abs)
{
    return Navi_JumpTouchLogic_UpdateWheels(&touch_logic,
                                            left_speed_abs,
                                            right_speed_abs,
                                            TEST_TOUCH_HIGH_SPEED,
                                            TEST_TOUCH_LOW_SPEED,
                                            TEST_TOUCH_HIGH_SAMPLES,
                                            TEST_TOUCH_LOW_CONFIRM);
}

static int expect_no_touch_on_cold_low_speed(void)
{
    uint8_t touched = 0;
    uint8_t i;

    Navi_JumpTouchLogic_Reset(&touch_logic);
    for (i = 0; i < 10U; i++)
    {
        touched |= update_touch(0.0f);
    }

    return touched == 0U;
}

static int expect_touch_after_established_speed_drop(void)
{
    uint8_t touched = 0;

    Navi_JumpTouchLogic_Reset(&touch_logic);
    touched |= update_touch(80.0f);
    touched |= update_touch(75.0f);
    touched |= update_touch(0.0f);
    touched |= update_touch(0.0f);

    return touched == 1U;
}

static int expect_no_touch_when_only_one_wheel_stops(void)
{
    uint8_t touched = 0U;

    Navi_JumpTouchLogic_Reset(&touch_logic);
    touched |= update_touch_wheels(80.0f, 75.0f);
    touched |= update_touch_wheels(80.0f, 75.0f);
    touched |= update_touch_wheels(0.0f, 60.0f);
    touched |= update_touch_wheels(0.0f, 60.0f);

    return touched == 0U;
}

static int expect_touch_after_both_wheels_stop(void)
{
    uint8_t touched = 0U;

    Navi_JumpTouchLogic_Reset(&touch_logic);
    touched |= update_touch_wheels(80.0f, 75.0f);
    touched |= update_touch_wheels(80.0f, 75.0f);
    touched |= update_touch_wheels(5.0f, 4.0f);
    touched |= update_touch_wheels(3.0f, 2.0f);

    return touched == 1U;
}

int main(void)
{
    if (!expect_no_touch_on_cold_low_speed())
    {
        printf("cold low speed incorrectly touched\n");
        return 1;
    }
    if (!expect_touch_after_established_speed_drop())
    {
        printf("established speed drop did not touch\n");
        return 1;
    }
    if (!expect_no_touch_when_only_one_wheel_stops())
    {
        printf("one-wheel slowdown incorrectly touched\n");
        return 1;
    }
    if (!expect_touch_after_both_wheels_stop())
    {
        printf("both-wheel stop did not touch\n");
        return 1;
    }

    return 0;
}
