#include <assert.h>
#include <stdio.h>

#include "course3_bridge_logic.h"
#include "navigation_tracking.h"

#define TEST_VEHICLE_MODE_COURSE_3 (3U)
#define TEST_VEHICLE_MODE_NORMAL   (0U)

int main(void)
{
    assert(Course3Bridge_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3,
                                           WP_TYPE_BRIDGE,
                                           NAVI_BRIDGE_ACTION_START) == 1U);
    assert(Course3Bridge_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3,
                                           WP_TYPE_BRIDGE,
                                           NAVI_BRIDGE_ACTION_END) == 0U);
    assert(Course3Bridge_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3,
                                           WP_TYPE_JUMP,
                                           NAVI_BRIDGE_ACTION_START) == 0U);
    assert(Course3Bridge_ShouldQueueAction(TEST_VEHICLE_MODE_NORMAL,
                                           WP_TYPE_BRIDGE,
                                           NAVI_BRIDGE_ACTION_START) == 0U);

    puts("course3_bridge_logic tests passed");
    return 0;
}
