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

    assert(Course3Segment_IsPairedType(WP_TYPE_BRIDGE) == 1U);
    assert(Course3Segment_IsPairedType(WP_TYPE_BUMP) == 1U);
    assert(Course3Segment_IsPairedType(WP_TYPE_STAIR_RAMP) == 1U);
    assert(Course3Segment_IsPairedType(WP_TYPE_JUMP) == 0U);
    assert(Course3Segment_RequiresVision(WP_TYPE_BRIDGE) == 1U);
    assert(Course3Segment_RequiresVision(WP_TYPE_STAIR_RAMP) == 1U);
    assert(Course3Segment_RequiresVision(WP_TYPE_BUMP) == 0U);
    assert(Course3Segment_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3,
                                            WP_TYPE_BUMP,
                                            NAVI_SEGMENT_ACTION_START) == 1U);
    assert(Course3Segment_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3,
                                            WP_TYPE_STAIR_RAMP,
                                            NAVI_SEGMENT_ACTION_END) == 0U);

    assert(Course3Remote_SelectSpecialType(192) == WP_TYPE_BRIDGE);
    assert(Course3Remote_SelectSpecialType(591) == WP_TYPE_BRIDGE);
    assert(Course3Remote_SelectSpecialType(592) == WP_TYPE_BUMP);
    assert(Course3Remote_SelectSpecialType(992) == WP_TYPE_BUMP);
    assert(Course3Remote_SelectSpecialType(1391) == WP_TYPE_BUMP);
    assert(Course3Remote_SelectSpecialType(1392) == WP_TYPE_STAIR_RAMP);
    assert(Course3Remote_SelectSpecialType(1792) == WP_TYPE_STAIR_RAMP);

    puts("course3_bridge_logic tests passed");
    return 0;
}
