#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "course3_bridge_logic.h"
#include "navigation_tracking.h"

#define TEST_VEHICLE_MODE_COURSE_3 (3U)
#define TEST_VEHICLE_MODE_COURSE_3_INERTIAL (4U)
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
    assert(Course3Mode_IsCourse3(TEST_VEHICLE_MODE_COURSE_3) == 1U);
    assert(Course3Mode_IsCourse3(TEST_VEHICLE_MODE_COURSE_3_INERTIAL) == 1U);
    assert(Course3Mode_IsCourse3(TEST_VEHICLE_MODE_NORMAL) == 0U);
    assert(Course3Mode_UsesVision(TEST_VEHICLE_MODE_COURSE_3) == 1U);
    assert(Course3Mode_UsesVision(TEST_VEHICLE_MODE_COURSE_3_INERTIAL) == 0U);

    assert(Course3Segment_IsPairedType(WP_TYPE_BRIDGE) == 1U);
    assert(Course3Segment_IsPairedType(WP_TYPE_BUMP) == 1U);
    assert(Course3Segment_IsPairedType(WP_TYPE_STAIR_RAMP) == 1U);
    assert(Course3Segment_IsPairedType(WP_TYPE_JUMP) == 0U);
    assert(Course3Segment_RequiresVision(WP_TYPE_BRIDGE) == 1U);
    assert(Course3Segment_RequiresVision(WP_TYPE_STAIR_RAMP) == 1U);
    assert(Course3Segment_RequiresVision(WP_TYPE_BUMP) == 0U);
    assert(Course3Segment_PointCount(WP_TYPE_BRIDGE) == 3U);
    assert(Course3Segment_PointCount(WP_TYPE_STAIR_RAMP) == 3U);
    assert(Course3Segment_PointCount(WP_TYPE_BUMP) == 2U);
    assert(Course3Segment_ExpectedAction(WP_TYPE_BRIDGE, 0U) == NAVI_VISION_SEGMENT_ACTION_CALIBRATE);
    assert(Course3Segment_ExpectedAction(WP_TYPE_BRIDGE, 1U) == NAVI_VISION_SEGMENT_ACTION_ENTRY);
    assert(Course3Segment_ExpectedAction(WP_TYPE_BRIDGE, 2U) == NAVI_VISION_SEGMENT_ACTION_END);
    assert(Course3Segment_ExpectedAction(WP_TYPE_BUMP, 1U) == NAVI_BUMP_ACTION_END);
    assert(Course3Segment_PointCountForMode(TEST_VEHICLE_MODE_COURSE_3,
                                           WP_TYPE_BRIDGE) == 3U);
    assert(Course3Segment_PointCountForMode(TEST_VEHICLE_MODE_COURSE_3_INERTIAL,
                                           WP_TYPE_BRIDGE) == 2U);
    assert(Course3Segment_PointCountForMode(TEST_VEHICLE_MODE_COURSE_3_INERTIAL,
                                           WP_TYPE_BUMP) == 2U);
    assert(Course3Segment_PointCountForMode(TEST_VEHICLE_MODE_COURSE_3_INERTIAL,
                                           WP_TYPE_STAIR_RAMP) == 2U);
    assert(Course3Segment_ExpectedActionForMode(TEST_VEHICLE_MODE_COURSE_3_INERTIAL,
                                               WP_TYPE_BRIDGE,
                                               0U) == NAVI_SEGMENT_ACTION_START);
    assert(Course3Segment_ExpectedActionForMode(TEST_VEHICLE_MODE_COURSE_3_INERTIAL,
                                               WP_TYPE_BRIDGE,
                                               1U) == NAVI_SEGMENT_ACTION_END);
    assert(Course3Segment_RequiresVisionForMode(TEST_VEHICLE_MODE_COURSE_3,
                                               WP_TYPE_BRIDGE) == 1U);
    assert(Course3Segment_RequiresVisionForMode(TEST_VEHICLE_MODE_COURSE_3_INERTIAL,
                                               WP_TYPE_BRIDGE) == 0U);
    assert(Course3Segment_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3,
                                            WP_TYPE_BUMP,
                                            NAVI_SEGMENT_ACTION_START) == 1U);
    assert(Course3Segment_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3,
                                            WP_TYPE_STAIR_RAMP,
                                            NAVI_SEGMENT_ACTION_END) == 0U);
    assert(Course3Segment_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3_INERTIAL,
                                            WP_TYPE_BRIDGE,
                                            NAVI_SEGMENT_ACTION_START) == 1U);
    assert(Course3Segment_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3_INERTIAL,
                                            WP_TYPE_BUMP,
                                            NAVI_SEGMENT_ACTION_START) == 1U);
    assert(Course3Segment_ShouldQueueAction(TEST_VEHICLE_MODE_COURSE_3_INERTIAL,
                                            WP_TYPE_STAIR_RAMP,
                                            NAVI_SEGMENT_ACTION_START) == 1U);
    assert(Course3Segment_ShouldApproach(TEST_VEHICLE_MODE_COURSE_3,
                                        WP_TYPE_BRIDGE,
                                        NAVI_SEGMENT_ACTION_START,
                                        0.50f) == 1U);
    assert(Course3Segment_ShouldApproach(TEST_VEHICLE_MODE_COURSE_3,
                                        WP_TYPE_BUMP,
                                        NAVI_SEGMENT_ACTION_START,
                                        0.501f) == 0U);
    assert(Course3Segment_ShouldApproach(TEST_VEHICLE_MODE_COURSE_3,
                                        WP_TYPE_STAIR_RAMP,
                                        NAVI_VISION_SEGMENT_ACTION_END,
                                        0.10f) == 0U);

    assert(Course3Remote_SelectSpecialType(192) == WP_TYPE_BRIDGE);
    assert(Course3Remote_SelectSpecialType(591) == WP_TYPE_BRIDGE);
    assert(Course3Remote_SelectSpecialType(592) == WP_TYPE_BUMP);
    assert(Course3Remote_SelectSpecialType(992) == WP_TYPE_BUMP);
    assert(Course3Remote_SelectSpecialType(1391) == WP_TYPE_BUMP);
    assert(Course3Remote_SelectSpecialType(1392) == WP_TYPE_STAIR_RAMP);
    assert(Course3Remote_SelectSpecialType(1792) == WP_TYPE_STAIR_RAMP);

    assert(fabsf(Course3AngleSlew_Step(0.0f, 90.0f, 0.9f) - 0.9f) < 0.001f);
    assert(fabsf(Course3AngleSlew_Step(10.0f, -30.0f, 1.0f) - 9.0f) < 0.001f);
    assert(fabsf(Course3AngleSlew_Step(179.0f, -179.0f, 1.0f) - 180.0f) < 0.001f);
    assert(fabsf(Course3AngleSlew_Step(-179.0f, 179.0f, 1.0f) + 180.0f) < 0.001f);
    assert(fabsf(Course3AngleSlew_Step(12.0f, 14.0f, 5.0f) - 14.0f) < 0.001f);

    puts("course3_bridge_logic tests passed");
    return 0;
}
