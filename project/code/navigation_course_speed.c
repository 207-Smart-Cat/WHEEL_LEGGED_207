#include "navigation_course_speed.h"
#include "runtime_status.h"
#include "navigation_tracking.h"

static const NaviCourseSpeedProfile_t navi_course_speed_profiles[NAVI_COURSE_SLOT_COUNT] =
{
    {
        900.0f,
        900.0f,
        1500.0f, /* high_speed_max_velocity */
        3.0f,
        800.0f,
        100.0f,
        100.0f,
        900.0f,
        80.0f,
        0U,
        0U
    },
    {
        550.0f,
        900.0f,
        2000.0f, /* high_speed_max_velocity */
        3.0f,
        600.0f,
        120.0f,
        100.0f,
        1000.0f,
        200.0f,
        0U,
        0U
    },
    {
        300.0f,
        300.0f,
        1000.0f, /* high_speed_max_velocity */
        3.0f,
        300.0f,
        0.0f,
        100.0f,
        300.0f,
        12.0f,
        0U,
        0U
    }
};

NaviCourseSlot_t Navi_CourseSpeed_Get_Slot(uint8 vehicle_mode)
{
    switch (vehicle_mode)
    {
        case VEHICLE_MODE_COURSE_1:
            return NAVI_COURSE_SLOT_1;
        case VEHICLE_MODE_COURSE_2:
            return NAVI_COURSE_SLOT_2;
        case VEHICLE_MODE_COURSE_3:
        case VEHICLE_MODE_COURSE_3_INERTIAL:
        default:
            return NAVI_COURSE_SLOT_3;
    }
}

const NaviCourseSpeedProfile_t *Navi_CourseSpeed_Get_Profile(uint8 vehicle_mode)
{
    return &navi_course_speed_profiles[Navi_CourseSpeed_Get_Slot(vehicle_mode)];
}

uint8 Navi_CourseSpeed_Get_Approach(uint8 vehicle_mode,
                                    uint8 waypoint_type,
                                    NaviWaypointApproachConfig_t *out)
{
    if (out == NULL)
    {
        return 0U;
    }

    out->trigger_distance_m = 0.0f;
    out->approach_speed = 0.0f;

    if (vehicle_mode == VEHICLE_MODE_COURSE_2 &&
        waypoint_type == WP_TYPE_MINE_SWEEP)
    {
        out->trigger_distance_m = 1.0f;
        out->approach_speed = 120.0f;
        return 1U;
    }

    if ((vehicle_mode == VEHICLE_MODE_COURSE_3 ||
         vehicle_mode == VEHICLE_MODE_COURSE_3_INERTIAL) &&
        (waypoint_type == WP_TYPE_BRIDGE ||
         waypoint_type == WP_TYPE_BUMP ||
         waypoint_type == WP_TYPE_STAIR_RAMP))
    {
        out->trigger_distance_m = 0.50f;
        out->approach_speed = 100.0f;
        return 1U;
    }

    return 0U;
}
