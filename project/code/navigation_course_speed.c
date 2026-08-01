#include "navigation_course_speed.h"
#include "runtime_status.h"

static const NaviCourseSpeedProfile_t navi_course_speed_profiles[NAVI_COURSE_SLOT_COUNT] =
{
    {
        550.0f,
        700.0f,
        950.0f,
        3.0f,
        550.0f,
        50.0f,
        100.0f,
        700.0f,
        80.0f,
        0U,
        0U
    },
    {
        550.0f,
        700.0f,
        950.0f,
        3.0f,
        550.0f,
        50.0f,
        100.0f,
        700.0f,
        80.0f,
        1U,
        1U
    },
    {
        300.0f,
        300.0f,
        500.0f,
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
