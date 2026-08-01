#ifndef NAVIGATION_COURSE_SPEED_H
#define NAVIGATION_COURSE_SPEED_H

#include "zf_common_typedef.h"

typedef enum
{
    NAVI_COURSE_SLOT_1 = 0,
    NAVI_COURSE_SLOT_2,
    NAVI_COURSE_SLOT_3,
    NAVI_COURSE_SLOT_COUNT
} NaviCourseSlot_t;

typedef struct
{
    float fixed_tracking_velocity;
    float normal_max_velocity;
    float high_speed_max_velocity;
    float high_speed_exit_distance_m;
    float smooth_zone_speed_limit;
    float min_running_speed;
    float turn_min_speed;
    float turn_entry_max_velocity;
    float speed_max_step;
    uint8 allow_vofa_max_override;
    uint8 allow_vofa_step_override;
} NaviCourseSpeedProfile_t;

typedef struct
{
    float trigger_distance_m;
    float approach_speed;
} NaviWaypointApproachConfig_t;

NaviCourseSlot_t Navi_CourseSpeed_Get_Slot(uint8 vehicle_mode);
const NaviCourseSpeedProfile_t *Navi_CourseSpeed_Get_Profile(uint8 vehicle_mode);
uint8 Navi_CourseSpeed_Get_Approach(uint8 vehicle_mode,
                                    uint8 waypoint_type,
                                    NaviWaypointApproachConfig_t *out);

#endif
