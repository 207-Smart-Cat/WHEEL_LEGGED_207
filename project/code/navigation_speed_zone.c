#include "navigation_speed_zone.h"
#include "navigation_course_speed.h"
#include "ipc_shared_data.h"
#include "runtime_status.h"
#include <math.h>

#define NAVI_SPEED_ZONE_INDEX_RETURN     (0xFEU)
#define NAVI_SPEED_ZONE_INDEX_NONE       (0xFFU)
#define NAVI_SPEED_ZONE_MAX_TURN_DEG     (12.0f)
#define NAVI_SPEED_ZONE_MIN_LENGTH_M     (5.0f)
#define NAVI_SPEED_ZONE_RAD_TO_DEG       (57.2957795f)

/*
 * One UI map owns one profile, and one profile may contain multiple zones:
 * static const NaviSpeedZone_t map01_speed_zones[] =
 * {
 *     {2U, 4U, 900.0f},
 *     {7U, 9U, 850.0f},
 * };
 * static const NaviSpeedZone_t map02_speed_zones[] =
 * {
 *     {1U, 3U, 800.0f},
 *     {5U, 8U, 920.0f},
 * };
 * Then replace the corresponding entries with, for example:
 * {map01_speed_zones, 2U, 3U},
 * {map02_speed_zones, 2U, 3U}.
 * The third member enables the original last-three-point return sprint.
 */

static const NaviSpeedZone_t map04_speed_zones[] =
{
    {1U, 2U, 1500.0f},
};
static const NaviSpeedZone_t map05_speed_zones[] =
{
    {4U, 5U, 1000.0f},
    {5U, 8U, 600.0f}
};

static const NaviMapSpeedZoneProfile_t
navi_speed_zone_profiles[NAV_EXEC_GROUP_COUNT] =
{
    {NULL, 0U, 3U}, /* UI map 1, Flash group 0 */
    {NULL, 0U, 3U}, /* UI map 2, Flash group 1 */
    {NULL, 0U, 0U}, /* UI map 3, Flash group 2 */
    {map04_speed_zones, 1U, 3U}, /* UI map 4, Flash group 3 */
    {map05_speed_zones, 2U, 3U}, /* UI map 5, Flash group 4 */
    {NULL, 0U, 3U}, /* UI map 6, Flash group 5 */
    {NULL, 0U, 3U}, /* UI map 7, Flash group 6 */
    {NULL, 0U, 0U}, /* UI map 8, Flash group 7 */
    {NULL, 0U, 0U}, /* UI map 9, Flash group 8 */
    {NULL, 0U, 0U}, /* UI map 10, Flash group 9 */
    {NULL, 0U, 0U}  /* UI map 11, Course 3 preset group 10 */
};

typedef char navi_speed_zone_map_count_must_match[
    (sizeof(navi_speed_zone_profiles) / sizeof(navi_speed_zone_profiles[0]) ==
     NAV_EXEC_GROUP_COUNT) ? 1 : -1];

static const NaviMapSpeedZoneProfile_t *navi_speed_zone_selected_profile = NULL;
static const Navi_WayPoint_t *navi_speed_zone_selected_map = NULL;
static uint16 navi_speed_zone_selected_count = 0U;
static uint8 navi_speed_zone_selected_group = NAVI_SPEED_ZONE_INDEX_NONE;
static uint8 navi_speed_zone_active_index = NAVI_SPEED_ZONE_INDEX_NONE;

static uint8 navi_speed_zone_type_is_allowed(WayPoint_Type type,
                                              uint16 point_idx,
                                              uint16 zone_end,
                                              uint16 map_count)
{
    if (type == WP_TYPE_NORMAL ||
        type == WP_TYPE_HIGH_SPEED ||
        type == WP_TYPE_HOME)
    {
        return 1U;
    }

    return (type == WP_TYPE_STOP &&
            point_idx == zone_end &&
            point_idx == (uint16)(map_count - 1U)) ? 1U : 0U;
}

static uint8 navi_speed_zone_validate_one(const NaviSpeedZone_t *zone,
                                          uint8 zone_index,
                                          const Navi_WayPoint_t *map,
                                          uint16 count,
                                          float high_speed_limit)
{
    uint16 idx;
    float total_length = 0.0f;
    float previous_heading = 0.0f;
    uint8 heading_valid = 0U;

    if (zone->start_point_idx >= zone->end_point_idx ||
        zone->end_point_idx >= count ||
        zone->speed <= 0.0f)
    {
        IPC_LOG_Printf("[NAVI_ZONE] map %d zone %d invalid range/speed.\r\n",
                       (int)(navi_speed_zone_selected_group + 1U),
                       (int)zone_index);
        return 0U;
    }

    if (zone->speed > high_speed_limit)
    {
        IPC_LOG_Printf("[NAVI_ZONE] map %d zone %d speed will be limited.\r\n",
                       (int)(navi_speed_zone_selected_group + 1U),
                       (int)zone_index);
    }

    for (idx = (uint16)(zone->start_point_idx + 1U);
         idx <= zone->end_point_idx;
         idx++)
    {
        float dx;
        float dy;
        float length;
        float heading;

        if (!map[idx - 1U].valid || !map[idx].valid ||
            !navi_speed_zone_type_is_allowed(map[idx].type,
                                             idx,
                                             zone->end_point_idx,
                                             count))
        {
            IPC_LOG_Printf("[NAVI_ZONE] map %d zone %d covers invalid/action point %d.\r\n",
                           (int)(navi_speed_zone_selected_group + 1U),
                           (int)zone_index,
                           (int)idx);
            return 0U;
        }

        dx = map[idx].x - map[idx - 1U].x;
        dy = map[idx].y - map[idx - 1U].y;
        length = (float)sqrt((double)(dx * dx + dy * dy));
        total_length += length;

        if (length > 0.001f)
        {
            float heading_delta;
            heading = (float)(atan2((double)dy, (double)dx) *
                              NAVI_SPEED_ZONE_RAD_TO_DEG);
            if (heading_valid)
            {
                heading_delta = heading - previous_heading;
                while (heading_delta > 180.0f) heading_delta -= 360.0f;
                while (heading_delta < -180.0f) heading_delta += 360.0f;
                if (fabs((double)heading_delta) > NAVI_SPEED_ZONE_MAX_TURN_DEG)
                {
                    IPC_LOG_Printf("[NAVI_ZONE] map %d zone %d turn warning.\r\n",
                                   (int)(navi_speed_zone_selected_group + 1U),
                                   (int)zone_index);
                }
            }
            previous_heading = heading;
            heading_valid = 1U;
        }
    }

    if (total_length < NAVI_SPEED_ZONE_MIN_LENGTH_M)
    {
        IPC_LOG_Printf("[NAVI_ZONE] map %d zone %d length warning.\r\n",
                       (int)(navi_speed_zone_selected_group + 1U),
                       (int)zone_index);
    }

    return 1U;
}

static uint8 navi_speed_zone_profiles_overlap(const NaviSpeedZone_t *left,
                                               const NaviSpeedZone_t *right)
{
    return (left->start_point_idx < right->end_point_idx &&
            right->start_point_idx < left->end_point_idx) ? 1U : 0U;
}

static uint8 navi_speed_zone_get_return_range(uint16 *start_point_idx,
                                               uint16 *end_point_idx)
{
    uint8 last_n;

    if (navi_speed_zone_selected_profile == NULL ||
        start_point_idx == NULL || end_point_idx == NULL)
    {
        return 0U;
    }

    last_n = navi_speed_zone_selected_profile->return_high_speed_last_n_points;
    if (last_n < 2U || navi_speed_zone_selected_count < (uint16)last_n)
    {
        return 0U;
    }

    *start_point_idx = (uint16)(navi_speed_zone_selected_count - (uint16)last_n);
    *end_point_idx = (uint16)(navi_speed_zone_selected_count - 1U);
    return 1U;
}

void Navi_SpeedZone_Reset(void)
{
    navi_speed_zone_selected_profile = NULL;
    navi_speed_zone_selected_map = NULL;
    navi_speed_zone_selected_count = 0U;
    navi_speed_zone_selected_group = NAVI_SPEED_ZONE_INDEX_NONE;
    navi_speed_zone_active_index = NAVI_SPEED_ZONE_INDEX_NONE;
}

uint8 Navi_SpeedZone_Select_Profile(uint8 map_group,
                                    const Navi_WayPoint_t *map,
                                    uint16 count)
{
    const NaviMapSpeedZoneProfile_t *profile;
    const NaviCourseSpeedProfile_t *course_profile;
    uint8 left;
    uint8 right;

    Navi_SpeedZone_Reset();
    if (map_group >= NAV_EXEC_GROUP_COUNT || map == NULL || count < 2U)
    {
        return 0U;
    }

    navi_speed_zone_selected_group = map_group;
    profile = &navi_speed_zone_profiles[map_group];
    if (profile->return_high_speed_last_n_points == 1U)
    {
        Navi_SpeedZone_Reset();
        return 0U;
    }
    if (profile->zone_count == 0U &&
        profile->return_high_speed_last_n_points == 0U)
    {
        navi_speed_zone_selected_profile = profile;
        navi_speed_zone_selected_map = map;
        navi_speed_zone_selected_count = count;
        IPC_LOG_Printf("[NAVI_ZONE] map %d has no speed zones.\r\n",
                       (int)(map_group + 1U));
        return 1U;
    }
    if (profile->zone_count > 0U && profile->zones == NULL)
    {
        Navi_SpeedZone_Reset();
        return 0U;
    }

    course_profile = Navi_CourseSpeed_Get_Profile(Runtime_Get_Vehicle_Mode());
    for (left = 0U; left < profile->zone_count; left++)
    {
        if (!navi_speed_zone_validate_one(&profile->zones[left],
                                          left,
                                          map,
                                          count,
                                          course_profile->high_speed_max_velocity))
        {
            Navi_SpeedZone_Reset();
            return 0U;
        }
        for (right = (uint8)(left + 1U); right < profile->zone_count; right++)
        {
            if (navi_speed_zone_profiles_overlap(&profile->zones[left],
                                                 &profile->zones[right]))
            {
                IPC_LOG_Printf("[NAVI_ZONE] map %d zones %d/%d overlap.\r\n",
                               (int)(map_group + 1U),
                               (int)left,
                               (int)right);
                Navi_SpeedZone_Reset();
                return 0U;
            }
        }
    }

    navi_speed_zone_selected_profile = profile;
    navi_speed_zone_selected_map = map;
    navi_speed_zone_selected_count = count;
    IPC_LOG_Printf("[NAVI_ZONE] map %d selected, zones=%d, return-last-n=%d.\r\n",
                   (int)(map_group + 1U),
                   (int)profile->zone_count,
                   (int)profile->return_high_speed_last_n_points);
    return 1U;
}

float Navi_SpeedZone_Get_Passage_Speed(uint16 start_point_idx,
                                       uint16 end_point_idx,
                                       float speed,
                                       uint16 current_target_idx)
{
    if (start_point_idx >= end_point_idx || speed < 0.0f)
    {
        return NAVI_SPEED_ZONE_INACTIVE;
    }

    return (current_target_idx > start_point_idx &&
            current_target_idx <= end_point_idx) ?
           speed : NAVI_SPEED_ZONE_INACTIVE;
}

float Navi_SpeedZone_Get_Speed(uint16 current_target_idx)
{
    uint8 idx;
    uint16 return_start_idx;
    uint16 return_end_idx;

    navi_speed_zone_active_index = NAVI_SPEED_ZONE_INDEX_NONE;
    if (navi_speed_zone_selected_profile == NULL ||
        current_target_idx >= navi_speed_zone_selected_count)
    {
        return NAVI_SPEED_ZONE_INACTIVE;
    }

    for (idx = 0U; idx < navi_speed_zone_selected_profile->zone_count; idx++)
    {
        const NaviSpeedZone_t *zone = &navi_speed_zone_selected_profile->zones[idx];
        float speed = Navi_SpeedZone_Get_Passage_Speed(zone->start_point_idx,
                                                        zone->end_point_idx,
                                                        zone->speed,
                                                        current_target_idx);
        if (speed >= 0.0f)
        {
            navi_speed_zone_active_index = idx;
            return speed;
        }
    }

    if (navi_speed_zone_get_return_range(&return_start_idx, &return_end_idx) &&
        navi_speed_zone_type_is_allowed(
            navi_speed_zone_selected_map[current_target_idx].type,
            current_target_idx,
            return_end_idx,
            navi_speed_zone_selected_count))
    {
        float speed = Navi_SpeedZone_Get_Passage_Speed(
            return_start_idx,
            return_end_idx,
            Navi_CourseSpeed_Get_Profile(
                Runtime_Get_Vehicle_Mode())->high_speed_max_velocity,
            current_target_idx);
        if (speed >= 0.0f)
        {
            navi_speed_zone_active_index = NAVI_SPEED_ZONE_INDEX_RETURN;
            return speed;
        }
    }

    return NAVI_SPEED_ZONE_INACTIVE;
}

uint8 Navi_SpeedZone_Is_Active(void)
{
    return (navi_speed_zone_active_index != NAVI_SPEED_ZONE_INDEX_NONE) ? 1U : 0U;
}

uint8 Navi_SpeedZone_Is_Final_Target_Hold(uint16 current_target_idx,
                                          uint16 total_points)
{
    const NaviSpeedZone_t *zone;
    uint16 return_start_idx;
    uint16 return_end_idx;

    if (!Navi_SpeedZone_Is_Active() ||
        navi_speed_zone_selected_profile == NULL ||
        navi_speed_zone_selected_map == NULL ||
        current_target_idx >= total_points ||
        current_target_idx >= navi_speed_zone_selected_count)
    {
        return 0U;
    }

    if (navi_speed_zone_active_index == NAVI_SPEED_ZONE_INDEX_RETURN)
    {
        if (!navi_speed_zone_get_return_range(&return_start_idx,
                                               &return_end_idx))
        {
            return 0U;
        }
        return (current_target_idx > return_start_idx &&
                current_target_idx <= return_end_idx &&
                (current_target_idx >= (uint16)(total_points - 1U) ||
                 navi_speed_zone_selected_map[current_target_idx].type == WP_TYPE_STOP)) ? 1U : 0U;
    }

    if (navi_speed_zone_active_index >= navi_speed_zone_selected_profile->zone_count)
    {
        return 0U;
    }

    zone = &navi_speed_zone_selected_profile->zones[navi_speed_zone_active_index];
    return (current_target_idx == zone->end_point_idx &&
            (current_target_idx >= (uint16)(total_points - 1U) ||
             navi_speed_zone_selected_map[current_target_idx].type == WP_TYPE_STOP)) ? 1U : 0U;
}
