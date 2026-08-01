#ifndef NAVIGATION_SPEED_ZONE_H
#define NAVIGATION_SPEED_ZONE_H

#include "zf_common_typedef.h"
#include "navigation_tracking.h"

#define NAVI_SPEED_ZONE_INACTIVE (-1.0f)

typedef struct
{
    uint16 start_point_idx;
    uint16 end_point_idx;
    float speed;
} NaviSpeedZone_t;

typedef struct
{
    const NaviSpeedZone_t *zones;
    uint8 zone_count;
    uint8 return_high_speed_last_n_points;
} NaviMapSpeedZoneProfile_t;

void Navi_SpeedZone_Reset(void);
uint8 Navi_SpeedZone_Select_Profile(uint8 map_group,
                                    const Navi_WayPoint_t *map,
                                    uint16 count);
float Navi_SpeedZone_Get_Passage_Speed(uint16 start_point_idx,
                                       uint16 end_point_idx,
                                       float speed,
                                       uint16 current_target_idx);
float Navi_SpeedZone_Get_Speed(uint16 current_target_idx);
uint8 Navi_SpeedZone_Is_Active(void);
uint8 Navi_SpeedZone_Is_Final_Target_Hold(uint16 current_target_idx,
                                          uint16 total_points);

#endif
