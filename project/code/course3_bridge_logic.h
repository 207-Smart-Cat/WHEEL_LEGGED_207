#ifndef COURSE3_BRIDGE_LOGIC_H
#define COURSE3_BRIDGE_LOGIC_H

#include "zf_common_typedef.h"

uint8 Course3Bridge_ShouldQueueAction(uint8 vehicle_mode,
                                      uint8 waypoint_type,
                                      uint16 action_cmd);
uint8 Course3Segment_IsPairedType(uint8 waypoint_type);
uint8 Course3Segment_RequiresVision(uint8 waypoint_type);
uint8 Course3Segment_ShouldQueueAction(uint8 vehicle_mode,
                                       uint8 waypoint_type,
                                       uint16 action_cmd);
uint8 Course3Remote_SelectSpecialType(int32 ch4_value);

#endif
