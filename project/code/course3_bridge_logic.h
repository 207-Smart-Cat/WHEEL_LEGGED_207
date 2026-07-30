#ifndef COURSE3_BRIDGE_LOGIC_H
#define COURSE3_BRIDGE_LOGIC_H

#include "zf_common_typedef.h"

uint8 Course3Bridge_ShouldQueueAction(uint8 vehicle_mode,
                                      uint8 waypoint_type,
                                      uint16 action_cmd);
uint8 Course3Segment_IsPairedType(uint8 waypoint_type);
uint8 Course3Segment_RequiresVision(uint8 waypoint_type);
uint8 Course3Segment_PointCount(uint8 waypoint_type);
uint16 Course3Segment_ExpectedAction(uint8 waypoint_type, uint8 ordinal);
uint8 Course3Segment_IsStartAction(uint8 waypoint_type, uint16 action_cmd);
uint8 Course3Segment_ShouldQueueAction(uint8 vehicle_mode,
                                       uint8 waypoint_type,
                                       uint16 action_cmd);
uint8 Course3Segment_ShouldApproach(uint8 vehicle_mode,
                                    uint8 waypoint_type,
                                    uint16 action_cmd,
                                    float distance_m);
uint8 Course3Remote_SelectSpecialType(int32 ch4_value);
float Course3AngleSlew_Step(float current_deg,
                            float desired_deg,
                            float max_step_deg);

#endif
