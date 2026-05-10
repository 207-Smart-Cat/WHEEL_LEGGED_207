#ifndef VEHICLE_SUPERVISOR_H
#define VEHICLE_SUPERVISOR_H

#include "zf_common_headfile.h"

typedef enum
{
    VEHICLE_EVENT_SOURCE_NONE = 0,
    VEHICLE_EVENT_SOURCE_WIFI,
    VEHICLE_EVENT_SOURCE_REMOTE,
    VEHICLE_EVENT_SOURCE_SCREEN,
    VEHICLE_EVENT_SOURCE_SOFTWARE
} vehicle_event_source_t;

void Vehicle_Emergency_Stop(vehicle_event_source_t source);
void Vehicle_Emergency_Recover(vehicle_event_source_t source);
uint8 Vehicle_Is_Emergency_Stop(void);
vehicle_event_source_t Vehicle_Get_Emergency_Source(void);

#endif
