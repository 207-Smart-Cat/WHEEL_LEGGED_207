#ifndef BRIDGE_ROLL_PEAK_H
#define BRIDGE_ROLL_PEAK_H

#include <stdint.h>

typedef struct
{
    float baseline_deg;
    int8_t last_sign;
    uint8_t peak_count;
} BridgeRollPeakTracker_t;

void BridgeRollPeak_Reset(BridgeRollPeakTracker_t *tracker, float baseline_deg);
uint8_t BridgeRollPeak_Update(BridgeRollPeakTracker_t *tracker, float roll_deg);

#endif
