#include "bridge_roll_peak.h"

#define BRIDGE_ROLL_PEAK_THRESHOLD_DEG (10.0f)

void BridgeRollPeak_Reset(BridgeRollPeakTracker_t *tracker, float baseline_deg)
{
    tracker->baseline_deg = baseline_deg;
    tracker->last_sign = 0;
    tracker->peak_count = 0U;
}

uint8_t BridgeRollPeak_Update(BridgeRollPeakTracker_t *tracker, float roll_deg)
{
    float delta = roll_deg - tracker->baseline_deg;
    int8_t sign = 0;

    if (delta >= BRIDGE_ROLL_PEAK_THRESHOLD_DEG)
    {
        sign = 1;
    }
    else if (delta <= -BRIDGE_ROLL_PEAK_THRESHOLD_DEG)
    {
        sign = -1;
    }
    else
    {
        return 0U;
    }

    if (tracker->last_sign == sign)
    {
        return 0U;
    }

    tracker->last_sign = sign;
    if (tracker->peak_count < 3U)
    {
        tracker->peak_count++;
    }
    return (tracker->peak_count >= 3U) ? 1U : 0U;
}
