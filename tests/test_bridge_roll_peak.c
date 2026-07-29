#include <assert.h>

#include "bridge_roll_peak.h"

int main(void)
{
    BridgeRollPeakTracker_t tracker;

    BridgeRollPeak_Reset(&tracker, 4.0f);
    assert(!BridgeRollPeak_Update(&tracker, 13.9f));
    assert(!BridgeRollPeak_Update(&tracker, 14.0f));
    assert(tracker.peak_count == 1U);
    assert(!BridgeRollPeak_Update(&tracker, 16.0f));
    assert(tracker.peak_count == 1U);
    assert(!BridgeRollPeak_Update(&tracker, -6.0f));
    assert(tracker.peak_count == 2U);
    assert(BridgeRollPeak_Update(&tracker, 14.0f));
    assert(tracker.peak_count == 3U);

    BridgeRollPeak_Reset(&tracker, -3.0f);
    assert(!BridgeRollPeak_Update(&tracker, -13.0f));
    assert(!BridgeRollPeak_Update(&tracker, 7.0f));
    assert(BridgeRollPeak_Update(&tracker, -13.0f));

    return 0;
}
