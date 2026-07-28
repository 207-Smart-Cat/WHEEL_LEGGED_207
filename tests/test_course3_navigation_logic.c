#include <assert.h>

#include "course3_display_state.h"
#include "navigation_tracking.h"

int main(void)
{
    assert(Course3Waypoint_RequiresAlign(WP_TYPE_NORMAL) == 0U);
    assert(Course3Waypoint_RequiresAlign(WP_TYPE_BRIDGE) == 1U);
    assert(Course3Waypoint_RequiresAlign(WP_TYPE_JUMP) == 1U);
    assert(Course3Waypoint_RequiresAlign(WP_TYPE_MINE_SWEEP) == 0U);
    return 0;
}
