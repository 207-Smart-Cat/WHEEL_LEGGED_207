#include <assert.h>
#include <math.h>
#include <string.h>

#include "course3_display_state.h"
#include "navigation_tracking.h"

int main(void)
{
    assert(strcmp(Course3DisplayState_Text(COURSE3_DISPLAY_TRACK_ALIGN), "TRACK ALIGN") == 0);
    assert(strcmp(Course3DisplayState_Text(COURSE3_DISPLAY_ACTION), "ACTION") == 0);
    assert(strcmp(Course3DisplayState_Text(COURSE3_DISPLAY_DONE), "DONE") == 0);
    assert(Course3DisplayState_Text(COURSE3_DISPLAY_IDLE) == 0);

    assert(strcmp(Course3TargetType_Text(WP_TYPE_NORMAL), "NORMAL") == 0);
    assert(strcmp(Course3TargetType_Text(WP_TYPE_BRIDGE), "BRIDGE") == 0);
    assert(strcmp(Course3TargetType_Text(WP_TYPE_JUMP), "JUMP") == 0);
    assert(strcmp(Course3TargetType_Text(WP_TYPE_BUMP), "BUMP") == 0);
    assert(strcmp(Course3TargetType_Text(WP_TYPE_STAIR_RAMP), "RAMP") == 0);
    assert(Course3TargetType_Text(WP_TYPE_MINE_SWEEP) == 0);

    assert(Course3Vision_ShouldEnter(3U, COURSE3_DISPLAY_TRACK_ALIGN, 0U) == 1U);
    assert(Course3Vision_ShouldEnter(3U, COURSE3_DISPLAY_ACTION, 0U) == 0U);
    assert(Course3Vision_ShouldEnter(2U, COURSE3_DISPLAY_TRACK_ALIGN, 0U) == 0U);
    assert(Course3Vision_ShouldEnter(3U, COURSE3_DISPLAY_IDLE, 0U) == 0U);
    assert(Course3Vision_ShouldRestore(COURSE3_DISPLAY_IDLE, 1U) == 1U);
    assert(Course3Vision_ShouldRestore(COURSE3_DISPLAY_ACTION, 1U) == 1U);

    assert(fabsf(Course3Search_TargetOffsetDeg(0U)) < 0.001f);
    assert(fabsf(Course3Search_TargetOffsetDeg(1000U)) <= 15.001f);
    assert(fabsf((Course3Search_TargetOffsetDeg(1001U) -
                  Course3Search_TargetOffsetDeg(1000U)) *
                 (3.14159265358979323846f / 180.0f) / 0.001f) <= 0.501f);
    return 0;
}
