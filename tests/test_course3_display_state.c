#include <assert.h>
#include <string.h>

#include "course3_display_state.h"

int main(void)
{
    assert(strcmp(Course3DisplayState_Text(COURSE3_DISPLAY_TRACK_ALIGN), "TRACK ALIGN") == 0);
    assert(strcmp(Course3DisplayState_Text(COURSE3_DISPLAY_ACTION), "ACTION") == 0);
    assert(strcmp(Course3DisplayState_Text(COURSE3_DISPLAY_DONE), "DONE") == 0);
    assert(Course3DisplayState_Text(COURSE3_DISPLAY_IDLE) == 0);

    assert(Course3Vision_ShouldEnter(3U, COURSE3_DISPLAY_TRACK_ALIGN, 0U) == 1U);
    assert(Course3Vision_ShouldEnter(2U, COURSE3_DISPLAY_TRACK_ALIGN, 0U) == 0U);
    assert(Course3Vision_ShouldEnter(3U, COURSE3_DISPLAY_IDLE, 0U) == 0U);
    assert(Course3Vision_ShouldRestore(COURSE3_DISPLAY_IDLE, 1U) == 1U);
    assert(Course3Vision_ShouldRestore(COURSE3_DISPLAY_ACTION, 1U) == 0U);
    return 0;
}
