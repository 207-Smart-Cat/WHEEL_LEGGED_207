# Course 3 Navigation Execution Display Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Course 3 execution information page, restrict automatic vision entry to alignment, and make center-line search continuous, faster, and angular-rate limited.

**Architecture:** Core A will publish current Course 3 target telemetry through the existing status registry. Core B will render it on a dedicated IPS200 page and temporarily switch to vision only for `TRACK_ALIGN`. A pure helper will provide target labels and the bounded sinusoidal search offset for host tests.

**Tech Stack:** C, Core A/Core B IPC, IPS200 display, host `assert` tests.

## Global Constraints

- Course 3 UI target types: normal, bridge, stair.
- Reached normal points advance immediately without an action state.
- Search speed: `100.0`; scan amplitude: `15°`; target angular-rate maximum: `0.5 rad/s`.
- The scan continues until vision is valid; low-battery UI remains higher priority.

---

### Task 1: Test and implement pure display/search helpers

**Files:**
- Modify: `project/code/course3_display_state.h`
- Modify: `project/code/course3_display_state.c`
- Modify: `tests/test_course3_display_state.c`

**Interfaces:**
- `const char *Course3TargetType_Text(uint8_t waypoint_type)`
- `uint8_t Course3Vision_ShouldEnter(uint8_t vehicle_mode, uint8_t state, uint8_t already_active)`
- `float Course3Search_TargetOffsetDeg(uint32_t elapsed_ms)`

- [ ] **Step 1: Write failing assertions**

```c
assert(strcmp(Course3TargetType_Text(WP_TYPE_NORMAL), "NORMAL") == 0);
assert(strcmp(Course3TargetType_Text(WP_TYPE_BRIDGE), "BRIDGE") == 0);
assert(strcmp(Course3TargetType_Text(WP_TYPE_JUMP), "STAIR") == 0);
assert(Course3Vision_ShouldEnter(3U, COURSE3_DISPLAY_ACTION, 0U) == 0U);
assert(fabsf(Course3Search_TargetOffsetDeg(0U)) < 0.001f);
```

- [ ] **Step 2: Verify red**

Run: `gcc -Iproject/code tests/test_course3_display_state.c project/code/course3_display_state.c -o tests/test_course3_display_state.exe`

Expected: compile failure because the target label and scan APIs do not exist; current vision predicate accepts `ACTION`.

- [ ] **Step 3: Implement minimal helpers**

```c
#define COURSE3_SEARCH_AMPLITUDE_RAD (0.2617994f)
#define COURSE3_SEARCH_MAX_RATE_RAD_S (0.5f)

float Course3Search_TargetOffsetDeg(uint32_t elapsed_ms)
{
    float omega = COURSE3_SEARCH_MAX_RATE_RAD_S / COURSE3_SEARCH_AMPLITUDE_RAD;
    return 15.0f * sinf(omega * ((float)elapsed_ms * 0.001f));
}
```

Map the three waypoint types, and return true from the vision predicate only for Course 3 `TRACK_ALIGN`.

- [ ] **Step 4: Verify green**

Run: `gcc -Iproject/code tests/test_course3_display_state.c project/code/course3_display_state.c -lm -o tests/test_course3_display_state.exe; .\\tests\\test_course3_display_state.exe`

Expected: exit code 0, including amplitude and numerical derivative-bound assertions.

- [ ] **Step 5: Commit**

```bash
git add project/code/course3_display_state.[ch] tests/test_course3_display_state.c
git commit -m "feat: add course3 search display logic"
```

### Task 2: Publish Course 3 execution telemetry and apply navigation behavior

**Files:**
- Modify: `project/code/navigation_action.h`
- Modify: `project/code/navigation_action.c`
- Modify: `project/code/navigation_tracking.c`
- Modify: `project/code/status_registry.def`
- Create: `tests/test_course3_navigation_logic.c`

**Interfaces:**
- Status fields: `course3_exec_active`, `course3_target_type`, `course3_target_x`, `course3_target_y`, `course3_target_yaw`, `course3_error_x`, `course3_error_y`, `course3_error_yaw`, `course3_distance`.
- `uint8_t Course3Waypoint_RequiresAlign(uint8_t type)`.

- [ ] **Step 1: Write failing test**

```c
assert(Course3Waypoint_RequiresAlign(WP_TYPE_NORMAL) == 0U);
assert(Course3Waypoint_RequiresAlign(WP_TYPE_BRIDGE) == 1U);
assert(Course3Waypoint_RequiresAlign(WP_TYPE_JUMP) == 1U);
```

- [ ] **Step 2: Verify red**

Run: `gcc -Iproject/code tests/test_course3_navigation_logic.c project/code/course3_display_state.c -o tests/test_course3_navigation_logic.exe`

Expected: compile failure because the helper does not exist.

- [ ] **Step 3: Implement minimal behavior**

Add the helper and use it so only bridge/stair targets start `FSM_COURSE3_TRACK_ALIGN`; remove the Course 3 normal-point transition to `FSM_COURSE3_ACTION`. Add status accessors based on `point_map[navi_ctrl.point_current_idx]` and `robot_pose`, inactive outside Course 3 execution. In the search branch set `target_velocity = 100.0f` and calculate target yaw from `Course3Search_TargetOffsetDeg(state_timer_ms)`; do not introduce a search timeout.

- [ ] **Step 4: Verify green**

Run: `gcc -Iproject/code tests/test_course3_navigation_logic.c project/code/course3_display_state.c -o tests/test_course3_navigation_logic.exe; .\\tests\\test_course3_navigation_logic.exe`

Expected: exit code 0; normal targets do not require alignment.

- [ ] **Step 5: Commit**

```bash
git add project/code/navigation_action.[ch] project/code/navigation_tracking.c project/code/status_registry.def tests/test_course3_navigation_logic.c
git commit -m "feat: publish course3 execution target state"
```

### Task 3: Render and transition the Core B execution information screen

**Files:**
- Modify: `project/code/screen_display.c`
- Modify: `tests/test_course3_display_state.c`

**Interfaces:**
- Consumes Task 2 `core_a_status.course3_*` fields.
- Adds `UI_SCREEN_COURSE3_EXEC` and `ui_draw_course3_exec()`.

- [ ] **Step 1: Extend failing predicate test**

```c
assert(Course3Vision_ShouldEnter(3U, COURSE3_DISPLAY_IDLE, 0U) == 0U);
assert(Course3Vision_ShouldEnter(3U, COURSE3_DISPLAY_TRACK_ALIGN, 0U) == 1U);
assert(Course3Vision_ShouldEnter(3U, COURSE3_DISPLAY_ACTION, 0U) == 0U);
```

- [ ] **Step 2: Verify red**

Run: `gcc -Iproject/code tests/test_course3_display_state.c project/code/course3_display_state.c -lm -o tests/test_course3_display_state.exe; .\\tests\\test_course3_display_state.exe`

Expected: the current predicate fails the `ACTION` assertion.

- [ ] **Step 3: Implement page**

After execute-load completion choose `UI_SCREEN_COURSE3_EXEC`. Render type, current pose, target pose, `dX/dY/dYaw`, distance, and state. On `TRACK_ALIGN`, save execution page and enter vision. After alignment/action, restore execution page when `course3_exec_active` remains true; otherwise restore the pre-execution screen.

- [ ] **Step 4: Verify green**

Run: `gcc -Iproject/code tests/test_course3_display_state.c project/code/course3_display_state.c -lm -o tests/test_course3_display_state.exe; .\\tests\\test_course3_display_state.exe`

Expected: exit code 0.

- [ ] **Step 5: Commit**

```bash
git add project/code/screen_display.c tests/test_course3_display_state.c
git commit -m "feat: show course3 execution navigation page"
```

### Task 4: Regression verification

**Files:**
- Verify: `tests/test_course3_display_state.c`
- Verify: `tests/test_course3_navigation_logic.c`
- Verify: `tests/test_course3_align_logic.c`
- Verify: `tests/test_vision_control.c`
- Verify: `tests/test_bridge_roll_peak.c`

- [ ] **Step 1: Build and run host tests**

```powershell
gcc -Iproject/code tests/test_course3_display_state.c project/code/course3_display_state.c -lm -o tests/test_course3_display_state.exe
.\\tests\\test_course3_display_state.exe
gcc -Iproject/code tests/test_course3_align_logic.c project/code/course3_align_logic.c -lm -o tests/test_course3_align_logic.exe
.\\tests\\test_course3_align_logic.exe
gcc -Iproject/code tests/test_vision_control.c project/code/vision_control.c -lm -o tests/test_vision_control.exe
.\\tests\\test_vision_control.exe
gcc -Iproject/code tests/test_bridge_roll_peak.c project/code/bridge_roll_peak.c -lm -o tests/test_bridge_roll_peak.exe
.\\tests\\test_bridge_roll_peak.exe
```

- [ ] **Step 2: Inspect final state**

Run: `git diff --check; git status --short`

Expected: no whitespace errors and no unexpected tracked changes.

- [ ] **Step 3: Commit plan**

```bash
git add docs/superpowers/plans/2026-07-28-course3-navigation-execution-display.md
git commit -m "docs: plan course3 execution display"
```
