# Course 3 Waypoint Framework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow screen-selected Course 3 to record normal, bridge, and stair waypoints with CH4 while bridge and stair waypoints keep ordinary tracking until their sub-FSMs are added.

**Architecture:** The existing `record_point_map` and `point_map` continue to persist `WayPoint_Type`. The remote layer selects type using the screen-selected vehicle mode. The action layer queues only mine-sweep and non-Course-3 jump points, so Course-3 bridge and stair points are non-blocking semantic markers.

**Tech Stack:** C for CYT4BB7/IAR, PowerShell source-contract regression checks, existing navigation modules.

## Global Constraints

- Do not alter `Navi_WayPoint_t`, `record_point_map`, `point_map`, or the CH6 edge-trigger recording flow.
- Keep Course 2 CH4 mapping as normal/mine-sweep/jump.
- Use Course 3 CH4 low/mid/high mapping as normal/bridge/jump.
- Course 3 bridge and jump points must not enter `action_seq`, set `is_action_busy`, or change vehicle control.
- Waypoints are recorded near obstacles with a safe offset; this change adds no bridge-distance or sensor logic.

---

### Task 1: Add Course 3 framework regression check

**Files:**
- Create: `project/code/navigation_course3_framework_test.ps1`

**Interfaces:**
- Consumes: `Remote_GetRecordPointType()` in `project/code/remote.c` and `navi_parse_global_path()` in `project/code/navigation_action.c`.
- Produces: A host-runnable source-contract check for Course-2/3 mappings and non-blocking Course-3 actions.

- [x] **Step 1: Write the failing check**

Create a PowerShell script which reads both source files and fails if these required contracts are absent:

```powershell
if ($remote -notmatch 'VEHICLE_MODE_COURSE_3') { throw 'Course 3 mapping branch was not found.' }
if ($remote -notmatch 'return WP_TYPE_BRIDGE;') { throw 'Course 3 mid switch must record bridge points.' }
if ($action -notmatch 'mode != VEHICLE_MODE_COURSE_3') { throw 'Course 3 jumps must not enter the action sequence.' }
```

- [x] **Step 2: Run the check and confirm failure**

Run `powershell -ExecutionPolicy Bypass -File project/code/navigation_course3_framework_test.ps1`.

Expected: failure because remote mapping lacks Course 3 and all jump points are queued as actions.

- [x] **Step 3: Extend the check to the full contract**

Check that Course 2 still returns `WP_TYPE_MINE_SWEEP`; Course 3 divides CH4 at `REMOTE_CH4_MID_THRESHOLD` and `REMOTE_CH4_HIGH_THRESHOLD`, returning normal/bridge/jump; and action parsing has only mine-sweep plus non-Course-3 jump points. Print `course 3 waypoint framework checks passed` on success.

### Task 2: Map CH4 by the screen-selected Course 3 mode

**Files:**
- Modify: `project/code/remote.c:253-272`

**Interfaces:**
- Consumes: `Runtime_Get_Vehicle_Mode()`, `Remote_GetChannelData(4)`, `REMOTE_CH4_MID_THRESHOLD`, `REMOTE_CH4_HIGH_THRESHOLD`.
- Produces: `Remote_GetRecordPointType()` returns normal, bridge, or jump in Course 3.

- [x] **Step 1: Preserve Course 2 behavior**

Keep `VEHICLE_MODE_COURSE_2`: high returns `WP_TYPE_JUMP`, mid returns `WP_TYPE_MINE_SWEEP`, and low falls back to `WP_TYPE_NORMAL`.

- [x] **Step 2: Add Course 3 mapping**

Add this branch after Course 2:

```c
if (mode == VEHICLE_MODE_COURSE_3)
{
    int32_t ch4 = Remote_GetChannelData(4);

    if (ch4 < REMOTE_CH4_MID_THRESHOLD)
    {
        return WP_TYPE_NORMAL;
    }
    if (ch4 < REMOTE_CH4_HIGH_THRESHOLD)
    {
        return WP_TYPE_BRIDGE;
    }
    return WP_TYPE_JUMP;
}
```

Add a concise comment: Course-3 CH4 low/mid/high equals normal/bridge/stair; CH6 triggers recording and the navigation layer creates the initial HOME point.

- [x] **Step 3: Run the regression check**

Run `powershell -ExecutionPolicy Bypass -File project/code/navigation_course3_framework_test.ps1`.

Expected: only the non-blocking Course-3 action check fails.

### Task 3: Keep Course 3 bridge and stair actions empty

**Files:**
- Modify: `project/code/navigation_action.c:1-10`
- Modify: `project/code/navigation_action.c:244-276`

**Interfaces:**
- Consumes: `Runtime_Get_Vehicle_Mode()` and `VEHICLE_MODE_COURSE_3`.
- Produces: `navi_parse_global_path()` queues only mine-sweep and non-Course-3 jump actions.

- [x] **Step 1: Add the runtime-mode header**

Add `#include "runtime_status.h"` beside existing navigation dependencies.

- [x] **Step 2: Lock action parsing to the selected mode**

Inside `navi_parse_global_path()`, read the mode once before the loop and use:

```c
uint8_t mode = Runtime_Get_Vehicle_Mode();

if (point_map[i].type == WP_TYPE_MINE_SWEEP ||
    (point_map[i].type == WP_TYPE_JUMP && mode != VEHICLE_MODE_COURSE_3))
```

Keep `point_map[i].action_cmd = NAVI_JUMP_ACTION_MODE;` inside that condition, so it is not written for Course-3 stair markers.

- [x] **Step 3: Document future sub-FSM handoff**

Add a comment beside the condition: Course-3 bridge and stair points are intentionally omitted from `action_seq` and tracking advances them normally. Future sub-FSMs must enqueue both types and complete through `action_done_pending/action_done_idx`. Do not modify `FSM_BRIDGE_*` or `FSM_JUMP_*` outputs now.

- [x] **Step 4: Run the full regression check**

Run `powershell -ExecutionPolicy Bypass -File project/code/navigation_course3_framework_test.ps1`.

Expected: pass with `course 3 waypoint framework checks passed`.

### Task 4: Verify and record completed work

**Files:**
- Modify: `Knowledge/2026-07-28-course3-waypoint-framework-plan.md`
- Modify: `project/code/remote.c`
- Modify: `project/code/navigation_action.c`
- Test: `project/code/navigation_course3_framework_test.ps1`

**Interfaces:**
- Consumes: Tasks 1-3.
- Produces: A delivered Course-3 waypoint framework and completed plan in `Knowledge`.

- [x] **Step 1: Check scope**

Run `git diff --check` and `git status --short`. Stage only the two source files, new PowerShell test, and this plan; exclude the unrelated `tests/test_vision_control.exe`.

- [x] **Step 2: Rerun verification**

Run `powershell -ExecutionPolicy Bypass -File project/code/navigation_course3_framework_test.ps1`. Expected: pass.

- [x] **Step 3: Mark the plan complete**

Change every completed checkbox to `[x]`; append the command, pass result, and the limitation that no IAR hardware build ran in this workspace.

- [x] **Step 4: Commit implementation**

Stage the two source files, the PowerShell check, and this plan, then commit with message `feat: add course 3 waypoint framework`.

## Self-Review

- The plan covers type mapping, unchanged storage, Course-2 isolation, empty Course-3 bridge/stair actions, and the future sub-FSM completion hook.
- It adds no map structure, sensor, bridge-length, speed, or posture behavior.
- Every named file, enum, function, and macro exists in the current repository.

## Verification Record

- 2026-07-28: `powershell -ExecutionPolicy Bypass -File project/code/navigation_course3_framework_test.ps1` — PASS.
- Limitation: this workspace has no IAR build runner or connected hardware, so the target firmware build and on-vehicle CH4/CH6 test remain to be performed.
