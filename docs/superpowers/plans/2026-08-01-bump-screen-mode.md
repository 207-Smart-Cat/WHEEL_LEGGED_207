# BUMP Screen Mode Implementation Plan

> **For Codex:** Execute this plan with `superpowers:executing-plans`; use strict red-green-refactor for each behavioral unit.

**Goal:** Add a safe, top-level BUMP screen that exposes live anti-stall telemetry, controls an isolated manual target speed, edits the anti-stall PWM gain, and persists its two configuration values independently in Flash.

**Architecture:** Put platform-independent BUMP configuration, adjustment, speed conversion, run-state, and record-validation behavior in a small pure C module with host tests. Extend the existing Core1-to-Core0 IPC command snapshot for BUMP settings/run state and the Core0-to-Core1 status registry for telemetry. Keep Course 3 on its existing compile-time speed and use a dedicated versioned Flash page so the 63-entry generic parameter registry remains unchanged.

**Tech Stack:** Embedded C, Infineon CYT4BB7 dual-core shared-memory IPC, IAR project files, IPS200 UI, MinGW GCC host tests, PowerShell regression tests.

---

## Task 1: Pure BUMP behavior and persistence record

**Files:**
- Create: `project/code/bump_mode_logic.h`
- Create: `project/code/bump_mode_logic.c`
- Create: `tests/test_bump_mode_logic.c`

1. Write host tests for default values, gain/speed short and long steps, clamping, balance-sign current speed, run ownership/stop transitions, record build/validation, corrupt-record rejection, and fallback defaults.
2. Compile the test against the not-yet-existing module and confirm RED because the BUMP API is absent.
3. Implement only the pure behavior needed by the tests, including the single source for the active assist PWM limit.
4. Recompile and run the test; confirm GREEN.

## Task 2: Dedicated IPC command and Flash configuration

**Files:**
- Modify: `project/code/ipc_shared_data.h`
- Modify: `project/code/ipc_shared_data.c`
- Modify: `project/user/main_cm7_1.c` (or actual Core1 startup path found in tree)
- Modify: both IAR `.ewp` files if a new source group entry is required

1. Add `MANUAL_TEST_MODE_BUMP`, dedicated gain/target/run fields, getters/setters, load/save APIs, and size guards.
2. Store the versioned record on an unused page outside navigation 0-79, camera 93-94, and generic parameters 95; verify the write and fall back to defaults on invalid data.
3. Initialize the BUMP configuration on Core1 startup and publish validated defaults/settings to shared memory.
4. Build/run the pure record tests again and inspect compiler-visible IPC assertions where the embedded compiler is unavailable.

## Task 3: Control-loop integration and live telemetry

**Files:**
- Modify: `project/code/control.c`
- Modify: `project/code/control.h`
- Modify: `project/code/status_registry.def`
- Modify: `project/code/remote.c`

1. Replace the local `0.32` anti-stall gain with the validated runtime BUMP gain and use the shared PWM-limit definition.
2. Publish current speed, active gain, assist PWM, PWM limit, and final left/right PWM through the existing status snapshot.
3. Make BUMP manual mode own `target_velocity` only while its explicit run flag is ON; force zero on run-off, mode exit/replacement, emergency, and safety gating.
4. Preserve all non-BUMP remote/navigation behavior and keep Course 3 reading only `COURSE3_AUX_SEGMENT_SPEED`.
5. Run the Course 3 bump/inertial PowerShell regressions and the BUMP host tests.

## Task 4: BUMP screen and home-menu scrolling

**Files:**
- Modify: `project/code/screen_display.c`
- Modify: `project/code/screen_display.h` if required

1. Add `BUMP` immediately after `JUMP`, introduce `UI_SCREEN_BUMP`, and render at most eight home rows while keeping selection visible.
2. Render run state, real assist-module state, target/current speeds, assist PWM, active limit, final wheel PWMs, gain, and Save Flash.
3. Add navigation/edit behavior: run and module toggles, short/long value steps, OK accept, BACK cancel, Save Flash feedback, and exit-to-zero.
4. Ensure entering the page sets BUMP Run OFF and leaving it clears run and manual mode.
5. Add/run a focused integration harness where practical, then run existing display-related host tests.

## Task 5: Project integration and verification

**Files:**
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`

1. Add the pure module to both relevant embedded projects and confirm include paths/symbol ownership are valid for both cores.
2. Run all host C tests with warnings enabled and all PowerShell regression tests.
3. Run `git diff --check`, review the complete diff, verify no generic registry entries were added, and verify Course 3 still uses constant 300 independently.
4. Commit the implementation on branch `bump` after verification.
