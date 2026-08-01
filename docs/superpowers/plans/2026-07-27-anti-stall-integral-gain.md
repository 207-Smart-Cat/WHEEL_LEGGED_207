# Anti-stall Integral Gain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change the anti-stall assist integral error accumulation gain from `1.0f` to `1.6f`.

**Architecture:** The change is localized to `anti_stall_update()` in the motor-control source. A named local constant documents the gain and is multiplied by `speed_error` before applying the existing integral clamp. Reset conditions and the integral-to-PWM mapping are untouched.

**Tech Stack:** C firmware; IAR Embedded Workbench project.

## Global Constraints

- Use `ANTI_STALL_INTEGRAL_GAIN = 1.6f`.
- Retain the integral clamp `[0.0f, 50000.0f]`.
- Retain the PWM mapping gain `0.04f` and PWM clamp `[0.0f, 6000.0f]`.

---

### Task 1: Apply and verify the anti-stall accumulation gain

**Files:**
- Modify: `project/code/control.c:150-192`
- Test: source-level verification of `project/code/control.c`

**Interfaces:**
- Consumes: `speed_error`, calculated as `target_velocity_cmd - measured_velocity`.
- Produces: `g_anti_stall_assist.integral`, retained as a `float` and subsequently mapped to `assist_pwm`.

- [ ] **Step 1: Define the expected source behavior**

The accumulation expression must be:

```c
g_anti_stall_assist.integral += ANTI_STALL_INTEGRAL_GAIN * speed_error;
```

and its constant must be:

```c
const float ANTI_STALL_INTEGRAL_GAIN = 1.6f;
```

- [ ] **Step 2: Verify the pre-change source does not meet the expectation**

Run:

```powershell
rg -n 'ANTI_STALL_INTEGRAL_GAIN|integral \+=.*speed_error' project/code/control.c
```

Expected: no gain constant exists; the accumulation expression omits the gain.

- [ ] **Step 3: Make the minimal implementation change**

Add the gain beside the existing anti-stall constants and multiply `speed_error` by it in the existing accumulation statement:

```c
const float ANTI_STALL_INTEGRAL_GAIN = 1.6f;
g_anti_stall_assist.integral += ANTI_STALL_INTEGRAL_GAIN * speed_error;
```

- [ ] **Step 4: Verify the source behavior**

Run:

```powershell
rg -n 'ANTI_STALL_INTEGRAL_GAIN = 1\.6f|integral \+= ANTI_STALL_INTEGRAL_GAIN \* speed_error|ANTI_STALL_INTEGRAL_LIMIT|ANTI_STALL_PWM_GAIN' project/code/control.c
```

Expected: the new constant and multiplication are present; the existing integral and PWM constants remain `50000.0f` and `0.04f`.

- [ ] **Step 5: Commit**

```powershell
git add project/code/control.c docs/superpowers/specs/2026-07-27-anti-stall-integral-gain-design.md docs/superpowers/plans/2026-07-27-anti-stall-integral-gain.md
git commit -m "tune: increase anti-stall integral gain"
```
