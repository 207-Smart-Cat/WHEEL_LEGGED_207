# Distance-Triggered Triple Jump Design

## Objective

Build a screen-controlled triple-jump workflow on top of the latest `p3` PWM jump behavior. The operator configures three ground distances and one forward-speed command, presses `Go`, and the vehicle performs three jumps while holding the yaw captured at start. Wheel spin while airborne must not contribute to odometry. After the third confirmed landing, any fault, or an operator stop, the vehicle stops and returns to `Standby`.

## Scope

This change covers:

- one reusable asynchronous jump-action API;
- a reusable Z-axis landing detector;
- a distance-triggered triple-jump coordinator;
- odometry freezing during each takeoff-to-landing interval;
- a dedicated Jump screen with editable, persistent settings;
- dual-core IPC for commands, configuration, status, and faults;
- migration of the servo-phase portion of the navigation jump to the shared action API.

The existing navigation approach logic (edge exploration, backoff, run-up, waypoint triggering, remote triggering, and course-three routing) remains outside the Jump screen. Entering the Jump screen must never invoke that legacy approach sequence.

## Confirmed User Requirements

- `Go` clears the first segment distance and immediately commands the configured forward speed.
- `Go` captures `IMU_data.filter_result.yaw` once. The captured yaw remains the target through all travel, jump, airborne, and landing phases.
- The first jump triggers at `x1` metres from the `Go` origin.
- The second jump triggers at `x2` metres of grounded forward displacement after the first confirmed landing.
- The third jump triggers at `x3` metres of grounded forward displacement after the second confirmed landing.
- Odometry is frozen from the instant each jump is triggered until landing is confirmed. Encoder reception and wheel-speed control continue.
- The screen shows confirmed landing count as `0/3` through `3/3`.
- A missed landing timeout is a fault, never a successful landing.
- The third confirmed landing commands zero forward speed; after servo recovery the workflow returns to `Standby`.
- The UI exposes only `Go/Standby`, `x1`, `x2`, `x3`, and speed as selectable rows.
- `x1` range is `0.00..1.00 m`; `x2` and `x3` ranges are `0.00..0.20 m`; speed range is `0..300`.
- Distance increments are `0.01 m`; speed increments are `10`.
- Long-pressing `OK` in `Standby` saves all four parameters to Flash.

## Architecture

### Shared single-jump action

`jump_action.c/.h` owns the complete non-blocking servo sequence. Its external contract is:

```c
typedef enum {
    JUMP_ACTION_PROFILE_FIRST = 0,
    JUMP_ACTION_PROFILE_FOLLOWUP
} JumpActionProfile_e;

typedef enum {
    JUMP_ACTION_IDLE = 0,
    JUMP_ACTION_PREPARE,
    JUMP_ACTION_TAKEOFF,
    JUMP_ACTION_AIRBORNE,
    JUMP_ACTION_RECOVER,
    JUMP_ACTION_DONE,
    JUMP_ACTION_FAULT
} JumpActionState_e;

typedef enum {
    JUMP_ACTION_RESULT_NONE = 0,
    JUMP_ACTION_RESULT_LANDED,
    JUMP_ACTION_RESULT_TIMEOUT,
    JUMP_ACTION_RESULT_ABORTED
} JumpActionResult_e;

uint8 JumpAction_Start(JumpActionProfile_e profile);
void JumpAction_Task5ms(float accel_z_g);
void JumpAction_Abort(void);
uint8 JumpAction_IsActive(void);
uint8 JumpAction_IsAirborne(void);
JumpActionState_e JumpAction_GetState(void);
JumpActionResult_e JumpAction_GetResult(void);
JumpActionResult_e JumpAction_ConsumeResult(void);
```

`JumpAction_Start()` is the one-call trigger required by consumers. The 5 ms PIT task advances the action; consumers do not reproduce timing or PWM commands.

The initial profiles preserve the latest `p3/navigation_action.c` values:

| Phase | First jump | Follow-up jump |
| --- | ---: | ---: |
| Prepare target | 370 PWM | 370 PWM |
| Prepare duration | 200 ms | 100 ms |
| Takeoff | 1300 PWM for 180 ms | 1300 PWM for 180 ms |
| Retract | 420 PWM for 80 ms movement + 50 ms hold | same |
| Landing buffer | 450 PWM | same |
| Recovery | 400 PWM for 50 ms | same |

All symmetric servo output passes through one bounded helper, preserving the `1500` PWM sum and the jump-specific `150..1350` bounds.

Only `jump_action.c` owns jump servo timing and landing transitions. The old standalone sequence in `jump_control.c` is removed or converted to thin compatibility wrappers; it must not remain as a second implementation.

### Landing detector

`landing_detector.c/.h` is a deterministic, reusable state machine driven by one Z-axis acceleration sample every 5 ms:

```c
typedef enum {
    LANDING_DETECTOR_WAIT_AIRBORNE = 0,
    LANDING_DETECTOR_WAIT_IMPACT,
    LANDING_DETECTOR_WAIT_SETTLE,
    LANDING_DETECTOR_LANDED,
    LANDING_DETECTOR_TIMEOUT
} LandingDetectorState_e;

void LandingDetector_Reset(float baseline_z_g);
LandingDetectorState_e LandingDetector_Update(float accel_z_g);
```

While grounded, a rolling 100 ms baseline determines the gravity sign and nominal Z-axis magnitude. An invalid baseline outside `0.60..1.40 g` blocks `Go` with a sensor fault.

Detection follows an ordered sequence:

1. The detector is reset when the action starts, but takeoff preparation and burst cannot produce a landing event.
2. On entry to the airborne phase, three low-gravity samples within four consecutive samples must fall below `0.65 * baseline` before landing detection is armed.
3. Hard landing path: at least two samples in a four-sample window must exceed `baseline + 0.50 g`, followed by four stable samples within five samples inside `baseline +/- 0.35 g`.
4. Soft landing path: after at least 80 ms in an armed airborne state, six consecutive samples inside `baseline +/- 0.25 g` also confirm landing.
5. If neither path confirms landing within 800 ms after entering the airborne phase, the detector reports `TIMEOUT`.

The detector normalizes the axis sign from the measured baseline. A takeoff impulse, an isolated noise spike, or return to the gravity band without prior airborne qualification cannot count as a landing.

### Triple-jump coordinator

`triple_jump.c/.h` owns the screen workflow:

```c
typedef struct {
    float x1_m;
    float x2_m;
    float x3_m;
    float speed;
} TripleJumpConfig_t;

typedef enum {
    TRIPLE_JUMP_STANDBY = 0,
    TRIPLE_JUMP_DRIVING,
    TRIPLE_JUMP_EXECUTING,
    TRIPLE_JUMP_RECOVERING,
    TRIPLE_JUMP_FAULT
} TripleJumpState_e;

uint8 TripleJump_Start(const TripleJumpConfig_t *config, float current_yaw_deg);
void TripleJump_Task5ms(void);
void TripleJump_Stop(void);
TripleJumpState_e TripleJump_GetState(void);
float TripleJump_GetSegmentDistance(void);
uint8 TripleJump_GetLandingCount(void);
uint8 TripleJump_GetFault(void);
```

The coordinator owns the configured speed and locked yaw for the lifetime of a run. It writes the locked value to `target_angle` on every 5 ms tick so no other non-emergency workflow can drift the direction target.

The state flow is:

```text
STANDBY
  -> Go: validate, capture yaw, zero segment origin
DRIVING
  -> segment distance >= x[n]: freeze odometry, start jump profile
EXECUTING
  -> detector confirms landing: increment count, reset origin, unfreeze odometry
RECOVERING
  -> grounded distance may accumulate, but no new jump starts before recovery ends
  -> landing count < 3: DRIVING
  -> landing count == 3: speed 0, STANDBY
FAULT
  -> speed 0, abort servo action, unfreeze odometry, STANDBY after reporting fault
```

If a configured distance is zero, its jump starts on the first eligible 5 ms tick. A follow-up jump cannot start while the preceding jump is still recovering.

### Odometry behavior

Wheel telemetry must continue during a jump. Discarding UART encoder frames would leave stale values in the speed loop, so `jump_should_suspend_encoder()` must not be used to stop reception.

Instead, `navigation_data_handling.c/.h` exposes an explicit odometry-freeze interface. While frozen:

- IMU attitude and yaw continue to update;
- encoder samples remain available to motor control;
- the navigation EKF does not fuse airborne wheel speed;
- `robot_pose.x` and `robot_pose.y` do not integrate;
- the triple-jump segment distance does not change.

Immediately after confirmed landing, the coordinator establishes the new segment origin and releases the freeze. Servo recovery may continue, but any real grounded movement from the landing point is therefore included in the next segment. Grounded segment distance is the yaw-projected net displacement from that origin, clamped to a minimum of zero. Reverse motion therefore cancels forward displacement instead of increasing travelled distance.

Every stop, timeout, supervisor abort, and emergency path must release the freeze exactly once.

### Navigation integration

The existing navigation outer states retain their responsibilities:

- edge exploration;
- edge-touch settling;
- backoff and run-up;
- waypoint and remote triggers;
- follow-up approach distance;
- route completion.

Once those states request a takeoff, servo sequencing and landing detection are delegated to `JumpAction_Start()` and `JumpAction_Task5ms()`. Navigation reads the shared phase and result to set its speed policy, airborne flag, and next outer state. This keeps existing route behavior while creating one source of truth for the jump action.

The Jump screen never calls `IPC_Request_Nav_Jump()` or `Navi_Jump_Start()`.

## UI and Input Behavior

Selecting `Jump` on the home screen opens `UI_SCREEN_JUMP`. It no longer triggers an action immediately.

The screen contains five selectable rows and three read-only telemetry rows:

```text
------ TRIPLE JUMP ------
Mode       Standby / Go
X1         0.50 m
X2         0.15 m
X3         0.15 m
Speed      0
Landings   0 / 3
Distance   0.00 m
Status     READY
```

- `UP/DOWN` selects a row.
- `OK` toggles `Standby/Go` on the mode row.
- `OK` enters or accepts editing on a parameter row.
- `UP/DOWN` edits by `0.01 m` or `10` and clamps at the required bounds.
- `BACK` cancels an edit. Outside editing it stops an active run before returning home.
- Long `OK` while in `Standby` stores all four values.
- Editing and Flash writes are rejected while a run is active.

The status field distinguishes at least `READY`, `RUNNING`, `AIRBORNE`, `RECOVER`, `DONE`, `IMU ERROR`, `BUSY`, `TIMEOUT`, `ABORTED`, `SAVED`, and `SAVE ERROR`.

## Persistent Configuration

The generic parameter registry already contains 63 parameters and uses a 64-bit update mask. The four triple-jump values therefore use a dedicated versioned structure rather than extending that registry.

Flash page allocation is:

- pages `0..79`: navigation groups;
- page `94`: triple-jump configuration;
- page `95`: existing generic parameters.

The page-94 record contains a magic value, format version, `TripleJumpConfig_t`, and CRC32. Load accepts only a valid magic/version/CRC and finite values inside all four ranges. Otherwise it loads safe defaults:

```text
x1 = 0.50 m
x2 = 0.15 m
x3 = 0.15 m
speed = 0
mode = Standby
```

The configuration loads at Core 1 startup before the main UI appears. Mode and progress are never persisted.

## IPC Contract

The shared-memory command block gains a dedicated triple-jump configuration plus monotonically increasing `start_seq` and `stop_seq`. Core 0 remembers the last consumed values, making repeated polling safe and preventing a boolean edge from being lost.

Core 0 status exposes:

- coordinator state;
- confirmed landing count;
- current segment distance;
- locked yaw;
- jump-action phase;
- fault code;
- last acknowledged start/stop sequence.

Compile-time shared-memory size assertions remain mandatory after adding fields.

## Safety and Arbitration

`Go` is accepted only when:

- system and IMU initialization are complete;
- the measured Z-axis gravity baseline is valid;
- motor, balance, and servo modules required by the workflow are enabled; `Go` enables the navigation/odometry module before clearing its origin;
- emergency stop is inactive;
- no standalone jump, navigation jump, bump action, or course-three action owns the actuators;
- configuration values are finite and in range.

Failure leaves the workflow in `Standby`, commands zero speed, and reports a specific fault. Emergency stop and vehicle-supervisor reset abort the action, clear motion ownership, and release odometry freeze. The final landing commands zero speed immediately; servo recovery completes before the state becomes `Standby`.

## Testing Strategy

Host-side deterministic tests use injected acceleration, time, pose, and action results. Required cases are:

- jump PWM phase boundaries for first and follow-up profiles;
- symmetric PWM clamping;
- takeoff impulse cannot count as landing;
- isolated acceleration spikes cannot count as landing;
- hard-impact-plus-settle landing;
- soft landing after airborne qualification;
- timeout without a false landing count;
- `Go` captures yaw once and clears the first segment;
- `x1`, `x2`, and `x3` trigger in order, including zero-distance values;
- segment distance remains unchanged while airborne;
- reverse displacement cancels forward displacement;
- each confirmed landing increments exactly once;
- the third landing stops and returns to `Standby` after recovery;
- stop, fault, and emergency paths release odometry freeze;
- configuration limits, increments, default fallback, CRC rejection, and Flash round trip;
- Jump home selection opens the screen without invoking the legacy navigation jump;
- existing navigation action tests still pass after shared-action migration.

The final verification also includes the repository's existing host tests and an IAR project build when the configured toolchain is available.

## Success Criteria

- A caller can trigger the entire servo jump with one `JumpAction_Start()` call.
- No jump servo timing is duplicated between the Jump screen path and navigation path.
- Airborne wheel spin produces zero change in navigation position and triple-jump segment distance.
- Only a qualified and stabilized Z-axis landing increments the displayed count.
- The three trigger distances are measured from `Go`, first landing, and second landing respectively.
- The captured yaw does not change during a run.
- Third landing, timeout, operator stop, and emergency stop all leave the vehicle stopped, in a safe servo state, with odometry unfrozen.
- Saved values survive restart and invalid Flash data falls back to safe defaults.
