# BUMP Screen Mode Design

## Goal

Add a top-level BUMP item beside Mode Select, Jump, and the other home-menu entries. The page provides a safe manual bump-test mode, live anti-stall telemetry, an isolated target-speed command, an editable assist PWM mapping gain, and dedicated Flash persistence.

## Home Menu

Insert BUMP immediately after Jump.

Core1 will then have nine home items. The home menu therefore keeps a top index and renders at most eight rows. Moving the selection keeps the selected row visible and prevents the ninth item from overlapping the footer.

Selecting BUMP opens UI_SCREEN_BUMP. Entering the page never starts motion automatically.

## Page Contents

The page displays these values:

    BUMP
    BUMP Run    : OFF
    Assist Mode : ON
    Target Speed: 300
    Current Spd : 0.0
    Assist PWM  : 0.0
    PWM Limit   : 4000
    Left PWM    : 0
    Right PWM   : 0
    PWM Gain    : 0.32
    Save Flash

The screen may compact labels and spacing to fit the 240x320 display, but the data set and meanings remain unchanged.

### Data Sources

- BUMP Run: BUMP-page manual-run state.
- Assist Mode: RUNTIME_MODULE_ANTI_STALL, not the instantaneous debug-enabled flag.
- Target Speed: BUMP-only configured speed.
- Current Spd: arithmetic mean of current left and right wheel speeds. It uses the same forward-speed sign convention as the balance controller measured velocity.
- Assist PWM: anti_stall_dbg_pwm.
- PWM Limit: the active anti-stall assist output limit, currently 4000.
- Left PWM and Right PWM: existing final limited Motor_Left and Motor_Right status values.
- PWM Gain: the BUMP configuration value actively used by anti_stall_update().

The existing Core0-to-Core1 status snapshot supplies live values. The screen uses the existing periodic refresh and does not print or block inside the 1 ms balance ISR.

## Page Interaction

The selectable actions are BUMP Run, Assist Mode, Target Speed, PWM Gain, and Save Flash.

- UP/DOWN moves among selectable actions.
- OK on BUMP Run toggles the manual run.
- OK on Assist Mode toggles RUNTIME_MODULE_ANTI_STALL.
- OK on Target Speed or PWM Gain enters inline edit mode.
- In edit mode, UP/DOWN changes the value, OK accepts and exits edit mode, and BACK cancels the edit.
- OK on Save Flash persists both BUMP configuration values and shows a short saved indication.
- BACK outside edit mode stops BUMP Run, commands zero target speed, and returns home.

### PWM Gain

- Default: 0.32.
- Range: 0.00 through 1.00.
- Short UP/DOWN: +/-0.01.
- Long UP/DOWN: +/-0.10.
- Each accepted step is synchronized to Core0 immediately and changes the gain used by anti_stall_update().

### Target Speed

- Default: 300, sourced from the current Course 3 bump-segment speed.
- Range: 0 through 800.
- Short UP/DOWN: +/-10.
- Long UP/DOWN: +/-50.
- Editing updates only the BUMP configuration. It does not change the global navigation target-speed parameter or COURSE3_AUX_SEGMENT_SPEED.

## Run Safety and Isolation

Add MANUAL_TEST_MODE_BUMP to the existing manual-test mode enumeration.

BUMP Run is OFF whenever the page is entered. While it is ON and the BUMP page remains active, Core0 continuously owns target_velocity and sets it to the BUMP-only target speed. The remote callback does not overwrite the command during this manual-test mode.

The command is forced to zero when any of these occurs:

- BUMP Run is switched OFF.
- The page is exited.
- Emergency stop becomes active.
- WiFi or motor safety gating stops motor output.
- The BUMP manual-test mode is cleared or replaced.

After the BUMP mode is cleared, normal remote and navigation ownership resumes. Course 3 bump execution continues to use the existing fixed COURSE3_AUX_SEGMENT_SPEED value and never reads the BUMP-page target speed.

## BumpConfig and IPC

Do not add these values to the existing 63-entry generic parameter registry. That registry is bounded by the current 64-bit update mask and VOFA frame sizing, while both new values are explicitly BUMP-specific.

Add a dedicated BUMP configuration to the CoreB command area:

- bump_pwm_gain;
- bump_target_speed;
- configuration update sequence or flag;
- BUMP run command.

Core1 setters validate and publish the values. Core0 consumes updates in the existing 10 ms IPC synchronization path:

- It copies bump_pwm_gain into the runtime gain used by anti_stall_update().
- It applies bump_target_speed to target_velocity only while MANUAL_TEST_MODE_BUMP and BUMP Run are active.

Core0 publishes the active gain, PWM limit, current arithmetic-mean speed, assist PWM, and wheel PWM values through the existing status registry.

## Flash Persistence

Use a dedicated, versioned BUMP configuration record in an unused Flash page outside:

- navigation storage pages 0-79;
- camera configuration pages 93-94;
- generic parameter page 95.

The record contains magic, version, a sequence or reserved field, bump_pwm_gain, bump_target_speed, and an integrity value.

At startup, invalid or absent BUMP configuration uses defaults 0.32 and 300. Saving from the BUMP page writes and verifies the dedicated record. Generic parameter save and load remain unchanged.

## Control Algorithm

Replace the local ANTI_STALL_PWM_GAIN value 0.32 inside anti_stall_update() with the validated runtime BUMP gain. All other anti-stall thresholds, integration behavior, reset reasons, and PWM limiting remain unchanged.

The active PWM limit is exposed from the same source used by the control algorithm so the page cannot display a stale duplicated value.

## Testing

Behavioral tests cover:

1. PWM gain and target-speed short and long steps and clamping.
2. BUMP Run entering OFF, explicit start, explicit stop, and exit-to-zero behavior.
3. Manual BUMP target speed ownership without affecting Course 3 speed constants.
4. BUMP configuration defaulting, serialization, validation, and Flash-load fallback.
5. Arithmetic-mean current-speed calculation with the balance-controller sign convention.
6. Existing Course 3 bump-assist and inertial-mode regressions.

Structural integration checks cover the home-menu entry, scrolling, screen dispatch and render wiring, status registry fields, and both-core IPC structure size guards.
