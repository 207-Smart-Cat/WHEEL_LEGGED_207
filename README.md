# WHEEL_LEGGED_207

Wheeled-legged robot control project for the CYT4BB7 platform.

## Project Layout

- `project/user`: core entry points and interrupt handlers.
- `project/code`: application logic, UI, communication, control, navigation, and runtime modules.
- `libraries`: SEEKFREE CYT4BB7 drivers, devices, SDK, and common utilities.
- `sim`: MATLAB simulation scripts for the wheeled-legged plant and controller.
- `Knowledge`: project notes and working logs.

## Display Configuration

The current IPS200 display is configured for the dual-row 8-bit parallel interface demo from:

`F:\Wheeled_legged Resources\ScreenDemo\E06_04_ips200_display_demo`

The active display type is selected in `project/code/screen_display.h`:

```c
#define IPS200_TYPE (IPS200_TYPE_PARALLEL8)
```

This routes `screen_display_init()` through the existing `zf_device_ips200` parallel initialization path for RD, WR, RS, RST, CS, BL, and D0-D7.

## Vision Yaw Calibration Parameters

Current Vision-page yaw calibration defaults:

```c
#define VISION_ALIGN_STABLE_WINDOW_FRAMES  (20U)
#define VISION_ALIGN_SAMPLE_COUNT_TARGET   (16U)
#define VISION_ALIGN_SIDE_SAMPLE_TARGET    (8U)
#define VISION_ALIGN_STABLE_ERROR_PX       (40)
#define VISION_ALIGN_SAMPLE_ERROR_PX       (5)
```

- Stable wait requires 20 consecutive valid vision frames within +/-40 px.
- Sampling records 8 left-side and 8 right-side center samples, within +/-5 px.
- Vision calibration drive speed is `target_velocity = 90.0f`; completion sets `target_velocity = 0.0f`.

## BUMP Tuning Changes

Current `bump_1` branch values relative to the previous baseline:

| Item | Previous value | Current value | Scope |
|---|---:|---:|---|
| `Direction_p_init` | `20.0f` | `90.0f` | Global default direction P |
| BUMP active `Direction_p` | inherited runtime value | `95.0f` | Applied only while BUMP mode is active |
| BUMP target speed | `300.0f` | `314.0f` | Forced when entering the BUMP screen |
| Anti-stall integral accumulation gain | `1.6f` | `9.6f` global, `8.0f` in BUMP | Runtime readable as `anti_stall_integral_gain` |
| Integral-to-PWM gain | `0.32f` | `1.5f` in BUMP | Runtime readable as `anti_stall_pwm_gain` |
| Assist PWM output limit | `4000` | `6000` | Shared `BUMP_ASSIST_PWM_LIMIT` |
| BUMP gain edit maximum | `1.00f` | `2.00f` | BUMP screen gain clamp |
| BUMP reverse assist | none | reverse at `-target_speed` for `0.5s` | Triggered after prior average speed `>15`, then average speed `<3` for `100ms` |

BUMP mode saves the entering `Direction_p`, integral accumulation gain, and integral-to-PWM gain before applying its overrides, then restores them after leaving BUMP mode.
