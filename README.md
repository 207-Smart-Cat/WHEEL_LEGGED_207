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
#define VISION_ALIGN_STABLE_ERROR_PX       (30)
#define VISION_ALIGN_SAMPLE_ERROR_PX       (5)
```

- Stable wait requires 30 consecutive valid vision frames within +/-30 px.
- Sampling records 10 left-side and 10 right-side center samples, within +/-5 px.
- Vision calibration drive speed is `target_velocity = 150.0f`; completion sets `target_velocity = 0.0f`.
