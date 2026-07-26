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
