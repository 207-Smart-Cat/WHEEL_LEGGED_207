# IMU_refine Branch Notes

This branch was created from local branch `p1`.

## Purpose

`IMU_refine` is used to debug and refine the IMU gravity compensation path and the IMU + wheel-speed Kalman odometry workflow.

## Changes In This Branch

- Added `Knowledge/Kalman_Knowledge.md` to document the current Kalman state estimator:
  - Kalman inputs and outputs.
  - Prediction and update equations.
  - How odometry uses the Kalman `v` and `w` outputs.
  - Which measurements are trusted more by the current `Q/R` configuration.
  - Known bug/risk checkpoints for IMU gravity compensation and sensor fusion.
- Adjusted gravity compensation signs for the navigation X/Y acceleration axes in `Navi_Remove_Gravity()`:
  - X/Y now use `raw_data.accel + g_comp`.
  - Z keeps `raw_data.accel - g_comp`, because the existing Z-axis correction was already stable in tests.
- Added optional Core0 serial debug output for corrected navigation acceleration:
  - `NAV_ACC_DEBUG_PRINT_MODE` prints `filter_data.accel[0..2]` every about 50 ms.
  - Output is numeric-only CSV: `ax,ay,az`.
  - Wheel-speed debug printing is disabled by `WHEEL_SPEED_DEBUG_PRINT_MODE`.
- Updated screen/debug behavior currently present in the workspace:
  - Page 1 displays heartbeat and navigation validity in the navigation row.
  - Camera assist initialization is moved into the vision UI path on CM7_1.

## Validation Notes

Expected serial acceleration output while the vehicle is static:

```text
filter_data.accel[0] ~= 0 m/s^2
filter_data.accel[1] ~= 0 m/s^2
filter_data.accel[2] ~= 0 m/s^2
```

Small residuals are expected from IMU noise, low-pass delay, and attitude estimation error. Large jumps during combined-axis tilt should be investigated in the gravity projection model and IMU axis mapping before tuning Kalman `Q/R`.
