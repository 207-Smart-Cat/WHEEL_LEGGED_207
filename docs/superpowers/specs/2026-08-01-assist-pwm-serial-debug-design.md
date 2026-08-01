# Assist PWM Serial Debug Design

## Goal

Continuously expose the latest anti-stall assist PWM and the two pre-limit logical motor outputs through the existing physical debug UART for both remote-control driving and Course 3 navigation.

## Output Contract

- Emit one sample every 30 ms.
- Emit samples at all times, regardless of whether Anti-Stall Assist is enabled, whether its activation thresholds are met, or whether the vehicle is controlled by the remote or Course 3 navigation.
- Emit exactly three comma-separated values in this order:

  ```text
  assist_pwm,logical_left,logical_right\r\n
  ```

- Do not add labels, spaces, headers, or workflow identifiers.
- Format `assist_pwm` with two fractional digits. Format `logical_left` and `logical_right` as signed decimal integers.
- When the centralized motor-stop path is active, emit:

  ```text
  0.00,0,0\r\n
  ```

## Architecture

Add one private serial-debug sampling helper in `project/code/control.c`. Both `Motor_Output_Apply()` and `Motor_Output_Stop()` call this helper once per 1 ms balance-control cycle:

- `Motor_Output_Apply()` passes the calculated `assist_pwm`, `logical_left`, and `logical_right` values before `cuu()` limiting and motor-sign conversion.
- `Motor_Output_Stop()` passes zero for all three values.

The helper retains a private cycle counter. On every 30th sample, it enqueues exactly one formatted line with `IPC_LOG_Printf()`. Core1 already drains that queue and calls `LOG_Printf()`, which always writes to the physical debug UART. This avoids calling the blocking physical UART `printf()` directly from the 1 ms balance ISR.

The 30 ms schedule is based on the configured 1 ms `PIT_Balance` period. The counter is shared by the apply and stop paths, so transitions among normal driving, safety stop, remote control, and Course 3 do not restart or disrupt the cadence.

## Data Semantics

`logical_left` and `logical_right` mean the existing local values calculated from gyro, assist, and turn PWM:

```c
logical_left = gyro_pwm + assist_pwm + turn_pwm;
logical_right = gyro_pwm + assist_pwm - turn_pwm;
```

When turning is disabled, both values are calculated without `turn_pwm`. These values are intentionally sampled before output limiting and before left/right driver-sign conversion, matching the variables named in the request.

## Safety and Failure Behavior

- Serial output must not bypass the existing IPC log queue.
- No direct UART transmission is added to the balance ISR.
- Queue overflow keeps the existing IPC behavior: the oldest queued log entry may be dropped rather than blocking control.
- Emergency stop, WiFi gating, jump-engine suspension, and every other `Motor_Output_Stop()` caller continue to command zero motor duty and also generate zero-valued debug samples.
- Logging does not depend on the Anti-Stall runtime-module switch.

## Verification

Add a source-level PowerShell regression test that fails unless all of the following are present:

1. A named 30 ms/cycle interval is defined.
2. One shared helper is called by both `Motor_Output_Apply()` and `Motor_Output_Stop()`.
3. The helper uses the existing IPC log path rather than direct `printf()`.
4. The format contains exactly three comma-separated values in the required order and ends with `\r\n`.
5. The normal path passes calculated logical outputs, while the stop path passes zeros.

Run the new test together with the existing Course 3 bump-assist regression test.
