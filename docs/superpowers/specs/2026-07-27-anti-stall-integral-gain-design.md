# Anti-stall Integral Gain Change

## Scope

Change only the anti-stall assist error accumulation gain from `1.0f` to `1.6f`.

## Behavior

When all existing assist enable and safety conditions pass:

`integral[k] = clamp(integral[k - 1] + 1.6f * speed_error, 0.0f, 50000.0f)`

All existing reset conditions, the `0.04f` integral-to-PWM gain, and the PWM clamp remain unchanged.

## Verification

Confirm that `control.c` defines the gain as `1.6f` and uses it in the integral accumulation expression.
