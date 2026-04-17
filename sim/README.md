# Wheel-Leg Longitudinal MATLAB Simulation

This is a MATLAB simulation for the current wheel-leg balance controller.

It is **not** the official Seekfree V vehicle model. It is a longitudinal surrogate built from:

- your current firmware controller structure
- your actual five-bar linkage geometry (`L1`~`L5`)
- the wheel size and track from the product page
- servo speed / torque limits from the product page

It is used to:

- reproduce the current cascaded loop timing
- reproduce the current controller sign conventions
- test rough tuning trends before burning code

## Files

- `run_wheelleg_longitudinal_sim.m`
  Main entry script.
- `init_wheelleg_sim_params.m`
  Simulation parameters and current controller defaults.
- `wheelleg_controller_step.m`
  Controller logic mapped from `project/code/control.c`.
- `wheelleg_plant_step.m`
  Simplified wheel-leg longitudinal plant.
- `wheelleg_leg_kinematics.m`
  Five-bar inverse kinematics and leg geometry helpers.
- `plot_wheelleg_results.m`
  Standard plotting for inspection.

## What is modeled

- Longitudinal body angle `theta`
- Body angular rate `theta_dot`
- Wheel position `x`
- Wheel velocity `x_dot`
- Five-bar leg geometry using `L1`~`L5`
- Leg target `(x, y)` and servo angle solution
- Servo rate limit and torque limit
- Equivalent COM height scheduled by leg geometry

The plant is a nonlinear cart-pole style surrogate with damping and a motor-force gain.
The COM height is updated from the current leg state, so changing `x_leg_target` / `y_leg_target`
changes the plant geometry seen by the controller.

## What is not modeled yet

- Five-bar leg geometry
- Lateral / yaw dynamics
- Steering loop
- Navigation states
- Real motor dead-zone, backlash, friction asymmetry
- Full IMU fusion
- Dual-leg full rigid-body multibody dynamics

## How to run

In MATLAB:

```matlab
cd('D:\learnplace\ZZZ_LUNTUI\sim');
run_wheelleg_longitudinal_sim
```

## What to tune first

If the simulation diverges too quickly or is too weak, do not change controller signs first.
Tune these plant-side parameters in `init_wheelleg_sim_params.m`:

- `p.plant.mass_body`
- `p.plant.mass_wheel`
- `p.plant.com_height`
- `p.leg.body_mount_offset`
- `p.plant.motor_force_gain`
- `p.plant.wheel_damping`
- `p.plant.body_damping`
- `p.sensor.encoder_scale`

These determine how close the surrogate is to your real car.

For leg geometry, start from:

- `p.leg.x_leg_target`
- `p.leg.y_leg_target`
- `p.leg.body_mount_offset`
- `p.leg.com_height_gain`

## Controller mapping

The simulation follows the current firmware structure:

- IMU update: `5 ms`
- Gyro loop: `1 ms`
- Angle loop: `5 ms`
- Speed loop: `20 ms`

It also keeps the current firmware sign conventions:

- speed loop returns `-velocity_output`
- angle loop uses `-kp * angle_error - ki * integral + kd * gyro_bias`
- gyro loop returns `-gyro_output`

## Practical use

Use this simulation to answer:

- does changing `Angle_i` reduce one-sided running?
- when does `Gyro_p` start to create high-frequency oscillation?
- how much `Speed_p` starts to dominate the target angle?

Do not expect the absolute angle or disturbance boundary to match the real car until plant parameters are identified.
