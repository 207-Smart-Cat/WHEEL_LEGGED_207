function p = init_wheelleg_sim_params()
%INIT_WHEELLEG_SIM_PARAMS Parameters for the simplified wheel-leg simulation.

% Timing mapped from current firmware.
p.dt_sim = 1e-3;
p.T_end = 8.0;
p.ctrl.Ts_imu = 5e-3;
p.ctrl.Ts_gyro = 1e-3;
p.ctrl.Ts_angle = 5e-3;
p.ctrl.Ts_speed = 20e-3;

% Current controller defaults / recent tuning baseline.
p.ctrl.target_velocity = 0.0;
p.ctrl.target_motor_stand = 0.0;
p.ctrl.balance_zero_offset_deg = 8.0;

p.ctrl.Speed_p = 0.025;
p.ctrl.Speed_i = 0.0;
p.ctrl.Speed_d = 0.0; %#ok<NASGU> % kept for interface parity

p.ctrl.Angle_p = 8.8;
p.ctrl.Angle_i = 0.10;
p.ctrl.Angle_d = 0.40;

p.ctrl.Gyro_p = 26.0;
p.ctrl.Gyro_i = 0.0;
p.ctrl.Gyro_d = 0.06;

% Controller clamps mirrored from firmware.
p.ctrl.speed_integral_limit = 2000;
p.ctrl.speed_output_limit = 6;
p.ctrl.angle_integral_limit = 1000;
p.ctrl.angle_output_limit = 5000;
p.ctrl.gyro_integral_limit = 1500;
p.ctrl.motor_output_limit = 8000; % MAX_DUTY * PWM_DUTY_MAX / 100 = 80 * 100

% Sensor conversion.
% Encoder scale is artificial: convert wheel linear speed [m/s] to encoder-like value.
p.sensor.encoder_scale = 1200;
p.sensor.gyro_bias_deg_s = 0.49;  % current firmware compensation value
p.sensor.gyro_noise_std = 0.12;
p.sensor.angle_noise_std = 0.05;

% Simplified plant.
p.plant.mass_body = 1.9;    % kg
p.plant.mass_wheel = 0.65;  % kg equivalent cart mass
p.plant.com_height = 0.12;  % nominal m, updated from leg geometry during simulation
p.plant.g = 9.81;
p.plant.wheel_damping = 1.2;
p.plant.body_damping = 0.03;
p.plant.motor_force_gain_forward = 0.0020; % N per PWM command
p.plant.motor_force_gain_backward = 0.0016; % N per PWM command
p.plant.motor_deadzone = 220; % PWM-like command below this produces no drive
p.plant.coulomb_friction = 0.55; % N
p.plant.viscous_friction = 0.28; % N/(m/s)

% Mechanical geometry from product page and code.
p.geometry.wheel_radius = 0.034;      % 68 mm wheel diameter
p.geometry.wheel_track = 0.156;       % 156 mm wheel center distance
p.geometry.body_size = [0.177, 0.172, 0.192]; % m

% Five-bar leg geometry from firmware.
p.leg.L1 = 0.06;
p.leg.L2 = 0.09;
p.leg.L3 = 0.09;
p.leg.L4 = 0.06;
p.leg.L5 = 0.038;
p.leg.x_min = -0.05;
p.leg.x_max = 0.05;
p.leg.y_min = 0.025;
p.leg.y_max = 0.14;

% Current leg operating point from firmware tuning.
p.leg.x_leg_target = 0.0;
p.leg.y_leg_target = 0.05;

% Servo constraints from product page.
p.leg.servo_speed_rad_s = deg2rad(60) / 0.13; % 7.4 V speed
p.leg.servo_torque_Nm = 10 * 0.0980665;       % 10 kg*cm
p.leg.servo_k = 40.0;                         % virtual servo stiffness for target tracking

% Mapping from leg y to equivalent COM height.
% This is a surrogate, not a full rigid-body leg model.
p.leg.body_mount_offset = 0.055;
p.leg.com_height_gain = 0.85;

% Scenario: make balancing behavior visually obvious.
p.scenario.theta0_deg = 8.0;
p.scenario.x0 = 0.0;
p.scenario.xdot0 = 0.0;
p.scenario.disturbance_time = 1.5;
p.scenario.disturbance_duration = 0.12;
p.scenario.disturbance_force = 10.0; % N
p.scenario.disturbance2_time = 4.0;
p.scenario.disturbance2_duration = 0.12;
p.scenario.disturbance2_force = -9.0; % N
