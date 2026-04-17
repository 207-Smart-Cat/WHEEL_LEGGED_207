clear;
clc;

p = init_wheelleg_sim_params();

N = round(p.T_end / p.dt_sim) + 1;
t = (0:N-1)' * p.dt_sim;

% State: [x; x_dot; theta; theta_dot]
x = zeros(4, N);
x(:,1) = [
    p.scenario.x0;
    p.scenario.xdot0;
    deg2rad(p.scenario.theta0_deg);
    0.0
];

ctrl.t_gyro = 0.0;
ctrl.t_angle = 0.0;
ctrl.t_speed = 0.0;
ctrl.angle_integral = 0.0;
ctrl.leg_error = 0.0;
ctrl.v_buchang = 0.0;
ctrl.balance_pwm_left = 0.0;
ctrl.balance_pwm_right = 0.0;
ctrl.velocity_angle_left = 0.0;
ctrl.velocity_angle_right = 0.0;
ctrl.velocity_left.integral = 0.0;
ctrl.velocity_left.output = 0.0;
ctrl.velocity_right.integral = 0.0;
ctrl.velocity_right.output = 0.0;
ctrl.gyro_left.integral = 0.0;
ctrl.gyro_left.last_error = 0.0;
ctrl.gyro_right.integral = 0.0;
ctrl.gyro_right.last_error = 0.0;

log.roll_raw_deg = zeros(N,1);
log.roll_ctrl_deg = zeros(N,1);
log.gyro_raw_deg_s = zeros(N,1);
log.gyro_ctrl_deg_s = zeros(N,1);
log.spd_out = zeros(N,1);
log.ang_out = zeros(N,1);
log.gyr_out = zeros(N,1);
log.motor_left = zeros(N,1);
log.motor_right = zeros(N,1);
log.motor_cmd = zeros(N,1);
log.cart_pos = zeros(N,1);
log.cart_vel = zeros(N,1);
log.theta = zeros(N,1);
log.encoder_left = zeros(N,1);
log.com_height = zeros(N,1);
log.leg_phi1_deg = zeros(N,1);
log.leg_phi4_deg = zeros(N,1);
log.t = t;

imu_roll_deg = p.ctrl.balance_zero_offset_deg + rad2deg(x(3,1));
imu_gyro_deg_s = p.sensor.gyro_bias_deg_s + rad2deg(x(4,1));

for k = 1:N
    tk = t(k);

    if mod(k-1, round(p.ctrl.Ts_imu / p.dt_sim)) == 0
        imu_roll_deg = p.ctrl.balance_zero_offset_deg ...
            + rad2deg(x(3,k)) ...
            + p.sensor.angle_noise_std * randn();
        imu_gyro_deg_s = p.sensor.gyro_bias_deg_s ...
            + rad2deg(x(4,k)) ...
            + p.sensor.gyro_noise_std * randn();
    end

    encoder_left = p.sensor.encoder_scale * x(2,k);
    encoder_right = -p.sensor.encoder_scale * x(2,k);

    meas.roll_deg = imu_roll_deg;
    meas.gyro_x_deg_s = imu_gyro_deg_s;
    meas.encoder_left = encoder_left;
    meas.encoder_right = encoder_right;

    [ctrl, y] = wheelleg_controller_step(ctrl, meas, p);
    leg = wheelleg_leg_kinematics(p.leg.x_leg_target, p.leg.y_leg_target, p);

    ext_force = 0.0;
    if tk >= p.scenario.disturbance_time && tk < p.scenario.disturbance_time + p.scenario.disturbance_duration
        ext_force = ext_force + p.scenario.disturbance_force;
    end
    if isfield(p.scenario, 'disturbance2_time') ...
            && tk >= p.scenario.disturbance2_time ...
            && tk < p.scenario.disturbance2_time + p.scenario.disturbance2_duration
        ext_force = ext_force + p.scenario.disturbance2_force;
    end

    if k < N
        x(:,k+1) = wheelleg_plant_step(x(:,k), y.motor_cmd, ext_force, p);
    end

    log.roll_raw_deg(k) = imu_roll_deg;
    log.roll_ctrl_deg(k) = y.roll_ctrl_deg;
    log.gyro_raw_deg_s(k) = imu_gyro_deg_s;
    log.gyro_ctrl_deg_s(k) = y.raw_gyro_x;
    log.spd_out(k) = y.spd_out;
    log.ang_out(k) = y.ang_out;
    log.gyr_out(k) = y.gyr_out;
    log.motor_left(k) = y.motor_left;
    log.motor_right(k) = y.motor_right;
    log.motor_cmd(k) = y.motor_cmd;
    log.cart_pos(k) = x(1,k);
    log.cart_vel(k) = x(2,k);
    log.theta(k) = x(3,k);
    log.encoder_left(k) = encoder_left;
    log.com_height(k) = leg.com_height;
    log.leg_phi1_deg(k) = leg.phi1_deg;
    log.leg_phi4_deg(k) = leg.phi4_deg;
end

plot_wheelleg_results(log, p);
animate_wheelleg_sim(log, p);

fprintf('Simulation complete.\\n');
fprintf('Peak |theta| = %.2f deg\\n', max(abs(rad2deg(log.theta))));
fprintf('Peak |motor cmd| = %.1f\\n', max(abs(log.motor_cmd)));
