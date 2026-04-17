function [ctrl, y] = wheelleg_controller_step(ctrl, meas, p)
%WHEELLEG_CONTROLLER_STEP Cascaded controller mapped from current firmware.
%
% meas.roll_deg        raw roll angle from simulated IMU
% meas.gyro_x_deg_s    raw gyro x from simulated IMU
% meas.encoder_left    encoder-like speed value
% meas.encoder_right   encoder-like speed value

% Apply balance zero offset exactly as firmware does.
roll = meas.roll_deg - p.ctrl.balance_zero_offset_deg;
raw_gyro_x = meas.gyro_x_deg_s - p.sensor.gyro_bias_deg_s;

% Gyro loop always runs at 1 ms.
ctrl.t_gyro = ctrl.t_gyro + p.dt_sim;
ctrl.t_angle = ctrl.t_angle + p.dt_sim;
ctrl.t_speed = ctrl.t_speed + p.dt_sim;

if ctrl.t_angle >= p.ctrl.Ts_angle
    ctrl.t_angle = ctrl.t_angle - p.ctrl.Ts_angle;

    if ctrl.t_speed >= p.ctrl.Ts_speed
        ctrl.t_speed = ctrl.t_speed - p.ctrl.Ts_speed;

        encoder_left = meas.encoder_left;
        encoder_right = meas.encoder_right;

        if ctrl.leg_error > 0
            [ctrl.velocity_left, ctrl.velocity_angle_left] = velocity_loop(ctrl.velocity_left, encoder_left, ...
                p.ctrl.target_velocity - ctrl.v_buchang, p);
            [ctrl.velocity_right, ctrl.velocity_angle_right] = velocity_loop(ctrl.velocity_right, -encoder_right, ...
                p.ctrl.target_velocity + ctrl.v_buchang, p);
        else
            [ctrl.velocity_left, ctrl.velocity_angle_left] = velocity_loop(ctrl.velocity_left, encoder_left, ...
                p.ctrl.target_velocity + ctrl.v_buchang, p);
            [ctrl.velocity_right, ctrl.velocity_angle_right] = velocity_loop(ctrl.velocity_right, -encoder_right, ...
                p.ctrl.target_velocity - ctrl.v_buchang, p);
        end
    end

    [ctrl, ctrl.balance_pwm_left] = balance_loop(ctrl, roll, raw_gyro_x, ctrl.velocity_angle_left, p);
    [~, ctrl.balance_pwm_right] = balance_loop(ctrl, roll, raw_gyro_x, ctrl.velocity_angle_right, p);
end

[ctrl.gyro_left, ctrl.gyro_pwm_left] = gyro_loop(ctrl.gyro_left, -ctrl.balance_pwm_left, raw_gyro_x, p);
[ctrl.gyro_right, ctrl.gyro_pwm_right] = gyro_loop(ctrl.gyro_right, -ctrl.balance_pwm_right, raw_gyro_x, p);

motor_left = clamp(ctrl.gyro_pwm_left, -p.ctrl.motor_output_limit, p.ctrl.motor_output_limit);
motor_right = clamp(ctrl.gyro_pwm_right, -p.ctrl.motor_output_limit, p.ctrl.motor_output_limit);

% Longitudinal equivalent torque/force uses the average command.
y.motor_left = motor_left;
y.motor_right = motor_right;
y.motor_cmd = 0.5 * (motor_left + motor_right);
y.raw_gyro_x = raw_gyro_x;
y.roll_ctrl_deg = roll;
y.spd_out = 0.5 * (ctrl.velocity_angle_left + ctrl.velocity_angle_right);
y.ang_out = 0.5 * (ctrl.balance_pwm_left + ctrl.balance_pwm_right);
y.gyr_out = 0.5 * (ctrl.gyro_pwm_left + ctrl.gyro_pwm_right);

end

function [state, out] = velocity_loop(state, encoder_left, target_velocity, p)
encoder_bias = target_velocity - encoder_left;
state.integral = clamp(state.integral + encoder_bias, ...
    -p.ctrl.speed_integral_limit, p.ctrl.speed_integral_limit);
state.output = p.ctrl.Speed_p * encoder_bias + p.ctrl.Speed_i * state.integral;
state.output = clamp(state.output, -p.ctrl.speed_output_limit, p.ctrl.speed_output_limit);
out = -state.output; % current firmware sign
end

function [ctrl, out] = balance_loop(ctrl, angle_deg, gyro_deg_s, target, p)
angle_bias = p.ctrl.target_motor_stand + target - angle_deg;
gyro_bias = -gyro_deg_s;
ctrl.angle_integral = clamp(ctrl.angle_integral + angle_bias, ...
    -p.ctrl.angle_integral_limit, p.ctrl.angle_integral_limit);
out = -p.ctrl.Angle_p * angle_bias ...
      -p.ctrl.Angle_i * ctrl.angle_integral ...
      +p.ctrl.Angle_d * gyro_bias;
out = clamp(out, -p.ctrl.angle_output_limit, p.ctrl.angle_output_limit);
end

function [state, out] = gyro_loop(state, target_gyro, current_gyro, p)
gyro_error = target_gyro - current_gyro;
state.integral = clamp(state.integral + gyro_error, ...
    -p.ctrl.gyro_integral_limit, p.ctrl.gyro_integral_limit);
gyro_delta = gyro_error - state.last_error;
gyro_control = p.ctrl.Gyro_p * gyro_error ...
             + p.ctrl.Gyro_i * state.integral ...
             + p.ctrl.Gyro_d * gyro_delta;
state.last_error = gyro_error;
out = -gyro_control;
end

function v = clamp(v, lo, hi)
v = min(max(v, lo), hi);
end
