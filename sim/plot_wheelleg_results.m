function plot_wheelleg_results(log, p)
%PLOT_WHEELLEG_RESULTS Standard plots for the simplified simulation.

t = log.t;

figure('Name','Wheel-Leg Longitudinal Simulation','Color','w');
tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, log.roll_raw_deg, 'b', 'LineWidth', 1.2); hold on;
plot(t, log.roll_ctrl_deg, 'r--', 'LineWidth', 1.0);
yline(p.ctrl.target_motor_stand, 'k:');
grid on;
xlabel('t (s)');
ylabel('deg');
title('Roll');
legend('raw roll','controller roll','target','Location','best');

nexttile;
plot(t, log.gyro_raw_deg_s, 'm', 'LineWidth', 1.2); hold on;
plot(t, log.gyro_ctrl_deg_s, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('t (s)');
ylabel('deg/s');
title('Gyro X');
legend('raw gyro','controller gyro','Location','best');

nexttile;
plot(t, log.spd_out, 'LineWidth', 1.2); hold on;
plot(t, log.ang_out, 'LineWidth', 1.2);
plot(t, log.gyr_out, 'LineWidth', 1.2);
grid on;
xlabel('t (s)');
ylabel('controller units');
title('Loop Outputs');
legend('SpdO','AngO','GyrO','Location','best');

nexttile;
plot(t, log.motor_cmd, 'LineWidth', 1.2);
grid on;
xlabel('t (s)');
ylabel('PWM-like');
title('Motor Command');

nexttile;
plot(t, log.cart_vel, 'LineWidth', 1.2); hold on;
plot(t, log.encoder_left, '--', 'LineWidth', 1.0);
grid on;
xlabel('t (s)');
ylabel('m/s / encoder-like');
title('Velocity');
legend('cart vel','encoder left','Location','best');

nexttile;
plot(t, rad2deg(log.theta), 'LineWidth', 1.2); hold on;
plot(t, log.cart_pos, 'LineWidth', 1.2);
plot(t, log.com_height, '--', 'LineWidth', 1.0);
grid on;
xlabel('t (s)');
ylabel('deg / m');
title('Body Angle, Position, COM Height');
legend(["theta","position","com height"], 'Location', 'best');
end
