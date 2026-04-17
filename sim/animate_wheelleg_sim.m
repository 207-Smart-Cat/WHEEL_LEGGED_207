function animate_wheelleg_sim(log, p)
%ANIMATE_WHEELLEG_SIM 2D side-view animation with wheel, body, and five-bar leg.

dt = log.t(2) - log.t(1);
step = max(1, round(0.01 / dt));
wheel_r = p.geometry.wheel_radius;
body_h = p.geometry.body_size(3);
body_w = p.geometry.body_size(1) * 0.55;
body_vis_offset = 0.035;
theta_c = linspace(0, 2 * pi, 100);
view_half_width = 0.22;
trail_len = max(10, round(0.6 / dt));

fig = figure('Name', 'Wheel-Leg 2D Animation', 'Color', 'w');
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, 'Wheel-Leg Robot 2D Side View');
plot(ax, [-2, 2], [0, 0], 'k-', 'LineWidth', 1.0);
ground_marks = gobjects(13, 1);
for i = 1:numel(ground_marks)
    ground_marks(i) = plot(ax, [0, 0], [-0.004, 0.004], 'Color', [0.65 0.65 0.65], 'LineWidth', 1.0);
end

wheel = plot(ax, wheel_r * cos(theta_c), wheel_r * sin(theta_c), 'b-', 'LineWidth', 2.0);
wheel_spoke_a = plot(ax, [0, 0], [0, 0], 'Color', [0.15 0.35 0.85], 'LineWidth', 1.2);
wheel_spoke_b = plot(ax, [0, 0], [0, 0], 'Color', [0.15 0.35 0.85], 'LineWidth', 1.2);
body_outline = patch(ax, 'XData', [], 'YData', [], ...
    'FaceColor', [0.92 0.24 0.20], 'FaceAlpha', 0.15, ...
    'EdgeColor', [0.80 0.10 0.08], 'LineWidth', 2.0);
body_centerline = plot(ax, [0, 0], [0, 0], 'Color', [0.80 0.10 0.08], 'LineWidth', 3.0);
axle_point = plot(ax, 0, 0, 'ko', 'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 6);
mount_bar = plot(ax, [0, 0], [0, 0], 'k-', 'LineWidth', 2.0);
left_upper = plot(ax, [0, 0], [0, 0], 'Color', [0.10 0.60 0.20], 'LineWidth', 2.0);
left_lower = plot(ax, [0, 0], [0, 0], 'Color', [0.10 0.60 0.20], 'LineWidth', 2.0);
right_upper = plot(ax, [0, 0], [0, 0], 'Color', [0.20 0.70 0.85], 'LineWidth', 2.0);
right_lower = plot(ax, [0, 0], [0, 0], 'Color', [0.20 0.70 0.85], 'LineWidth', 2.0);
left_elbow = plot(ax, 0, 0, 'o', 'Color', [0.10 0.60 0.20], 'MarkerFaceColor', [0.10 0.60 0.20], 'MarkerSize', 5);
right_elbow = plot(ax, 0, 0, 'o', 'Color', [0.20 0.70 0.85], 'MarkerFaceColor', [0.20 0.70 0.85], 'MarkerSize', 5);
foot_point = plot(ax, 0, 0, 'ks', 'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerSize', 7);
com_point = plot(ax, 0, 0, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
com_trail = plot(ax, nan, nan, '-', 'Color', [0.95 0.75 0.10], 'LineWidth', 1.2);
axle_trail = plot(ax, nan, nan, '-', 'Color', [0.40 0.40 0.40], 'LineWidth', 1.0);
txt = text(ax, -0.34, 0.30, '', 'FontName', 'Consolas', 'FontSize', 10, ...
    'VerticalAlignment', 'top');

xlim(ax, [-view_half_width, view_half_width]);
ylim(ax, [-0.03, 0.34]);

for k = 1:step:numel(log.t)
    if ~isvalid(fig)
        return;
    end

    wheel_center = [log.cart_pos(k), wheel_r];
    theta = log.theta(k);
    body_axis = [sin(theta), cos(theta)];
    body_lat = [cos(theta), -sin(theta)];

    wheel_phase = wheel_center(1) / wheel_r;
    spoke_a = [cos(wheel_phase), sin(wheel_phase)];
    spoke_b = [cos(wheel_phase + pi / 2), sin(wheel_phase + pi / 2)];

    set(wheel, ...
        'XData', wheel_center(1) + wheel_r * cos(theta_c), ...
        'YData', wheel_center(2) + wheel_r * sin(theta_c));
    set(wheel_spoke_a, ...
        'XData', [wheel_center(1), wheel_center(1) + wheel_r * spoke_a(1)], ...
        'YData', [wheel_center(2), wheel_center(2) + wheel_r * spoke_a(2)]);
    set(wheel_spoke_b, ...
        'XData', [wheel_center(1), wheel_center(1) + wheel_r * spoke_b(1)], ...
        'YData', [wheel_center(2), wheel_center(2) + wheel_r * spoke_b(2)]);
    set(axle_point, 'XData', wheel_center(1), 'YData', wheel_center(2));

    x_center = wheel_center(1);
    xlim(ax, [x_center - view_half_width, x_center + view_half_width]);
    ground_x = linspace(x_center - view_half_width, x_center + view_half_width, numel(ground_marks));
    for i = 1:numel(ground_marks)
        set(ground_marks(i), 'XData', [ground_x(i), ground_x(i)], 'YData', [-0.004, 0.004]);
    end

    foot_world = wheel_center;
    mount_center = foot_world - rotate_local([p.leg.x_leg_target, -p.leg.y_leg_target], theta);
    left_mount = mount_center + rotate_local([-p.leg.L5 / 2, 0], theta);
    right_mount = mount_center + rotate_local([p.leg.L5 / 2, 0], theta);

    phi1 = deg2rad(log.leg_phi1_deg(k));
    phi4 = deg2rad(log.leg_phi4_deg(k));

    left_elbow_world = left_mount + rotate_local(p.leg.L1 * [cos(phi1), sin(phi1)], theta);
    right_elbow_world = right_mount + rotate_local(p.leg.L4 * [cos(phi4), sin(phi4)], theta);

    set(mount_bar, ...
        'XData', [left_mount(1), right_mount(1)], ...
        'YData', [left_mount(2), right_mount(2)]);

    set(left_upper, ...
        'XData', [left_mount(1), left_elbow_world(1)], ...
        'YData', [left_mount(2), left_elbow_world(2)]);
    set(left_lower, ...
        'XData', [left_elbow_world(1), foot_world(1)], ...
        'YData', [left_elbow_world(2), foot_world(2)]);
    set(right_upper, ...
        'XData', [right_mount(1), right_elbow_world(1)], ...
        'YData', [right_mount(2), right_elbow_world(2)]);
    set(right_lower, ...
        'XData', [right_elbow_world(1), foot_world(1)], ...
        'YData', [right_elbow_world(2), foot_world(2)]);

    set(left_elbow, 'XData', left_elbow_world(1), 'YData', left_elbow_world(2));
    set(right_elbow, 'XData', right_elbow_world(1), 'YData', right_elbow_world(2));
    set(foot_point, 'XData', foot_world(1), 'YData', foot_world(2));

    body_center = mount_center + body_axis * (body_h * 0.50 + body_vis_offset);
    body_half_h = body_h * 0.48;
    body_half_w = body_w * 0.50;
    body_corners_local = [
        -body_half_w, -body_half_h;
         body_half_w, -body_half_h;
         body_half_w,  body_half_h;
        -body_half_w,  body_half_h
    ];
    body_corners_world = zeros(size(body_corners_local));
    for i = 1:4
        body_corners_world(i,:) = body_center ...
            + body_lat * body_corners_local(i,1) ...
            + body_axis * body_corners_local(i,2);
    end
    set(body_outline, 'XData', body_corners_world(:,1), 'YData', body_corners_world(:,2));
    set(body_centerline, ...
        'XData', [body_center(1) - body_axis(1) * body_half_h, body_center(1) + body_axis(1) * body_half_h], ...
        'YData', [body_center(2) - body_axis(2) * body_half_h, body_center(2) + body_axis(2) * body_half_h]);

    com_world = wheel_center + body_axis * log.com_height(k);
    set(com_point, 'XData', com_world(1), 'YData', com_world(2));

    trail_start = max(1, k - trail_len);
    set(com_trail, ...
        'XData', log.cart_pos(trail_start:step:k) + sin(log.theta(trail_start:step:k)) .* log.com_height(trail_start:step:k), ...
        'YData', wheel_r + cos(log.theta(trail_start:step:k)) .* log.com_height(trail_start:step:k));
    set(axle_trail, ...
        'XData', log.cart_pos(trail_start:step:k), ...
        'YData', wheel_r * ones(size(log.cart_pos(trail_start:step:k))));

    txt.String = sprintf([ ...
        't = %.2f s\n' ...
        'raw roll = %.2f deg\n' ...
        'ctrl roll = %.2f deg\n' ...
        'motor = %.0f\n' ...
        'leg phi1/phi4 = %.1f / %.1f deg\n' ...
        'com_h = %.3f m'], ...
        log.t(k), log.roll_raw_deg(k), log.roll_ctrl_deg(k), log.motor_cmd(k), ...
        log.leg_phi1_deg(k), log.leg_phi4_deg(k), log.com_height(k));

    drawnow limitrate;
end
end

function v_world = rotate_local(v_local, theta)
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
v_world = (R * v_local(:)).';
end
