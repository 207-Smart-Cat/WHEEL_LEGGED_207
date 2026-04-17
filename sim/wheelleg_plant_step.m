function x_next = wheelleg_plant_step(x, motor_cmd, ext_force, p)
%WHEELLEG_PLANT_STEP Simplified nonlinear longitudinal wheel-leg surrogate.
%
% State x = [cart_pos; cart_vel; theta; theta_dot]
% theta > 0 means forward lean.

pos = x(1);
vel = x(2);
theta = x(3);
theta_dot = x(4);

M = p.plant.mass_wheel;
m = p.plant.mass_body;
leg = wheelleg_leg_kinematics(p.leg.x_leg_target, p.leg.y_leg_target, p);
l = leg.com_height;
g = p.plant.g;

bt = p.plant.body_damping;

drive_cmd = apply_deadzone(motor_cmd, p.plant.motor_deadzone);
if drive_cmd >= 0
    F_drive = p.plant.motor_force_gain_forward * drive_cmd;
else
    F_drive = p.plant.motor_force_gain_backward * drive_cmd;
end

if abs(vel) > 1e-4
    F_fric = p.plant.coulomb_friction * sign(vel) + p.plant.viscous_friction * vel;
else
    if abs(F_drive + ext_force) <= p.plant.coulomb_friction
        F_fric = F_drive + ext_force;
    else
        F_fric = p.plant.coulomb_friction * sign(F_drive + ext_force);
    end
end

F = F_drive + ext_force - F_fric;
bv = p.plant.wheel_damping;

s = sin(theta);
c = cos(theta);
den = M + m * s^2;

xdd = (F - bv * vel + m * s * (l * theta_dot^2 + g * c)) / den;
tdd = (-F * c + bv * vel * c - m * l * theta_dot^2 * c * s - (M + m) * g * s - bt * theta_dot) / (l * den);

pos = pos + vel * p.dt_sim;
vel = vel + xdd * p.dt_sim;
theta = theta + theta_dot * p.dt_sim;
theta_dot = theta_dot + tdd * p.dt_sim;

x_next = [pos; vel; theta; theta_dot];
end

function u = apply_deadzone(u, dz)
if abs(u) <= dz
    u = 0.0;
else
    u = sign(u) * (abs(u) - dz);
end
end
