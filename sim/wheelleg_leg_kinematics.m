function leg = wheelleg_leg_kinematics(x_leg, y_leg, p)
%WHEELLEG_LEG_KINEMATICS Five-bar inverse kinematics mapped from firmware.

x_leg = min(max(x_leg, p.leg.x_min), p.leg.x_max);
y_leg = min(max(y_leg, p.leg.y_min), p.leg.y_max);

[phi1_ok, phi1_a, phi1_b] = solve_phi1_candidates(x_leg, y_leg, p);
[phi4_ok, phi4_a, phi4_b] = solve_phi4_candidates(x_leg, y_leg, p);

leg.valid = phi1_ok && phi4_ok;
if ~leg.valid
    leg.phi1_deg = NaN;
    leg.phi4_deg = NaN;
    leg.com_height = p.plant.com_height;
    return;
end

leg.phi1_deg = select_phi1(phi1_a, phi1_b);
leg.phi4_deg = select_phi4(phi4_a, phi4_b);

% Equivalent COM height scheduling.
leg.com_height = p.leg.body_mount_offset + p.leg.com_height_gain * y_leg;
leg.com_height = max(0.06, min(0.20, leg.com_height));
end

function [ok, phi1_a_deg, phi1_b_deg] = solve_phi1_candidates(x_target, y_target, p)
x_plus = x_target + p.leg.L5 / 2.0;
a = 2.0 * x_plus * p.leg.L1;
b = 2.0 * y_target * p.leg.L1;
c = x_plus^2 + y_target^2 + p.leg.L1^2 - p.leg.L2^2;
den = a^2 + b^2;
radicand = den - c^2;

ok = radicand >= 0.0 && den > 0.0;
if ~ok
    phi1_a_deg = NaN;
    phi1_b_deg = NaN;
    return;
end

psi = atan2(b, a);
alpha = acos(c / sqrt(den));
phi1_a_deg = normalize_360(rad2deg(psi + alpha));
phi1_b_deg = normalize_360(rad2deg(psi - alpha));
end

function [ok, phi4_a_signed_deg, phi4_b_signed_deg] = solve_phi4_candidates(x_target, y_target, p)
x_minus = x_target - p.leg.L5 / 2.0;
a = 2.0 * x_minus * p.leg.L4;
b = 2.0 * y_target * p.leg.L4;
c = x_minus^2 + y_target^2 + p.leg.L4^2 - p.leg.L3^2;
den = a^2 + b^2;
radicand = den - c^2;

ok = radicand >= 0.0 && den > 0.0;
if ~ok
    phi4_a_signed_deg = NaN;
    phi4_b_signed_deg = NaN;
    return;
end

psi = atan2(b, a);
alpha = acos(c / sqrt(den));
phi4_a_signed_deg = normalize_signed(rad2deg(psi - alpha));
phi4_b_signed_deg = normalize_signed(rad2deg(psi + alpha));
end

function out = select_phi1(a, b)
valid_a = (a >= 99.0 && a <= 261.0);
valid_b = (b >= 99.0 && b <= 261.0);
if valid_a
    out = a;
elseif valid_b
    out = b;
else
    out = a;
end
end

function out = select_phi4(a, b)
valid_a = (a >= -81.0 && a <= 81.0);
valid_b = (b >= -81.0 && b <= 81.0);
if valid_a
    out = a;
elseif valid_b
    out = b;
else
    out = a;
end
end

function angle_deg = normalize_360(angle_deg)
while angle_deg >= 360.0
    angle_deg = angle_deg - 360.0;
end
while angle_deg < 0.0
    angle_deg = angle_deg + 360.0;
end
end

function angle_deg = normalize_signed(angle_deg)
angle_deg = normalize_360(angle_deg);
if angle_deg > 180.0
    angle_deg = angle_deg - 360.0;
end
end
