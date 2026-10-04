function [u1_ff, u2_ff] = feedforward_control(z1_d_x, z2_d_y, param)
% Flatness-based feedforward control for the normalized planar VTOL.

% Parameters: g = 1 for the exercise's normalized plant.
g = param.g;

% Unpack z1* derivatives: [z1; dz1; d2z1; d3z1; d4z1].
z1   = z1_d_x(1);
dz1  = z1_d_x(2);
d2z1 = z1_d_x(3);
d3z1 = z1_d_x(4);
d4z1 = z1_d_x(5);

% Unpack z2* derivatives: [z2; dz2; d2z2; d3z2; d4z2].
z2   = z2_d_y(1);
dz2  = z2_d_y(2);
d2z2 = z2_d_y(3);
d3z2 = z2_d_y(4);
d4z2 = z2_d_y(5);

% Position and velocity are unpacked for clarity; these inputs depend on
% acceleration, jerk and snap. A near-zero thrust makes inversion singular.
den = d2z1^2 + (d2z2 + g)^2;
assert(den > 0.05^2);

% VTOL thrust magnitude.
u1_ff = sqrt(den);

% VTOL angular acceleration: second time derivative of desired attitude.
u2_ff = 2*(d2z1*d3z1 + (d2z2 + g)*d3z2) ...
    *(d3z1*(d2z2 + g) - d2z1*d3z2)/den^2 ...
    - (d4z1*(d2z2 + g) - d2z1*d4z2)/den;
end
