function attitude = desired_attitude(z1_d_x, z2_d_y, param)
% Nominal attitude for comparison with the simulated plant.
g = param.g;
d2z1 = z1_d_x(3); d3z1 = z1_d_x(4);
d2z2 = z2_d_y(3); d3z2 = z2_d_y(4);
den = d2z1^2 + (d2z2 + g)^2;
assert(den > 0.05^2);
theta_d = atan2(-d2z1, d2z2 + g);
omega_d = (d2z1*d3z2 - (d2z2 + g)*d3z1)/den;
attitude = [theta_d; omega_d];
end
