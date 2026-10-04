function [z1_soll, z2_soll] = fcn(t, T, z1_center, z2_center, R)
% T is the time for one revolution; center coordinates are not initial position.
if T > 0
    w = 2*pi/T;
else
    w = 0;
end
z1 = z1_center + R*cos(w*t);
dz1 = -R*w*sin(w*t);
d2z1 = -R*w^2*cos(w*t);
d3z1 = R*w^3*sin(w*t);
d4z1 = R*w^4*cos(w*t);
z2 = z2_center + R*sin(w*t);
dz2 = R*w*cos(w*t);
d2z2 = -R*w^2*sin(w*t);
d3z2 = -R*w^3*cos(w*t);
d4z2 = R*w^4*sin(w*t);
z1_soll = [z1; dz1; d2z1; d3z1; d4z1];
z2_soll = [z2; dz2; d2z2; d3z2; d4z2];
end
