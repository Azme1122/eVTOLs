function [z1_soll, z2_soll] = fcn(t, T, z10, z20, z1T, z2T)
if T > 0
    tau = min(max(t/T,0),1);
    % Polynomial (C^4 smooth)
    P  = 126*tau^5 - 420*tau^6 + 540*tau^7 - 315*tau^8 + 70*tau^9;
    dP = (630*tau^4 - 2520*tau^5 + 3780*tau^6 - 2520*tau^7 + 630*tau^8) / T;
    d2P = (2520*tau^3 - 12600*tau^4 + 22680*tau^5 - 17640*tau^6 + 5040*tau^7) / T^2;
    d3P = (7560*tau^2 - 50400*tau^3 + 113400*tau^4 - 105840*tau^5 + 35280*tau^6) / T^3;
    d4P = (15120*tau - 151200*tau^2 + 453600*tau^3 - 529200*tau^4 + 211680*tau^5) / T^4;
else
    % Hold initial state if T <= 0.
    P = 0; dP = 0; d2P = 0; d3P = 0; d4P = 0;
end
% z1 trajectory
z1 = z10 + (z1T - z10) * P;
dz1 = (z1T - z10) * dP;
d2z1 = (z1T - z10) * d2P;
d3z1 = (z1T - z10) * d3P;
d4z1 = (z1T - z10) * d4P;
% z2 trajectory
z2 = z20 + (z2T - z20) * P;
dz2 = (z2T - z20) * dP;
d2z2 = (z2T - z20) * d2P;
d3z2 = (z2T - z20) * d3P;
d4z2 = (z2T - z20) * d4P;
z1_soll = [z1; dz1; d2z1; d3z1; d4z1];
z2_soll = [z2; dz2; d2z2; d3z2; d4z2];
end
