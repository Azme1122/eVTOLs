function reference = reference_block(t, mode)
% Rows: x and y. Columns: position, velocity, acceleration, jerk, snap.
reference = zeros(2,5);
if mode == 2
    radius = 0.5; frequency = 0.35;
    for derivative = 0:4
        phase = frequency*t + derivative*pi/2;
        reference(:,derivative+1) = radius*frequency^derivative*[cos(phase);sin(phase)];
    end
    reference(:,1) = reference(:,1)+[2;1.5];
else
    duration = 6;
    tau = min(max(t/duration,0),1);
    % Ninth-degree blend: first four derivatives vanish at both endpoints.
    p = 126*tau^5-420*tau^6+540*tau^7-315*tau^8+70*tau^9;
    dp = (630*tau^4-2520*tau^5+3780*tau^6-2520*tau^7+630*tau^8)/duration;
    ddp = (2520*tau^3-12600*tau^4+22680*tau^5-17640*tau^6+5040*tau^7)/duration^2;
    dddp = (7560*tau^2-50400*tau^3+113400*tau^4-105840*tau^5+35280*tau^6)/duration^3;
    ddddp = (15120*tau-151200*tau^2+453600*tau^3-529200*tau^4+211680*tau^5)/duration^4;
    reference = [2;1]*[p dp ddp dddp ddddp];
    reference(:,1) = reference(:,1)+[1;1];
end
end
