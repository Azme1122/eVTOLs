function reference = trajectory(time, kind)
% Columns contain position, velocity, acceleration, jerk and snap.
reference = zeros(2,5);
if strcmp(kind,'circle')
    frequency = 0.35;
    for derivative = 0:4
        angle = frequency*time + derivative*pi/2;
        reference(:,derivative+1) = 0.5*frequency^derivative*[cos(angle);sin(angle)];
    end
    reference(:,1) = reference(:,1)+[2;1.5];
else
    duration = 6;
    coefficients = [-20 70 -84 35 0 0 0 0];
    if time >= duration
        reference(:,1) = [3;2];
    elseif time <= 0
        reference(:,1) = [1;1];
    else
        for derivative = 0:4
            reference(:,derivative+1) = [2;1]*polyval(coefficients,time/duration)/duration^derivative;
            coefficients = polyder(coefficients);
        end
        reference(:,1) = reference(:,1)+[1;1];
    end
end
end
