function command = controller(state, reference)
%#codegen
% State: [x vx y vy theta omega thrust thrust_rate].
theta = state(5); omega = state(6);
thrust = state(7); rate = state(8);
if thrust < 0.05
    error('Thrust inversion outside valid domain');
end
n = [-sin(theta); cos(theta)];
tangent = [-cos(theta); -sin(theta)];
position = [state(1); state(3)];
velocity = [state(2); state(4)];
acceleration = thrust*n - [0;1];
jerk = rate*n + thrust*omega*tangent;
v = reference(:,5) - 16*(position-reference(:,1)) ...
    -32*(velocity-reference(:,2)) -24*(acceleration-reference(:,3)) ...
    -8*(jerk-reference(:,4));
thrustAcceleration = n.'*v + thrust*omega^2;
torque = (tangent.'*v - 2*rate*omega)/thrust;
command = [thrust; torque; thrustAcceleration];
end
