function [input, attitude] = feedforward_block(reference)
% q is the required translational acceleration including normalized gravity.
q = reference(:,3)+[0;1];
dq = reference(:,4); ddq = reference(:,5);
squaredThrust = q.'*q;
assert(squaredThrust > 0.05^2);
thrust = sqrt(squaredThrust);
crossValue = q(1)*dq(2)-q(2)*dq(1);
theta = atan2(-q(1),q(2));
omega = crossValue/squaredThrust;
alpha = (q(1)*ddq(2)-q(2)*ddq(1))/squaredThrust ...
    -2*(q.'*dq)*crossValue/squaredThrust^2;
input = [thrust;alpha];
attitude = [theta;omega];
end
