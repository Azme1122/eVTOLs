function [input, attitude, thrustRate] = feedforward(reference)
q = reference(:,3)+[0;1];
dq = reference(:,4); ddq = reference(:,5);
thrust = norm(q);
if thrust < 0.05
    error('Reference approaches zero thrust');
end
crossValue = q(1)*dq(2)-q(2)*dq(1);
omega = crossValue/thrust^2;
alpha = (q(1)*ddq(2)-q(2)*ddq(1))/thrust^2 ...
    -2*(q.'*dq)*crossValue/thrust^4;
input = [thrust;alpha];
attitude = [atan2(-q(1),q(2));omega];
thrustRate = q.'*dq/thrust;
end
