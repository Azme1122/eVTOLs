function result = simulate(kind, disturbed, control)
dt = 0.005; time = (0:dt:16).';
reference = trajectory(0,kind);
[input,attitude,rate] = feedforward(reference);
state = [reference(1,1);reference(1,2);reference(2,1);reference(2,2);attitude;input(1);rate];
if disturbed
    state(1) = state(1)+0.3;
    state(3) = state(3)-0.2;
    state(5) = state(5)+0.1;
end
states = zeros(numel(time),8); commands = zeros(numel(time),2);
errors = zeros(numel(time),1);
for index = 1:numel(time)
    reference = trajectory(time(index),kind);
    command = control(state,reference);
    states(index,:) = state.'; commands(index,:) = command(1:2).';
    errors(index) = norm(state([1 3])-reference(:,1));
    if index < numel(time)
        k1 = rhs(time(index),state);
        k2 = rhs(time(index)+dt/2,state+dt*k1/2);
        k3 = rhs(time(index)+dt/2,state+dt*k2/2);
        k4 = rhs(time(index)+dt,state+dt*k3);
        state = state+dt*(k1+2*k2+2*k3+k4)/6;
        assert(all(isfinite(state)) && state(7)>0.05,'Invalid simulation state');
    end
end
result = struct('time',time,'states',states,'commands',commands,'error',errors);
    function derivative = rhs(t,s)
        c = control(s,trajectory(t,kind));
        derivative = [plant(s(1:6),c(1:2));s(8);c(3)];
    end
end
