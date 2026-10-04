function derivative = plant(state, input)
derivative = [state(2); -input(1)*sin(state(5)); ...
    state(4); input(1)*cos(state(5))-1; state(6); input(2)];
end
