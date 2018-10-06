%% Dubins car dynamics
function dx = dyn(x,u)
    dx = [u(1)*cos(x(3)); u(1)*sin(x(3)); u(2)];
end