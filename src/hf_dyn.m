%% Dubins car dynamics
function dx = hf_dyn(x,u)
    dx = [u(1)*cos(x(3)); u(1)*sin(x(3)); u(2)];
end