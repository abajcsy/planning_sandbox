%% Point dynamics
function dx = lf_dyn(x,u)
    dx = [u(1); u(2)];
end