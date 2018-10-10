%% Simulates MPC planning

simT = 45;
hf_planT = 7;
lf_planT = 20;
dt = 0.2;
x0_lf = [4.0; 0.5];
x0_hf = [4.0; 0.5; 3*pi/4];
%x0 = [4.0; 0.5; pi/2];
%x0 = [4.0; 0.5; 0];

nx = 3;
nu = 2;

hold on
x0 = x0_hf;
for t=0:simT
    clf
    [hf_xopt, hf_uopt] = hf_fmincon_planner(x0, hf_planT, dt);
    %[lf_xopt, lf_uopt] = lf_fmincon_planner(hf_xopt(end-2:end-1), lf_planT, dt);
    %dx = lf_dyn(xopt(1:nx),uopt(1:nu));
    dx = hf_dyn(hf_xopt(1:nx),hf_uopt(1:nu));
    x0 = x0 + dt*dx;
    
    % plot the current state after application of control
    plot(x0(1), x0(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
    axis([0 8 0 10]);
    drawnow   
end