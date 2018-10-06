%% Simulates MPC planning

simT = 45;
planT = 7;
dt = 0.2;
%x0 = [4.0; 0.5; 3*pi/4];
%x0 = [4.0; 0.5; pi/2];
x0 = [4.0; 0.5; 0];

nx = 3;
nu = 2;

clf
hold on
for t=0:simT
    [xopt, uopt] = hf_planner(x0, planT, dt);
    dx = dyn(xopt(1:nx),uopt(1:nu));
    x0 = x0 + dt*dx;
    
    % plot the current state after application of control
    plot(x0(1), x0(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
    axis([0 8 0 10]);
    drawnow   
end