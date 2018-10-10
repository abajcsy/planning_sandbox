%% Simulates MPC planning

% Time parameters
simT = 50;
hfPlanT = 7;
lfPlanT = 20;
dt = 0.2;

% Initial conditions
hfx0 = [4.0; 0.5; pi/2]; %3*pi/4];
%lfx0 = [4.0; 0.5];

% State dimensions
nx = 3;
nu = 2;

% World dimensions
lfWidth = 8; %16;
lfHeight = 10; %20;

% Goal location
hfgoal = [4; 9; pi/2]; 
lfgoal = [4; 9]; %[8; 18];

% Compute the optimal value for every state "offline"
optV = lf_global_planner_vi(lfgoal, lfWidth, lfHeight);

% Set the initial condition for mpc
x0 = hfx0;
for t=0:simT
    clf
    hold on
    % plot value function
    clims = [-10.0,0.0];
    imagesc(optV, clims);
    colorbar
    
    [hf_xopt, hf_uopt] = hf_fmincon_planner(x0, hfPlanT, optV, dt, lfWidth, lfHeight);
    %[lf_xopt, lf_uopt] = lf_fmincon_planner(hf_xopt(end-2:end-1), lfPlanT, dt);
    %dx = lf_dyn(xopt(1:nx),uopt(1:nu));
    dx = hf_dyn(hf_xopt(1:nx),hf_uopt(1:nu));
    x0 = x0 + dt*dx;
        
    % plot the current state after application of control
    %[r,c] = hfToLfState(x0, lfWidth, lfHeight);
    %plot(c, r, 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
    plot(x0(1), x0(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
    axis([0 8 0 10]);
    drawnow   
end

function [r,c] = hfToLfState(hfstate, lfW, lfH)
    % TODO: THESE ARE HARD CODED.
    hfW = 8.0;
    hfH = 10.0;
    
    % Compute resultion (real meters)/(sim dim)
    resCol = hfW/lfW;
    resRow = hfH/lfH;
    
    % Take only the x,y entries and convert with resolution
    r = ceil(hfstate(2)/resRow);
    c = ceil(hfstate(1)/resCol);
    
    % Make sure (r,c) are within world bounds
    r = min(lfH,max(1,r));
    c = min(lfW,max(1,c));
end