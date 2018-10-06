%% High-Fidelity Planner
% Plan a trajectory from start to goal in an environment with a Dubins car
% model. Here, we use a 3D model where the state is
%      x = [pos_x; pos_y; theta_heading]
% and the control is
%      u = [lin_vel; angular_vel]
% The optimization variable is denoted by
%      z = [x0; x1; x2; ...; xT; u0; u1; ... ; uT-1]
% so there are T+1 planned states, and T planned controls.

function [xopt, uopt] = hf_planner(x0, planT, planDt)
global nx nu T dt xgoal obsType
clf

% State/control dim 
nx = 3;
nu = 2;

% Time horizon and timestep
T = planT;
dt = planDt;

% Goal location
xgoal = [4; 9; pi/2]; 

% Obstacle type (circle or square or none)
obsType = 'square';

% Initial trajectory
z0 = zeros(nx*(T+1) + nu*T, 1);
for t=0:T
    z0(nx*t+1:nx*t+nx) = linInterp(x0, xgoal, t);
    %z0(nx*t+1:nx*t+nx) = segmentInterp(x0, xgoal, t);
end

% Plot initial trajectory
init = 1;
plotTraj(z0(1:nx*(T+1)), init);

% Generate all the optimization variables
[A,b,Aeq,beq,lb,ub] = generateOptProb(x0);

% Get nonlinear constraints
nonlcon = @constraintFun;

% Solve for optimal trajectory
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[zopt, objval] = fmincon(@cost,z0,A,b,Aeq,beq,lb,ub,nonlcon,options);

fprintf("Initial traj cost: %f\n", cost(z0));
fprintf("Optimized traj cost: %f\n", objval);

xopt = zopt(1:nx*(T+1));
uopt = zopt(nx*(T+1)+1:nx*(T+1) + nu*T);

% Plot final trajectory
init = 0;
plotTraj(xopt, init);
end

%% Cost function (of entire optimization variable)
function c = cost(z)
    global nx nu T xgoal

    c = 0.0;
    xweight = 10.0;
    uweight = 5.0;
    
    % Running cost
    for t=0:T-1
        xt = z(nx*t+1:nx*t+nx);
        ut = z(nx*T + nu*t + 1:nx*T + nu*t + nu);
        %c = c + xweight*norm(xt-xgoal)^2 + uweight*norm(ut)^2;
        c = c + xweight*norm(xt(1:2)-xgoal(1:2))^2 + uweight*norm(ut)^2;
        %c = c + uweight*norm(ut)^2;
    end
    
    % Terminal cost
    c = c + xweight*norm(z(nx*T+1:nx*T+nx) - xgoal)^2;
end

%% Constraints Setup
function [c,ceq] = constraintFun(z)
    global nx nu T dt obsType
    
    % Initialize constraint vectors
    if strcmp(obsType, 'none')
        c = []; 
    else
        c = zeros(nx*(T+1),1);   % for obstacles
    end
    ceq = zeros(nx*T,1); % for the dynamics
    
    for t = 0:T-1
        xstate = z(nx*t+1);
        ystate = z(nx*t+2);
        theta = z(nx*t+3);
        
        %====== Obstacle Constraints ======%
        %c(nx*t+1:nx*t+nx) = squareCons(xstate,ystate);
        %c(nx*t+1:nx*t+nx) = circleCons(xstate, ystate);
        if ~strcmp(obsType, 'none')
            c(nx*t+1:nx*t+nx) = lineCollisionCheck(z(nx*t+1:nx*t+2), z(nx*(t+1)+1:nx*(t+1)+2));
        end
        %=========================================% 
                
        % Dynamics: Equality constrain dynamics
        xt = z(nx*t+1:nx*t+nx);        
        ut = z(nx*(T+1) + nu*t + 1:nx*(T+1) + nu*t + nu);
        xt1 = z(nx*(t+1)+1:nx*(t+1)+nx);
        ceq(nx*t+1:nx*t+nx) = xt1 - (xt + dt*dyn(xt,ut)); 
    end
end

%% Optimization Problem Setup
function [A,b,Aeq,beq,lb,ub] = generateOptProb(x0)
    global nx nu T 
    % We have no linear constraints
    A = [];             
    b = [];     
    Aeq = [];   
    beq = [];   
    
    % Setup variables
    zLen = nx*(T+1) + nu*T;
    lb = zeros(zLen, 1);
    ub = zeros(zLen, 1);
    
    % Min and max on state and control
    minX = [0;0;0];       
    maxX = [8;10;2*pi];
    minU = [0;-2.63];
    maxU = [1;2.63];
    
    for t = 0:T
        if t == 0
            % Constrain the initial condition
            lb = setX(lb, t, x0);
            ub = setX(ub, t, x0);
        else
            % Set the state limits
            lb = setX(lb, t, minX);
            ub = setX(ub, t, maxX);
        end

        if t <= T-1
            % Set the control limits
            lb = setU(lb, t, minU);
            ub = setU(ub, t, maxU);
        end
    end
end

%% Set the state in the optimization vector
function z = setX(z, t, x)
    % Sets the state at time t in vector z
    %
    % Input:    z   - optimization vector
    %           t   - time point between [0,...T]
    %           x   - new configuration 
    % Output:   z   - updated optimization vector with x_t inside
    global nx
    z(nx*t + 1:nx*t + nx) = x;
end

%% Set the control in the optimization vector
function z = setU(z, t, u)
    % Sets the control at time t in vector z
    %
    % Input:    z   - optimization vector
    %           t   - time point between [0,...T]
    %           u   - new control
    % Output:   z   - updated optimization vector with u_t inside
    global nx nu T
    if(nx*T + nu*t + 1 > nx*(T+1) + nu*T || nx*T + nu*t + nu > nx*(T+1) + nu*T)
        error("Set U trying to index outside of dimensions!");
    end
    z(nx*(T+1) + nu*t + 1:nx*(T+1) + nu*t + nu) = u;
end

%% Linearly interpolate between two states
% Input:    x0          - start point of line
%           xf          - end point of line
%           t           - desired interpolation time
% Output:   xi          - interpolated point at time t
function xi = linInterp(x0, xf, t)
    global T
    xi = x0 + (t / T) * (xf - x0);
end

%% Parabola interpolate between two states
function xi = segmentInterp(x0, xf, t)
    global T
    % THREE SEGMENTS
    xm1 = [x0(1) + 3; (xf(2) - x0(2))/3 + x0(2); 0];
    xm2 = [x0(1) + 3; 2*(xf(2) - x0(2))/3 + x0(2); 0];
    T1 = T/3;
    T2 = 2*T/3;
    if t < T1
        xi = x0 + (t / T1) * (xm1 - x0);
    elseif t < T2 
        xi = xm1 + (t / T2) * (xm2 - xm1);
    else
        xi = xm2 + (t / T) * (xf - xm2);

    % TWO SEGMENTS
%     xm = [x0(1)+3; (xf(2) - x0(2))/2;0];
%     if t < T/2
%         xi = x0 + (t/T)*(xm - x0);
%     else
%         xi = xm + (t/T)*(xf - xm);
    end
end

%% Square obstacle constraints
function c = squareCons(xstate, ystate)   
    % [Square] Obstacle lower (xy) and upper (xy)
    obs1 = [1 5 3 7];
    w1 = obs1(3) - obs1(1);
    h1 = obs1(4) - obs1(2);
    cx1 = obs1(1) + w1/2;
    cy1 = obs1(2) + h1/2;
    
    obs2 = [3 3 6 7];
    w2 = obs2(3) - obs2(1);
    h2 = obs2(4) - obs2(2);
    cx2 = obs2(1) + w2/2;
    cy2 = obs2(2) + h2/2;
    
    % Compute distance to square 1
    dx1 = max(abs(xstate - cx1) - w1/2,0);
    dy1 = max(abs(ystate - cy1) - h1/2,0);
    c1 = -(dx1^2 + dy1^2);
    
    % Compute distance to square 2
    dx2 = max(abs(xstate - cx2) - w2/2, 0);
    dy2 = max(abs(ystate - cy2) - h2/2, 0);
    c2 = -(dx2^2 + dy2^2);
    
    c = max(c1,c2);
    
    % Constrain if inside obstacle 1
%     c1 = 0;
%     if xstate > obs1(1) && xstate < obs1(3) && ystate > obs1(2) && ystate < obs1(4)
%         c1 = -exp(-(sqrt((xstate - obs1(1))^2 + (ystate - obs1(2))^2) - obs1(3)));
%         %c(nx*t+1:nx*t+nx) = 1;
%     end
%     c2 = 0;
%     % Constrain if inside obstacle 2
%     if xstate > obs2(1) && xstate < obs2(3) && ystate > obs2(2) && ystate < obs2(4)
%         c2 = -exp(-(sqrt((xstate - obs2(1))^2 + (ystate - obs2(2))^2) - obs2(3)));
%         %c(nx*t+1:nx*t+nx) = 1;
%     end
%     c = max(c1,c2);
end

%% Circle obstacle constraints
function c = circleCons(xstate, ystate)
    % [Circle] Obstacle center (xy) and radius
    obs1 = [3 5.5 1];
    obs2 = [4 5.5 1.5];
    obs3 = [2.5 5.5 0.8];
    pad = 0;
    
    c1 = -(sqrt((xstate - obs1(1))^2 + (ystate - obs1(2))^2) - (obs1(3)+pad));
    c2 = -(sqrt((xstate - obs2(1))^2 + (ystate - obs2(2))^2) - (obs2(3)+pad));
    c3 = -(sqrt((xstate - obs3(1))^2 + (ystate - obs3(2))^2) - (obs3(3)+pad));
    c = max(c3,max(c1,c2));
end

%% Collision check between two states
function dist = lineCollisionCheck(x1, x2)
    global obsType
    numPts = 100;
    dist = 0;
    for i=0:numPts
        p = i/(numPts+1);
        q = x1 * (1 - p) + x2 * p;
        % Returns -dist to obstacle edge
        if strcmp('circle', obsType)
            c = circleCons(q(1), q(2));
        elseif strcmp('square', obsType)
            c = squareCons(q(1), q(2));
        end
        dist = max(c,dist);
    end
end

%% Plotting
function plotTraj(xopt, init)
    global nx T xgoal obsType
    hold on
    
    % setup unique colors for inital pt or nah
    color = [0.5 0.5 0.5];
    if init
        color = [0.9 0.9 0.9];
    end
    
    % Plot start and goal
    plot(4, 0.5, 'bo', 'MarkerSize', 10);
    plot(xgoal(1), xgoal(2), 'ro', 'MarkerSize', 10);

    % Plot square obstacles
    if strcmp(obsType, 'square')
        rectangle('Position', [1 5 2 2], 'LineWidth',2);
        rectangle('Position', [3 3 3 4], 'LineWidth',2);
    % Plot circle obstacles
    elseif strcmp(obsType, 'circle')
        viscircles([2.5 5.5], 0.8, 'Color', 'k');
        viscircles([3 5.5], 1, 'Color', 'k');
        viscircles([4 5.5], 1.5, 'Color', 'k');
    end
    
    % Plot trajectory
    traj = reshape(xopt, nx, T+1);
    p1 = plot(traj(1,:), traj(2,:), 'ko','MarkerSize', 5, 'MarkerEdgeColor', color, 'MarkerFaceColor', color);
    p1.Color(4) = 0.2;
    
    % Plot heading
    for t=1:T+1
        center = [traj(1,t); traj(2,t)];
        % rotation matrix
        R = [cos(traj(3,t)) -sin(traj(3,t)); 
             sin(traj(3,t)) cos(traj(3,t))];
        % heading pt
        hpt = [0.5; 0];
        hptRot = R*hpt + center;
        if init
            plot([center(1) hptRot(1)], [center(2) hptRot(2)], 'Color', color);
        else
            p2 = plot([center(1) hptRot(1)], [center(2) hptRot(2)]);
            p2.Color(4) = 0.2;
        end
    end
    % Bounds of world
    axis([0 8 0 10])
    set(gcf,'Position',[600 600 850 1000])
    drawnow
end
