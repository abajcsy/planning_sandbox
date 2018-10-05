%% High-Fidelity Planner
% Plan a trajectory from start to goal in an environment with a Dubins car
% model. Here, we use a 3D model where the state is
%      x = [pos_x; pos_y; theta_heading]
% and the control is
%      u = [lin_vel; angular_vel]
% The optimization variable is denoted by
%      z = [x0; x1; x2; ...; xT; u0; u1; ... ; uT-1]

global nx nu T dt xgoal

% Init conditions
x0 = [4.0; 0.5; pi/2];
%x0 = [4.0; 0.5; 3.0*pi/4.0];

% State/control dim 
nx = 3;
nu = 2;

% Time horizon and timestep
T = 3;
dt = 0.5;

% Goal location
xgoal = [4; 9; pi/2]; 

% Initial trajectory
z0 = zeros(nx*T + nu*(T-1), 1);
for t=0:T
    z0(nx*t+1:nx*t+nx) = linInterp(x0, xgoal, t);
end

% Plot initial trajectory
init = 1;
plotTraj(z0(1:nx*T), init);

% Generate all the optimization variables
[A,b,Aeq,beq,lb,ub] = generateOptProb(x0);

% Get nonlinear constraints
nonlcon = @constraintFun;

% Solve for optimal trajectory
zopt = fmincon(@cost,z0,A,b,Aeq,beq,lb,ub,nonlcon);
xopt = zopt(1:nx*T);
uopt = zopt(nx*T+1:nx*T + nu*(T-1));

% Plot final trajectory
init = 0;
plotTraj(xopt, init);

%% Dubins car dynamics
function dx = dyn(x,u)
    dx = [u(1)*cos(x(3)); u(1)*sin(x(3)); u(2)];
end

%% Cost function (of entire optimization variable)
function c = cost(z)
    global nx nu T xgoal
    
%     % Extract state and control parts of opt var
%     xvec = z(1:nx*T);
%     uvec = z(nx*T+1:nx*T+nu*(T-1));
%     
%     % NOTE: here i am also make a heading goal! 
%     xgoalmat = repmat(xgoal, 1, T);
%     xgoalvec = reshape(xgoalmat, nx*T, 1);
%     
%     alpha = 100.0;
%     beta = 1.0;
%     c = alpha*norm(xvec - xgoalvec)^2 + beta*norm(uvec)^2;

    c = 0.0;
    alpha = 1.0;
    beta = 1.0;
    
    % Running cost
    for t=0:T-1
        xt = z(nx*t + 1:nx*t + nx);
        ut = z(nx*T + nu*(t-1) + 1:nx*T + nu*(t-1) + nu);
        c = c + alpha*norm(xt - xgoal)^2 + beta*norm(ut)^2;
    end
    
    % Terminal cost
    c = c + alpha*norm(z(nx*(T-1)+1:nx*T) - xgoal)^2;
end

%% Constraints Setup
function [c,ceq] = constraintFun(z)
    global nx nu T dt
    
    % Obstacle lower (xy) and upper (xy)
    obs1 = [1 5 3 7];
    obs2 = [3 3 6 7];
    
    % Initialize constraint vectors
    c = []; %zeros(nx*T,1);   % for obstacles
    ceq = zeros(nx*T,1); % for the dynamics
    
    for t = 0:T-1
%         xstate = z(nx*t+1);
%         ystate = z(nx*t+2);
%         theta = z(nx*t+3);
%         
%         % Obstacle: Constrain if inside obstacle 1
%         if xstate > obs1(1) && xstate < obs1(3) && ystate > obs1(2) && ystate < obs1(4)
%             %c(nx*t+1:nx*t + nx) = exp(-((xstate - obs(1))^2 + obs(1))/10);
%             c(nx*t+1:nx*t + nx) = 1;
%         end
        

        %TODO: there are problems with indexing!!
        
        % Dynamics: Equality constrain dynamics
        xt = z(nx*t+1:nx*t+nx);
        ut = z(nx*T+nu*t+1:nx*T+nu*t+nu);
        xt1 = z(nx*(t+1)+1:nx*(t+1)+nx);
        ceq(nx*t+1:nx*t+nx) = xt1 - (xt + dt*dyn(xt,ut)); 
    end
end

%% Optimization Problem Setup
function [A,b,Aeq,beq,lb,ub] = generateOptProb(x0)
    global nx nu T xgoal
    % We have no linear constraints
    A = [];             
    b = [];     
    Aeq = [];   
    beq = [];   
    
    % Setup variables
    zLen = nx*T + nu*(T-1);
    lb = zeros(zLen, 1);
    ub = zeros(zLen, 1);
    
    % Min and max on state and control
    minX = [0;0;0];       
    maxX = [8;10;2*pi];
    minU = [0;0]; %[0;-2.63];
    maxU = [1;0]; %[1;2.63];
    
    for t = 0:T
        if t == 0
            % Constrain the initial condition
            lb = setX(lb, t, x0);
            ub = setX(ub, t, x0);
        %elseif t == T
        %    lb = setX(lb, t, xgoal);
        %    ub = setX(ub, t, xgoal);
        else
            % Set the state limits
            lb = setX(lb, t, minX);
            ub = setX(ub, t, maxX);
        end

        if t > 0
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
    z(nx*T + nu*(t-1) + 1:nx*T + nu*(t-1) + nu) = u;
end

function xi = linInterp(x0, xf, t)
    global T
    % Interpolates a single point at time t
    %
    % Input:    x0          - start point of line
    %           xf          - end point of line
    %           t           - desired interpolation time
    % Output:   xi          - interpolated point at time t
    xi = x0 + (t / T) * (xf - x0);
end

%% Plotting
function plotTraj(xopt, init)
    global nx T xgoal
    hold on
    
    % setup unique colors for inital pt or nah
    color = [0 0 0];
    if init
        color = [0.8 0.8 0.8];
    end
    
    % Plot start and goal
    plot(4, 0.5, 'bo', 'MarkerSize', 10);
    plot(xgoal(1), xgoal(2), 'ro', 'MarkerSize', 10);

    % Plot obstacles
    %rectangle('Position', [1 5 2 2])
    %rectangle('Position', [3 3 3 4])
    
    % Plot trajectory
    traj = reshape(xopt, nx, T);
    plot(traj(1,:), traj(2,:), 'ko','MarkerSize', 5, 'MarkerEdgeColor', color, 'MarkerFaceColor', color);
    
    % Plot heading
    for t=1:T
        center = [traj(1,t); traj(2,t)];
        % rotation matrix
        R = [cos(traj(3,t)) -sin(traj(3,t)); 
             sin(traj(3,t)) cos(traj(3,t))];
        % heading pt
        hpt = [0.5; 0];
        hptRot = R*hpt + center;
        plot([center(1) hptRot(1)], [center(2) hptRot(2)]);
    end
    
    % Bounds of world
    axis([0 8 0 10])
end
