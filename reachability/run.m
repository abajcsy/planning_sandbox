%% Run the exploration & planning procedure. 
% DONE:
% - [done] assume sensing radius is square + obstacles are square -- compute 
%   intersection of the two to get new square that becomes a l(x)
% - [done] put in warm-starting based on updated sensor measurements
% - [done] try to plot different perspectives to verify the computation (?)
% - [done] compute solution IF YOU KNEW ENTIRE ENVIRONMENT *beforehand*

% TODO:
% - put in discounting
% - plot the final V(x) that we got after execution is done
% - implement MPC-style planning with the value function being queried for
%   collision-checking
% - look at Kene's temporal differencing work
% - look at approximate reachability techniques

% Clear old figure plotting and variables.
clf 
clear

% Setup environment bounds.
lowEnv = [0;0];
upEnv = [10;7];

% Setup obstacle.
lowRealObs = [4;1];
upRealObs = [7;4];

% Setup lower and upper computation domains and discretization.
gridLow = [lowEnv;-pi];
gridUp = [upEnv;pi];
N = [31;31;21];

% Timestep for computation and simulation.
dt = 0.05;

% Initial condition.
x = [2.0; 2.5; pi/2];

%% Compute first safe set based on sensing. 

% get the sensing radius
senseRad = 1.5;
lowSense = [x(1)-senseRad; x(2)-senseRad];
upSense = [x(1)+senseRad; x(2)+senseRad];

% TODO: WARM STARTING DOESN'T WORK RN.
% If we want to warm start with prior value function.
warmStart = false;
% Setup avoid set object and compute first set.
set = AvoidSet(gridLow, gridUp, lowRealObs, upRealObs, N, dt, warmStart);
set.computeAvoidSet(lowSense, upSense);

%% Plot initial conditions, sensing, and safe set.

hold on

% Plot l(x) and V(x).
plt = Plotter(lowEnv, upEnv, lowRealObs, upRealObs);
valueFunc = plt.plotFuncLevelSet(set.grid, -set.valueFun(:,:,:,1), x(3), true, [1,0,0], 'hot');
beliefObstacle = plt.plotFuncLevelSet(set.grid, -set.lCurr, x(3), true, [0.6,0.6,0.6], 'bone');

% Plot environment, car, and sensing.
envHandle = plt.plotEnvironment();
carVis = plt.plotCar(x);
senseVis = plt.plotSensing(x, senseRad);

%% Simulate dubins car moving around environment and the safe set changing

% Total simulation timesteps.
T = 200; 

for t=1:T
    % Get the current control.
    u = getControl(t);

    % Apply control to dynamics.
    dx = dynamics(set.dCar,t,x,u);
    x = x + dx*dt;
    
    % get the sensing radius
    lowSense = [x(1)-senseRad; x(2)-senseRad];
    upSense = [x(1)+senseRad; x(2)+senseRad];
    
    % update l(x) and the avoid set.
    set.computeAvoidSet(lowSense, upSense);
    
    % -------------- Plotting -------------- %
    % plot belief obstacle -- original l(x) which can be found at valueFun(end)
    % plot reachable set -- V_converged which can be found at valueFun(1)
    
    % Delete old visualizations.
    delete(beliefObstacle);
    delete(valueFunc);
    
    % Plot belief obstacle (i.e. everything unsensed) and the value function.
    valueFunc = plt.plotFuncLevelSet(set.grid, -set.valueFun(:,:,:,1), x(3), true, [1,0,0], 'hot');
    beliefObstacle = plt.plotFuncLevelSet(set.grid, -set.lCurr, x(3), true, [0.6,0.6,0.6], 'bone');

	% Delete old visualizations.
    delete(carVis);
    delete(senseVis);
    
    % Plot the state of the car (point), environment, and sensing.
    carVis = plt.plotCar(x);
    envHandle = plt.plotEnvironment();
	senseVis = plt.plotSensing(x, senseRad);
    % ----------------------------------- %
    
    % Pause based on timestep.
    pause(dt);
end

%% Returns control to apply to car at a particular time.
function u = getControl(t)
    if t >= 1 && t < 20
        u = [1.0, 0.0];
    elseif t >= 20 && t < 40
        u = [1.0,-1.0];
    elseif t >= 40 && t < 50
        u = [1.0,1.0];
    elseif t >=50 && t < 70
        u = [1.0,-1.0];
    elseif t >= 70 && t < 120
        u = [1.0,0.0];
    elseif t >= 120 && t < 150
        u = [1.0,-1.0];
    else
        u = [1.0,0.0];
    end
end