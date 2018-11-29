%% Run the exploration & planning procedure. 
% TODO:
% - [done] assume sensing radius is square + obstacles are square -- compute 
%   intersection of the two to get new square that becomes a l(x)
% - [done] put in warm-starting based on updated sensor measurements
% - try to plot different perspectives to verify the computation (?)
% - plot the final V(x) that we got after execution is done
% - compute solution IF YOU KNEW ENTIRE ENVIRONMENT *beforehand*
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
lowRealObs = [4,1];
upRealObs = [7,4];

% Setup environment representation.
env = Environment(lowEnv, upEnv, lowRealObs, upRealObs);

% Setup lower and upper computation domains and discretization.
gridLow = [0;0;-pi];
gridUp = [5;5;pi];
N = [21;21;21];

% Timestep for computation and simulation.
dt = 0.05;

% Setup current known obstacle list.
lowObsXY = [[4;1]]; %[[lowRealObs(1);lowRealObs(2)]];
upObsXY = [[5;4]]; %[[upRealObs(1);upRealObs(2)]]; 

% Setup current known env.
lowKnownXY = [0.05; 0.05]; 
upKnownXY = [4.95; 4.95]; %[9.95;6.95];

% Setup avoid set object and compute first set.
set = AvoidSet(gridLow, gridUp, N, dt);
set.computeAvoidSet(lowObsXY, upObsXY, lowKnownXY, upKnownXY, [], []);

%% Simulate dubins car moving around environment and the safe set changing
hold on

% Initial condition.
x = [2.0; 2.5; pi/2];

% Plot level set corresponding to the initial theta
convergedSetHandle = set.plotLevelSet(x(3), true);
envHandle = env.plotEnvironment();

% Draw rectangle which represents sensed (x,y) states.
senseWidth = set.gridUp(1) - set.gridLow(1);
senseHeight = set.gridUp(2) - set.gridLow(2);
computeGridVis = rectangle('Position',[set.gridLow(1),set.gridLow(2),senseWidth,senseHeight], ...
    'FaceColor',[0.5 .5 .5 .2], 'Linestyle', 'none');

% Plot car.
carVis = plot(x(1), x(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');

% Plot sensing radius (as rectangle).
senseRad = 1.5;
lowx = x(1)-senseRad;
lowy = x(2)-senseRad;
senseVis = rectangle('Position',[lowx,lowy,senseRad*2,senseRad*2], 'EdgeColor', [1,0,0,0.5]);

% Total simulation timesteps.
T = 200; 

for t=1:T
    % Get the current control.
    u = getControl(t);
    
    % Apply control to dynamics.
    dx = dynamics(set.dCar,t,x,u);
    x = x + dx*dt;
    
    % (1) check if sensing region is outside known environment
     lowSense = [x(1)-senseRad, x(2)-senseRad];
     upSense = [x(1)+senseRad, x(2)+senseRad];
     [isOutside,lowAmnt,upAmnt] = sensingOutsideEnv(lowSense,upSense,lowKnownXY,upKnownXY);
     
     if isOutside
         % if yes, 
         %   (2) enlarge the compute grid size by the amount its outside
         lowKnownXY = lowKnownXY + lowAmnt;  
         upKnownXY = upKnownXY + upAmnt;
         set.updateGridBounds(lowKnownXY, upKnownXY);
     end

    % (3) check if sensor picked up obstacle or free-space.
    [hasIntersection, lowXY, upXY] = env.obstacleIntersection(lowSense, upSense);
    % if detected an obstacle, add the coordinates to the obstacle list
    % (where columns represent unique obstacles)
    if hasIntersection
        lowObsXY = [lowObsXY, lowXY];
        upObsXY = [upObsXY, upXY];
    end
    
    % (4) update l(x) based on sensor hits 
    % (5) recompute V'(x) by warm-starting with previous V(x)
    if isOutside || hasIntersection % NOTE: IN GENERAL WE WOULDNT HAVE THIS CHECK HERE.
        set.computeAvoidSet(lowObsXY, upObsXY, lowKnownXY, upKnownXY, lowSense, upSense);
    end
    
    % -------------- Plotting -------------- %
    
    % Delete old visualizations.
    delete(computeGridVis);
    
    % Draw rectangle which represents sensed (x,y) states.
    senseWidth = set.gridUp(1) - set.gridLow(1);
    senseHeight = set.gridUp(2) - set.gridLow(2);
    computeGridVis = rectangle('Position',[set.gridLow(1),set.gridLow(2),senseWidth,senseHeight], ...
        'FaceColor',[0.5 .5 .5 .2], 'Linestyle', 'none');

    % Delete old visualizations.
    delete(carVis);
    
    % Plot the state of the car (point).
    carVis = plot(x(1), x(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
    drawnow   
    
    % Plot heading.
    center = x(1:2);
    % rotation matrix
    R = [cos(x(3)) -sin(x(3)); 
         sin(x(3)) cos(x(3))];
    % heading pt
    hpt = [0.5; 0];
    hptRot = R*hpt + center;
    p2 = plot([center(1) hptRot(1)], [center(2) hptRot(2)]);
    p2.Color(4) = 0.5;
    
    % Delete old visualizations.
    delete(senseVis);
    delete(convergedSetHandle);
    
    % Plot level set corresponding to the current theta.
    convergedSetHandle = set.plotLevelSet(x(3), true);
    envHandle = env.plotEnvironment();
    
    % Plot sensing radius.
    lowx = x(1)-senseRad;
    lowy = x(2)-senseRad;
    senseVis = rectangle('Position',[lowx,lowy,senseRad*2,senseRad*2],'EdgeColor',[1,0,0,0.5]);
    
    % ----------------------------------- %
    
    % Pause based on timestep.
    pause(dt);
end

%% Check if (and if yes, by how much) the sensing rectangle is outside the environment.
function [isOutside,lowAmnt,upAmnt] = sensingOutsideEnv(lowSense,upSense,lowEnv,upEnv)
    isOutside = false;
    lowAmnt = [0.0; 0.0];
    upAmnt = [0.0; 0.0];
    
    % Check if the lower left corner of sensing is outside lower left of 
    % environment. Compute how far if yes. 
    if lowSense(1) < lowEnv(1)
        lowAmnt(1) = lowSense(1) - lowEnv(1);
        isOutside = true;
    end
    if lowSense(2) < lowEnv(2)
        lowAmnt(2) = lowSense(2) - lowEnv(2);
        isOutside = true;
    end
        
    % Check if upper right corner of sensing is outside upper right of 
    % environment. Compute how far if yes. 
    if upSense(1) > upEnv(1) 
        upAmnt(1) = upSense(1) - upEnv(1);
        isOutside = true;
    end
    if upSense(2) > upEnv(2)
        upAmnt(2) = upSense(2) - upEnv(2);
        isOutside = true;
    end
end

%% Returns control to apply to car at a particular time.
function u = getControl(t)
    if t >= 1 && t < 20
        u = 0.0;
    elseif t >= 20 && t < 40
        u = -1.0;
    elseif t >= 40 && t < 50
        u = 1.0;
    elseif t >=50 && t < 70
        u = -1.0;
    else
        u = 0.0;
    end
    
% for init condition: x = [2.0; 2.5; pi/2];
%     if t >= 1 && t < 40
%         u = 0.8;
%     elseif t >= 40 && t < 60
%         u = -1.0;
%     elseif t >= 60 && t < 70
%         u = 1.0;
%     elseif t >=70 && t < 95
%         u = -1.0;
%     else
%         u = 0.0;
%     end
end