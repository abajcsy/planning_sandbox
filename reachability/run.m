%% Run the exploration & planning procedure. 

% Clear old figure plotting.
clf 

% Setup environment bounds.
lowEnv = [0,0];
upEnv = [10,7];

% Setup obstacle.
lowObs = [4,1];
upObs = [7,4];

% Setup environment representation.
env = Environment(lowEnv, upEnv, lowObs, upObs);

% Setup lower and upper computation domains and discretization.
gridLow = [0;0;-pi];
gridUp = [5;5;pi];
N = [21;21;21];

% Timestep for computation and simulation.
dt = 0.05;

% Setup current sensed obstacle.
lowObsXY = [4;1];
upObsXY = [5;4];

% Setup avoid set object and compute first set.
set = AvoidSet(gridLow, gridUp, N, dt);
set.computeAvoidSet(lowObsXY, upObsXY);

%% Simulate dubins car moving around environment and the safe set changing
hold on

% Initial condition.
x = [0; 2.5; 0];

% Plot level set corresponding to the initial theta
convergedSetHandle = set.plotLevelSet(x(3), true);
envHandle = env.plotEnvironment();

% Draw rectangle which represents sensed (x,y) states.
senseWidth = set.gridUp(1) - set.gridLow(1);
senseHeight = set.gridUp(2) - set.gridLow(2);
sensing = rectangle('Position',[set.gridLow(1),set.gridLow(2),senseWidth,senseHeight], ...
    'FaceColor',[0.5 .5 .5 .2], 'Linestyle', 'none');

% Plot sensing radius.
senseRad = 1.5
senseVis = viscircles([x(1), x(2)], senseRad, 'Color', [1,0,0,0.5]);

% Total simulation timesteps.
T = 100; 

for t=1:T
    % Get the current control.
    u = getControl(t);
    
    % Apply control to dynamics.
    dx = dynamics(set.dCar,t,x,u);
    x = x + dx*dt;
    
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
    
    % Plot sensing radius.
    senseVis = viscircles([x(1), x(2)], senseRad, 'Color', [1,0,0,0.5]);
    
    % Plot level set corresponding to the current theta.
    convergedSetHandle = set.plotLevelSet(x(3), true);
    envHandle = env.plotEnvironment();
    
    % Pause based on timestep.
    pause(dt);
end

%% Returns control to apply to car at a particular time.
function u = getControl(t)
    if t >= 1 && t <= 20
        u = -0.5;
    elseif t >= 20 && t <= 80
        u = 0.8;
    else 
        u = -0.8;
    end
end