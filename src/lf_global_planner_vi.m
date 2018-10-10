%% Value-iteration with low-fidelity model.
function optV = lf_global_planner_vi(goal, width, height)
    global actions gridW gridH xgoal INF 

    % Goal location
    xgoal = goal;
    %xgoal = [4; 9];
    %xgoal = [8; 18];
    
    % Definition of infinity
    INF = 100000;

    % World bounds. 
    % Lower LH corner of world is (1,1)
    % Upper RH corner of world is (8,10)
    gridW = width; %8; 
    gridH = height; %10; 
    %gridW = 16;
    %gridH = 20;
    
    % Action list
    actions = {'up', 'down', 'left', 'right'};
    
    % Compute value
    gamma = 0.9;
    optV = valueIteration(gamma);
    
    % Plot the world
    plotWorld(optV)
end

%% Value iteration algorithm
function V = valueIteration(gamma)
    global gridW gridH xgoal INF
    maxIter = 1000;
    eps = 1e-8;
    % Important to initialize V with -inf everywhere but goal
    V = ones(gridH, gridW)*(-INF);
    V(xgoal(2),xgoal(1)) = 0.0;
    for i=1:maxIter
        prevV = V;
        V = runIteration(prevV, gamma);
        % Return if we converged
        diff = sum(sum(abs(prevV - V)));
        if diff <= eps
            fprintf('Value iteration converged at iteration: %d\n', i);
            return 
        else
            fprintf('Not converged: %f\n', diff);
        end
    end
end

%% Evaluate Policy by updating the value of every state by diffusing the 
% rewards backwards through the dynamics of the world and current policy
function V = runIteration(V, gamma)
    global gridW gridH 
    for r=1:gridH
        for c=1:gridW
            allowedA = allowedActions(r, c);
            [~,numAllowed] = size(allowedA);
            qsa = zeros(size(allowedA));
            for idx=1:numAllowed
                a = allowedA{idx};
                [rnext, cnext, ~] = transition(r,c,a);
                qsa(idx) = reward(r,c,a) + gamma*V(rnext,cnext);
            end
            % Do \argmax_a Q(s,a)
            V(r,c) = max(qsa);
        end
    end
end

%% Reward function
function re = reward(r,c,a)
    global xgoal INF gridH gridW
    
    % specify square obstacles
    lowerO1 = [gridH/3, gridW/3];
    upperO1 = [2*gridH/3+1, 2*gridW/3]; %2*gridW/3+2];
    
    obs1height = floor(upperO1(1) - lowerO1(1));
    pad = 2*floor(obs1height/2);
    
    lowerO2 = [gridH/3+1, gridW/3-1]; %gridW/3-4];
    upperO2 = [gridH/3+pad-1, gridW/3];

    inObs1 = insideObs(r,c,lowerO1,upperO1); %(r == 5 && c == 2) || (r == 6 && c == 2) || (r >=4 && r <= 7 && c >= 3 && c <= 5);
    inObs2 = insideObs(r,c,lowerO2,upperO2);
    if r == xgoal(2) && c == xgoal(1)
        % if you're at the goal, no penalty
        re = 0.0;
    elseif inObs1 || inObs2
        % penalize being inside obs with -Inf
        re = -INF;
    else
        % penalize all actions -1 (bc up,down,left,right)
        re = -1;
    end
end

%% Tells you if (r,c) is inside obstacle
function inObs = insideObs(r,c,lower,upper)
    if r >= lower(1) && r <= upper(1) && c >= lower(2) && c <= upper(2)
        inObs = 1;
    else
        inObs = 0;
    end
end

%% Initializes rewards on gridworld
function reward = initializeRewards()
    global gridH gridW xgoal INF
    reward = zeros(gridH, gridW);
    % Set goal reward
    reward(xgoal(2),xgoal(1)) = 0.0;
    % Set obstacle penalty
    reward(5,2) = -INF; 
    reward(6,2) = -INF; 
    for i=4:7
        for j=3:5
            reward(i,j) = -100000;%-Inf;
        end
    end
end

%% Get a list of all possible actions that are allowed from (r,c)
function allowedAct = allowedActions(r, c)
    global actions
    allowedAct = {};
    [~,numA] = size(actions);
    for i=1:numA
        a = actions{i};
        [~,~,illegal] = transition(r,c,a);
        if ~illegal
            allowedAct{end+1} = a;
        end
    end
end

%% Given state = (row,col) and current action, returns 
% the next state we would end up in if we took this action.
% Sets illegal flag if this action is not allowed
function [rnext,cnext,illegal] = transition(r, c, a)
	global gridW gridH
    illegal = 0;
    rnext = r;
    cnext = c;
    if strcmp(a,'up')
        rnext = r + 1;
    elseif strcmp(a,'down')
        rnext = r - 1;
    elseif strcmp(a,'left')
        cnext = c - 1;
    elseif strcmp(a,'right')
        cnext = c + 1;
    end
        
    % Check that the new locations are within world bounds
    if rnext <= 0 || rnext > gridH
        rnext = r;
        illegal = 1;
    end
    if cnext <= 0 || cnext > gridW
        cnext = c;
        illegal = 1;
    end
end

%% Plotting
function plotWorld(world)
    global xgoal gridH gridW
    clf
    hold on
    % Plot values of gridworld
    clims = [-10.0,0.0];
    imagesc(world, clims);
    colorbar

    % Plot start and goal
    plot(4, 0.5, 'bo', 'MarkerSize', 10);
    plot(xgoal(1), xgoal(2), 'ro', 'MarkerSize', 10);

    % Plot obstacles
    %viscircles([2.5 5.5], 0.8, 'Color', 'k');
    %viscircles([3 5.5], 1, 'Color', 'k');
    %viscircles([4 5.5], 1.5, 'Color', 'k');

    axis([0 gridW 0 gridH]);
end
