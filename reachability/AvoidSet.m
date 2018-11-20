classdef AvoidSet < handle
    %AVOIDSET Computes and stores an avoid set.
    % Find the set of all states that can avoid a given set of	
    % states despite the disturbance for a time	duration of	T
    % 
    % min_u max_d min_{t \in [0,T]} l(x(t))
    %   s. t.   \dot{x} = f(x,u,d,t)
    %           L = {x : l(x) <= 0}
    
    properties
        gridLow         % Lower corner of computation domain
        gridUp          % Upper corner of computation domain
        N               % Number of grid points per dimension
        grid            % Computation grid
        dt              % Timestep in discretization
        reachTube       % Stores the converged backwards reachable tube
        computeTimes    % Stores the computation times
        dCar            % Dynamical system (dubins car)
        timeDisc        % Discretized time vector          
        schemeData      % Used to specify dyn, grid, etc. for HJIPDE_solve()
        HJIextraArgs    % Specifies extra args to HJIPDE_solve()
    end
    
    methods
        %% Constructor. 
        % NOTE: Assumes DubinsCar dynamics!
        function obj = AvoidSet(gridLow, gridUp, N, dt)
            obj.gridLow = gridLow;  
            obj.gridUp = gridUp;    
            obj.N = N;      
            pdDims = 3;      % 3rd dimension is periodic for dubins
            obj.dt = dt;
            obj.grid = createGrid(obj.gridLow, obj.gridUp, obj.N, pdDims);
            obj.reachTube = [];
            obj.computeTimes = [];
            
            % Input bounds for dubins car.
            speed = 1;
            wMax = 1;

            % Define dynamic system.
            % NOTE: init condition doesn't matter for computation.
            xinit = [0, 2.5, 0]; 
            obj.dCar = DubinsCar(xinit, wMax, speed);
            
            % Time vector.
            t0 = 0;
            tMax = 100; % compute infinite-horizon solution
            obj.timeDisc = t0:obj.dt:tMax; 
            
            % Control is trying to maximize value function.
            uMode = 'max';
            
            % Put grid and dynamic systems into schemeData.
            obj.schemeData.grid = obj.grid;
            obj.schemeData.dynSys = obj.dCar;
            obj.schemeData.accuracy = 'high'; % Set accuracy.
            obj.schemeData.uMode = uMode;

            % Convergence information
            obj.HJIextraArgs.stopConverge = 1;
            obj.HJIextraArgs.convergeThreshold=.01;
        end
        
        %% Computes avoid set. 
        % Inputs:
        %   lowObsXY [vector] - (x,y) coords of lower left-corner of obstacle
        %   upObsXY [vector] - (x,y) coords of upper right-hand corner of obstacle
        function dataOut = computeAvoidSet(obj, lowObsXY, upObsXY)
            
            % Construct obstacle set. Function is positive inside the set
            % and negative outside.
            lowObs = [lowObsXY(1);lowObsXY(2);-inf];
            upObs = [upObsXY(1);upObsXY(2);inf];
            data0 = shapeRectangleByCorners(obj.grid,lowObs,upObs);
            
            % Compute value function
            % minWith zero gives us a tube instead of a set.
            minWith = 'zero';
            [dataOut, tau, extraOuts] = ...
              HJIPDE_solve(data0, obj.timeDisc, obj.schemeData, minWith, obj.HJIextraArgs);
          
            % Update internal variables.
            obj.reachTube = dataOut;
            obj.computeTimes = tau;
        end
        
        %% Plot slice of value function at specific theta
        % Inputs:
        %   theta [float] - angle for which to plot the level set
        %   plotFinalTime [bool] - if true, plots converged level set.
        %                          Otherwise plots initial level set.
        % Outputs: 
        %   plots level set in (x,y) for fixed theta.
        function h = plotLevelSet(obj, theta, plotFinalTime)
            % By default plot final time set.
            data = obj.reachTube(:,:,:,end);
            extraArgs.LineWidth = 1.0;
            
            if ~plotFinalTime
                % If plotting inital set.
                data = obj.reachTube(:,:,:,1);
                extraArgs.LineWidth = 3.0;
            end
            
            % Grab slice at theta.
            [gPlot, dataPlot] = proj(obj.grid, data, [0 0 1], theta);

            % Visualize final set.
            % NOTE: plot -data because by default contourf plots all values
            % that are ABOVE zero, but inside our obstacle we have values
            % BELOW zero.
            h = visSetIm(gPlot, -dataPlot, 'k', 0, extraArgs);
            colormap(bone)
        end
    end
end

