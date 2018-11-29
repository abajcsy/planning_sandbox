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
        pdDims          % Which dimension is periodic?
        data0           % Stores initial value function (could be l(x) or old V(0))
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
            obj.pdDims = 3;      % 3rd dimension is periodic for dubins
            obj.dt = dt;
            obj.grid = createGrid(obj.gridLow, obj.gridUp, obj.N, obj.pdDims);
            obj.reachTube = [];
            obj.computeTimes = [];
            
            % Before the problem starts, dont initialize the value
            % function.
            obj.data0 = NaN;
            
            % Input bounds for dubins car.
            speed = 1;
            wMax = 1;

            % Define dynamic system.
            % NOTE: init condition doesn't matter for computation.
            xinit = [2.0, 2.5, 0.0]; 
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
            obj.HJIextraArgs.convergeThreshold=.06;
        end
        
        %% Computes avoid set. 
        % Inputs:
        %   lowObsXY [vector]   - (x,y) coords of lower left-corner of obstacle
        %   upObsXY [vector]    - (x,y) coords of upper right-hand corner of obstacle
        %   lowKnownXY [vector] - (x,y) coords of lower left compute/known
        %   env
        %   upKnownXY [vector]  - (x,y) coords of upper right compute/known
        %   env
        %   lowSenseXY [vector] - (x,y) coords of lower left sensing box
        %   upSenseXY [vector]  - (x,y) coords of upper right sensing box
        % Outputs:
        %   dataOut             - infinite-horizon (converged) value function 
        function dataOut = computeAvoidSet(obj, lowObsXY, upObsXY, ...
                lowKnownXY, upKnownXY, lowSenseXY, upSenseXY)
            
            % ---------- START CONSTRUCT l(x) ---------- %
            
            % Construct cost function for obstacle. 
            % Function is positive inside the set and negative outside.
            [~,numObs] = size(lowObsXY); % get number of obstacles
            
            % NOTE: we need at least one obstacle!
            low = lowObsXY(:,1);
            up = upObsXY(:,1);
            lowObs = [low(1);low(2);-inf];
            upObs = [up(1);up(2);inf];
            obsShape = shapeRectangleByCorners(obj.grid,lowObs,upObs);
            
            % Union all other obstacles with the first one.
            for idx=2:numObs
                low = lowObsXY(:,idx);
                up = upObsXY(:,idx);
                lowObs = [low(1);low(2);-inf];
                upObs = [up(1);up(2);inf];
                newObs = shapeRectangleByCorners(obj.grid,lowObs,upObs);
                obsShape = shapeUnion(obsShape, newObs);
            end
            
            % Construct cost function for region outside 'known' environment.
            % NOTE: need to negate the default shape function to make sure
            %       free space is assigned (+) and unknown space is (-)
            lowObs = [lowKnownXY(1);lowKnownXY(2);-inf];
            upObs = [upKnownXY(1);upKnownXY(2);inf];
            envShape = -shapeRectangleByCorners(obj.grid,lowObs,upObs);
            
            %obj.plotFuncLevelSet(envShape, pi/2, true, 'm');
            
            if ~isempty(lowSenseXY) && ~isempty(upSenseXY)
                % Construct cost function for region outside sensing radius.
                % NOTE: need to negate the default shape function to make sure
                %       free space is assigned (+) and unknown space is (-)
                lowObsSense = [lowSenseXY(1);lowSenseXY(2);-inf];
                upObsSense = [upSenseXY(1);upSenseXY(2);inf];
                senseShape = -shapeRectangleByCorners(obj.grid,lowObsSense,upObsSense);
            end
            
            % Combine the obstacle and environment cost functions together.
            costData = shapeUnion(obsShape, envShape);
            
            % Combine the unioned obs/env cost with the sensing radius
            if ~isempty(lowSenseXY) && ~isempty(upSenseXY)
                %obj.plotFuncLevelSet(senseShape, pi/2, true, 'r');
                %obj.plotFuncLevelSet(costData, pi/2, true, 'g');
                lOfX = shapeIntersection(costData, senseShape);
            else
                lOfX = costData;
            end
            
            % Sanity check: you can plot the cost function l(x).
            %obj.plotFuncLevelSet(lOfX, pi/2, true, 'b');

            % ------------- CONSTRUCT V(x) ----------- %
            
            if isnan(obj.data0)
                % If we don't have a value function initialized, 
                % use l(x) as our initial value function.
                obj.data0 = lOfX;
                % minWith zero gives us a tube instead of a set.
                minWith = 'zero';
            else
                % Otherwise, use the prior instantiation of the 
                % value function data0 = Vold, and just set new l(x)
                obj.HJIextraArgs.targets = lOfX;
                minWith = 'minVwithL';
            end
            
            % ------------ Compute value function ---------- %
            [dataOut, tau, extraOuts] = ...
              HJIPDE_solve(obj.data0, obj.timeDisc, obj.schemeData, minWith, obj.HJIextraArgs);
          
            % Update internal variables.
            obj.data0 = dataOut; % may not neat reachTube anymore
            %obj.plotFuncLevelSet(obj.data0, pi/2, true, 'y');
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
            data = obj.data0(:,:,:,end);
            extraArgs.LineWidth = 1.0;
            
            if ~plotFinalTime
                % If plotting inital set.
                data = obj.data0(:,:,:,1);
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
            drawnow   
        end
        
        %% Plot level set of arbitrary function
        function h = plotFuncLevelSet(obj, func, theta, plotFinalTime, color)
            % By default plot final time set.
            data = func(:,:,:,end);
            extraArgs.LineWidth = 1.0;
            
            if ~plotFinalTime
                % If plotting inital set.
                data = func(:,:,:,1);
                extraArgs.LineWidth = 3.0;
            end
            
            % Grab slice at theta.
            [gPlot, dataPlot] = proj(obj.grid, data, [0 0 1], theta);

            % Visualize final set.
            % NOTE: plot -data because by default contourf plots all values
            % that are ABOVE zero, but inside our obstacle we have values
            % BELOW zero.
            h = visSetIm(gPlot, -dataPlot, color, 0, extraArgs);
            colormap(bone)
            drawnow   
        end
        
        %% Updates the grid bounds based on given bounds.
        function updateGridBounds(obj, newGridLow, newGridUp)
            obj.gridLow(1:2) = newGridLow;  
            obj.gridUp(1:2) = newGridUp;  
            
            % Create new computation grid.
            %gNew = createGrid(obj.gridLow, obj.gridUp, obj.N, obj.pdDims);
            gOld = obj.grid;
            
            % may want to do migrate data into new grid
            gNew = createGrid(obj.gridLow, obj.gridUp, obj.N, obj.pdDims);
            %obj.plotFuncLevelSet(obj.data0, pi/2, true, 'g');
            obj.data0 = migrateGrid(gOld, obj.data0, gNew, 'min');
            %obj.plotFuncLevelSet(obj.data0, pi/2, true, 'b');
            obj.grid = gNew;
        end
    end
end

