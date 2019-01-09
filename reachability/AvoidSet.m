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
        grid            % Computation grid struct
        dt              % Timestep in discretization
        computeTimes    % Stores the computation times
        dynSys          % Dynamical system (dubins car)
        pdDims          % Which dimension is periodic?
        lowRealObs      % (x,y) lower left corner of obstacle
        upRealObs       % (x,y) upper right corner of obstacle
        lReal           % cost function representing TRUE environment
        lRealMask       % bitmap representing cost function lReal
        lCurr           % current cost function representation (constructed from measurements)
        valueFun        % Stores most recent converged value function 
        timeDisc        % Discretized time vector          
        schemeData      % Used to specify dyn, grid, etc. for HJIPDE_solve()
        HJIextraArgs    % Specifies extra args to HJIPDE_solve()
        warmStart       % (bool) if we want to warm start with prior V(x)
        firstCompute    % (bool) flag to see if this is the first time we have done computation
    end
    
    methods
        %% Constructor. 
        % NOTE: Assumes DubinsCar dynamics!
        function obj = AvoidSet(gridLow, gridUp, lowRealObs, upRealObs, N, dt, warmStart)
            obj.gridLow = gridLow;  
            obj.gridUp = gridUp;    
            obj.N = N;      
            obj.pdDims = 3;      % 3rd dimension is periodic for dubins
            obj.dt = dt;
            obj.grid = createGrid(obj.gridLow, obj.gridUp, obj.N, obj.pdDims);
            obj.computeTimes = [];
            
            % Store obstacle coordinates.
            obj.lowRealObs = lowRealObs;
            obj.upRealObs = upRealObs;
            
            % Create the 'ground-truth' cost function from obstacle.
            lowObs = [lowRealObs;-inf];
            upObs = [upRealObs;inf];
            obj.lReal = shapeRectangleByCorners(obj.grid,lowObs,upObs);
            
            % Create bitmask representing 'ground-truth' cost function.
            mask = obj.lReal;
            mask(mask == 0) = -0.1; % THIS IS A HACK!
            mask(mask > 0.0) = 0.0;
            mask(mask < 0.0) = 1.0;
            obj.lRealMask = mask;
            
            % Store the current estimate of the cost function (from
            % sensing).
            obj.lCurr = NaN;
            
            % Before the problem starts, dont initialize the value
            % function.
            obj.valueFun = NaN;
            obj.warmStart = warmStart;
            
            % flag to tell us if this is the first time we have computed
            % lCurr or valueFun
            obj.firstCompute = true;
            
            % Input bounds for dubins car.
            wMax = 1;
            vrange = [0,1];
            
            % --- DISTURBANCE --- %
            dMode = 'min';
            dMax = [.2, .2, .2];
            % ------------------- %

            % Define dynamic system.
            % NOTE: init condition doesn't matter for computation.
            xinit = [2.0, 2.5, 0.0]; 
            
            % Create dubins car where u = [v, w]
            obj.dynSys = Plane(xinit, wMax, vrange, dMax);
            
            % Time vector.
            t0 = 0;
            tMax = 100; % compute infinite-horizon solution
            obj.timeDisc = t0:obj.dt:tMax; 
            
            % Control is trying to maximize value function.
            uMode = 'max';
            
            % Put grid and dynamic systems into schemeData.
            obj.schemeData.grid = obj.grid;
            obj.schemeData.dynSys = obj.dynSys;
            obj.schemeData.accuracy = 'high'; % Set accuracy.
            obj.schemeData.uMode = uMode;
            obj.schemeData.dMode = dMode;

            % Convergence information
            obj.HJIextraArgs.stopConverge = 1;
            obj.HJIextraArgs.convergeThreshold = .01;
            
            % Built-in plotting information
            %obj.HJIextraArgs.visualize.valueSet = 1;
            %obj.HJIextraArgs.visualize.initialValueSet = 1;
            %obj.HJIextraArgs.visualize.figNum = 1; %set figure number
            %obj.HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
        end
        
        %% Computes avoid set. 
        % Inputs:
        %   senseData [vector] - if rectangle sensing region, 
        %                        (x,y) coords of lower left sensing box and
        %                        (x,y) coords of upper right sensing box.
        %                        if circle sensing region, 
        %                        (x,y) coords of center and radius
        %   senseShape [string] - either 'rectangle' or 'circle'
        % Outputs:
        %   dataOut             - infinite-horizon (converged) value function 
        function dataOut = computeAvoidSet(obj, senseData, senseShape)
            
            % ---------- START CONSTRUCT l(x) ---------- %
            
            % Construct cost function for region outside sensing radius
            % or 'known' environment.
            % NOTE: need to negate the default shape function to make sure
            %       free space is assigned (+) and unknown space is (-)
            if strcmp(senseShape, 'rectangle')
                lowSenseXY = senseData(:,1);
                upSenseXY = senseData(:,2);
                lowObs = [lowSenseXY;-inf];
                upObs = [upSenseXY;inf];
                sensingShape = -shapeRectangleByCorners(obj.grid,lowObs,upObs);
            else % if circular sensing region
                center = senseData(:,1);
                radius = senseData(1,2);
                sensingShape = -shapeCylinder(obj.grid, 3, center, radius);
            end
            
            % Union the sensed region with the actual obstacle.
            unionL = shapeUnion(sensingShape, obj.lReal);
            if isnan(obj.lCurr)
                obj.lCurr = unionL;
            else
                obj.lCurr = shapeIntersection(unionL, obj.lCurr);
            end
            
            % ------------- CONSTRUCT V(x) ----------- %
            if obj.firstCompute
                % First time we are doing computation, set data0 to lcurr
                data0 = obj.lCurr;
                obj.firstCompute = false;
            else
                if obj.warmStart
                    % If we are warm starting, use the old value function
                    % as initial V(x) and then the true/correct l(x) in targets
                    data0 = obj.valueFun(:,:,:,1);
                    % TODO: WARM STARTING DOESN'T WORK RN.
                else
                    data0 = obj.lCurr;
                end
            end

            obj.HJIextraArgs.targets = obj.lCurr;
            %minWith = 'minVwithL';
            %minWith = 'minVOverTime';
            minWith = 'zero';
            
            % ------------ Compute value function ---------- % 
            [dataOut, tau, extraOuts] = ...
              HJIPDE_solve(data0, obj.timeDisc, obj.schemeData, minWith, obj.HJIextraArgs);
            
            % Update internal variables.
            obj.valueFun = dataOut;
            obj.computeTimes = tau;
        end
        
        %% Get shape that represents sensed part of obstacle.
        function sensedObsShape = getSensedObs(obj, senseData)
            % NOTE: assumes circular sensing radius.
            center = senseData(:,1);
            radius = senseData(1,2);
            sensingShape = shapeCylinder(obj.grid, 3, center, radius);
            
            % Union the sensed region with the actual obstacle.
            sensedObsShape = shapeIntersection(sensingShape, obj.lReal);
        end
    end
end

