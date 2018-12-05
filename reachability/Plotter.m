classdef Plotter
    %PLOTTER Plots level sets and environment and stuff.
    
    properties
        lowEnv      % lower (x,y) corner of environment
        upEnv       % upper (x,y) corner of environment
        lowObs      % lower (x,y) of the rectangular obstacle 
        upObs       % upper (x,y) of the rectangular obstacle
    end
    
    methods
        %% Constructor.
        function obj = Plotter(lowEnv, upEnv, lowObs, upObs)
            obj.lowEnv = lowEnv;
            obj.upEnv = upEnv;
            obj.lowObs = lowObs;
            obj.upObs = upObs;
        end
        
        %% Plots the environment with the obstacle.
        % Output: 
        %   e - figure handle
        function e = plotEnvironment(obj)
            % draw *actual* obstacle
            width = obj.upObs(1) - obj.lowObs(1);
            height = obj.upObs(2) - obj.lowObs(2);
            obsCoord = [obj.lowObs(1), obj.lowObs(2), width, height];
            e = rectangle('Position', obsCoord, 'Linewidth', 2.0, 'LineStyle', '--'); 
            %h = rectangle('Position', obsCoord, 'Linewidth', 0.5, 'FaceColor',[0,0,0]);
            
            % Setup the figure axes to represent the entire environment
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
            
            xlabel('x1');
            ylabel('x2');
            set(gca,'TickLength',[0 0]);
            box on
        end
        
        %% Plot level set of arbitrary function
        % Inputs:
        %   g [array]     - grid corresponding to data
        %   func [array]  - data for function to visualize
        %   theta [float] - angle for which to plot the level set
        %   visSet [bool] - if true, plots 2D slice of func.
        %                   Otherwise plots 3D.
        %   edgeColor [vector or string] - color of level set boundary
        %   cmap [string] - name of colormap to use
        % Outputs: 
        %   plots level set in (x,y) for fixed theta.
        function h = plotFuncLevelSet(obj, g, func, theta, visSet, edgeColor, cmap)
            
            % Grab slice at theta.
            [gPlot, dataPlot] = proj(g, func, [0 0 1], theta);
            extraArgs.LineWidth = 2;

            % Visualize final set.
            % NOTE: plot -data because by default contourf plots all values
            % that are ABOVE zero, but inside our obstacle we have values
            % BELOW zero.
            if visSet
                h = visSetIm(gPlot, dataPlot, edgeColor, 0, extraArgs);
            else
                h = visFuncIm(gPlot, dataPlot, edgeColor, 0.5);
                xlabel('V(x)');
            end

            colormap(cmap);
            xlabel('x1');
            ylabel('x2');
            grid off
            set(gca,'TickLength',[0 0]);
        end
        
        %% Plots dubins car point and heading.
        % Inputs:
        %   x [vector]  - 3D state of dubins car
        % Ouput:
        %   c   - handle for figure
        function c = plotCar(obj, x)
            c = plot(x(1), x(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
            % Plot heading.
            center = x(1:2);
            % Rotation matrix.
            R = [cos(x(3)) -sin(x(3)); 
                 sin(x(3)) cos(x(3))];
            % Heading pt.
            hpt = [0.5; 0];
            hptRot = R*hpt + center;
            p2 = plot([center(1) hptRot(1)], [center(2) hptRot(2)], 'LineWidth', 1.5);
            p2.Color(4) = 1.0;
        end
        
        %% Plots sensing radius centered around car position (x)
        % Inputs:
        %   x [vector]  - 3D state of dubins car
        %   senseRad [float] - sensing radius
        % Ouput:
        %   s   - handle for figure
        function s = plotSensing(obj, x, senseRad, senseShape)
            if strcmp(senseShape,'rectangle')
                lowx = x(1)-senseRad;
                lowy = x(2)-senseRad;
                s = rectangle('Position',[lowx,lowy,senseRad*2,senseRad*2], 'EdgeColor', [1,0,0,0.5], 'LineWidth', 2);
            else
                s = viscircles([x(1),x(2)],senseRad);
            end
        end
    end
end

