classdef Environment < handle
    %ENVIRONMENT Represents (x,y) planar environment.
    
    properties
        lowEnv      % lower (x,y) corner of environment
        upEnv       % upper (x,y) corner of environment
        lowObs      % lower (x,y) of the rectangular obstacle 
        upObs       % upper (x,y) of the rectangular obstacle
        avoidSet    % stores avoidSet computed by HJIPDE_solve()
    end
    
    methods
        %% Constructor.
        function obj = Environment(lowEnv, upEnv, lowObs, upObs)
            obj.lowEnv = lowEnv;
            obj.upEnv = upEnv;
            obj.lowObs = lowObs;
            obj.upObs = upObs;
        end
        
        %% Plots the environment with the obstacle.
        % Return: h -- figure handle
        function h = plotEnvironment(obj)
            % draw *actual* obstacle
            width = obj.upObs(1) - obj.lowObs(1);
            height = obj.upObs(2) - obj.lowObs(2);
            obsCoord = [obj.lowObs(1), obj.lowObs(2), width, height];
            h = rectangle('Position', obsCoord, 'Linewidth', 0.5, 'LineStyle', '--');
            
            % setup the figure axes to represent the entire environment
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
        end
    end
end

