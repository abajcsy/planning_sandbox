classdef Environment
    %ENVIRONMENT Represents (x,y) planar environment.
    
    properties
        lowEnv      % lower (x,y) corner of environment
        upEnv       % upper (x,y) corner of environment
        lowObs      % lower (x,y) of the rectangular obstacle 
        upObs       % upper (x,y) of the rectangular obstacle
        avoidSet    % stores avoidSet computed by HJIPDE_solve()
    end
    
    methods
        function obj = Environment(lowEnv, upEnv, lowObs, upObs)
            %ENVIRONMENT Construct an environment object.
            obj.lowEnv = lowEnv;
            obj.upEnv = upEnv;
            obj.lowObs = lowObs;
            obj.upObs = upObs;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

