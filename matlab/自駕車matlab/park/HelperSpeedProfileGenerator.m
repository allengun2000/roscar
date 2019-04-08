%HelperSpeedProfileGenerator Generate a simple desired speed profile
%   This is a helper class that generates desired speed profile based on
%   the reference path data. The speed profile consists of three phases:
%       
%   Accelerate  - accelerate to the maximum speed from the starting speed
%   Constant    - stay at the maximum speed
%   Decelerate  - decelerate to the end speed
%
%   Use the setReferencePath method to set reference path obtained from the
%   spline fitter and the generate method to generate the speed profile.
%   Currently only forward motion (speed > 0) is supported.
%
%   NOTE: The name of this class and it's functionality may change without
%   notice in a future release, or the class itself may be removed.
%
%   profiler = HelperSpeedProfileGenerator(splineData) returns a object to
%   plan speed. splineData is the output of HelperCubicSplineFit that
%   stores the reference path information.
% 
%   
%   HelperSpeedProfileGenerator properties:
%   StartSpeed       - Start speed of the vehicle (in m/s)  
%   EndSpeed         - End speed of the vehicle (in m/s)     
%   MaxSpeed         - Maximum speed of the vehicle (in m/s)
%   MaxAcceleration  - Maximum acceleration/deceleration magnitude (in m/s^2)
%   RefPathLength    - Length of the reference path curve at each 
%                      point (in m) (read-only)
% 
% 
%   HelperSpeedProfileGenerator methods:
%   setReferencePath    - Set smoothed reference path
%   generate            - Generate the speed profile for the reference path
%   
% 
%   See also HelperCubicSplineFit
% 

% Copyright 2017 The MathWorks, Inc.

classdef HelperSpeedProfileGenerator < handle

    properties 
        %StartSpeed Start speed
        %   Default : 0 (m/s)
        StartSpeed        = 0
        
        %EndSpeed Ending speed
        %   Default : 0 (m/s)
        EndSpeed          = 0
        
        %MaxSpeed Maximum speed
        %   Default : 5 (m/s)
        MaxSpeed          = 5
        
        %MaxAcceleration Maximum acceleration of a car
        %   Default : 3 (m/s^2)
        MaxAcceleration   = 3
    end
    
    properties (SetAccess = private)
        %RefPathLength Total length of the reference path curve
        RefPathLength
    end
    
    properties (Access = private)
        %RefPath A matrix storing the reference path data
        ReferencePathData
    end
    
    methods
        %------------------------------------------------------------------
        function obj = HelperSpeedProfileGenerator(splineData)
            
            if nargin == 0
                return
            end

            obj.setReferencePath(splineData);     
        end
        
        %------------------------------------------------------------------
        function setReferencePath(obj, splineData)
            %setReferencePath set the reference path for the speed profile
            %generator.
            
            % Validate spline data
            validateattributes(splineData, {'struct'}, {'scalar'}, ...
                'splineData', mfilename);
            
            % Extract spline data from struct and store in a matrix
            refPathData = [splineData.x, splineData.y, splineData.dx, ...
                splineData.dy, splineData.kappa, splineData.ts];
            
            % Retrieve spline data
            obj.ReferencePathData = refPathData;
            
            % Calculate path length
            obj.setRefPathLength();
        end
        
        %------------------------------------------------------------------
        function refSpeed = generate(obj)
            %generate generate a speed profile.
            
            % Validate speed values
            if obj.StartSpeed > obj.MaxSpeed                
                error('Expected StartSpeed to be less than MaxSpeed');
            end
            if obj.EndSpeed > obj.MaxSpeed                
                error('Expected EndSpeed to be less than MaxSpeed');
            end
            
            % Check if the maximum speed can be attained on the reference
            % path. If not, calculate a new maximum admissible speed based
            % on reference path length
            if obj.checkReachMaxSpeed()
                maxSpeed = obj.MaxSpeed;
            else 
                maxSpeed = sqrt((2*obj.MaxAcceleration*obj.RefPathLength(end)+...
                    obj.StartSpeed^2 + obj.EndSpeed^2)/2);
            end
            
            % Initialize the speed vector with maximum speed
            refSpeed = ones(size(obj.ReferencePathData, 1), 1)*maxSpeed;
            
            % Accelerate phase
            speed_acc = sqrt(2*obj.MaxAcceleration*obj.RefPathLength +...
                obj.StartSpeed^2);
            idx_acc = (speed_acc <= maxSpeed);
            refSpeed(idx_acc) = sqrt(2*obj.MaxAcceleration*obj.RefPathLength(idx_acc) +...
                obj.StartSpeed^2);
            
            % Decelerate phase
            speed_dec = sqrt(2*obj.MaxAcceleration*(obj.RefPathLength(end)-obj.RefPathLength) +...
                obj.EndSpeed^2);
            idx_dec = (speed_dec <= maxSpeed);
            refSpeed(idx_dec) = sqrt(2*obj.MaxAcceleration*(obj.RefPathLength(end)-obj.RefPathLength(idx_dec)) +...
                obj.EndSpeed^2);
        end   
    end
    
    %----------------------------------------------------------------------
    % Path related methods
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function setRefPathLength(obj)
            %setRefPathLength Compute the length of path at each point
            %   
            dX = obj.ReferencePathData(:, 3); 
            dY = obj.ReferencePathData(:, 4); 
            Ts = obj.ReferencePathData(:, 6); 
            
            vel = sqrt(dX.^2 + dY.^2);

            obj.RefPathLength = cumtrapz(Ts, vel);
        end

        %------------------------------------------------------------------
        function reachMaxSpeed = checkReachMaxSpeed(obj)
            %checkReachMaxSpeed Check if the vehicle can reach the maximum
            %   speed
            dis = (2*obj.MaxSpeed^2 - obj.StartSpeed^2 - ...
                obj.EndSpeed^2)/(2*obj.MaxAcceleration);
            
            if dis > obj.RefPathLength(end)
                reachMaxSpeed = false;
            else
                reachMaxSpeed = true;
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Setters
    %----------------------------------------------------------------------    
    methods
        %------------------------------------------------------------------
        function set.StartSpeed(obj, startSpeed)
  
            validateattributes(startSpeed, {'double'}, ...
                {'scalar','nonnegative'}, 'startSpeed', mfilename);
            
            obj.StartSpeed = startSpeed;
        end
        
        %------------------------------------------------------------------
        function set.EndSpeed(obj, endSpeed)          
            validateattributes(endSpeed, {'double'}, ...
                {'scalar','nonnegative'}, 'endSpeed', mfilename);
            
            obj.EndSpeed = endSpeed;
        end
        
        %------------------------------------------------------------------
        function set.MaxSpeed(obj, maxSpeed)           
            validateattributes(maxSpeed, {'double'}, ...
                {'scalar','nonnegative'}, 'maxSpeed', mfilename);
            
            obj.MaxSpeed = maxSpeed;
        end
    end
end

