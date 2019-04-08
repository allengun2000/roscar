%HelperFeedbackController Compute control commands to drive vehicle
%   This is a helper class that drives the vehicle to follow the planned
%   reference path. Use updateControlCommands to compute front wheel speed
%   and steering angle commands based on the current pose and reference
%   path information. The feedback controller minimizes the distance
%   between the rear wheel and the reference path. It also minimizes the
%   difference between vehicle heading and the tangent at the nearest point
%   on the path to the rear wheel.
%
%
%   NOTE: The name of this class and it's functionality may change without
%   notice in a future release, or the class itself may be removed.
%
%   controller = HelperFeedbackController(splineData, refSpeeds, vehicleDims) 
%   returns a controller object. splineData is the output of
%   HelperCubicSplineFit that stores the reference path information.
%   vehicleDims is a vehicleDimensions object storing vehicle dimensions.
%
%
%   HelperFeedbackController properties:
%    Wheelbase            - Wheelbase of the vehicle
%    FrontWheelSpeed      - Front wheel speed command
%    SteeringAngle        - Steering command
%    MinTurningRadius     - Minimum turning radius of the vehicle
%    KTheta               - Gain of controlling orientation
%    KPos                 - Gain of controlling position
%    GoalDisTolerance     - Tolerance in checking ReachGoal
%    ReachGoal            - Flag indicating if the vehicle reach the goal
%                           (read-only)
%
%   HelperFeedbackController methods:
%   setProfile            - Set speed profile
%   updateControlCommands - Compute speed and steering
%   resetReachGoal        - Reset ReachGoal flag to false  
%
%   See also HelperSpeedProfileGenerator, vehicleDimensions
%

% Copyright 2017 The MathWorks, Inc.

%   References
%   ----------
%   [1] Paden, Brian, et al. "A survey of motion planning and control
%       techniques for self-driving urban vehicles." IEEE Transactions on
%       Intelligent Vehicles vol. 1, issue 1, pp. 33-55, 2016.

classdef HelperFeedbackController < handle
    
    properties
        %Wheelbase
        %   A scalar specifying the distance between the front and rear
        %   axles.
        %
        %   Default: 2.8 (m)
        Wheelbase        = 2.8
        
        %FrontWheelSpeed
        %   Front wheel speed
        %
        %   Default: 0 (m/s)
        FrontWheelSpeed  = 0
        
        %SteeringAngle
        %   Steering angle of the vehicle
        %
        %   Default: 0 (deg)
        SteeringAngle    = 0
        
        %MinimumTurningRadius
        %   Minimum turning radius of the vehicle
        %
        %   Default: 4 (m)
        MinTurningRadius = 4
        
        %KTheta
        %   Gain of controlling the vehicle orientation
        %
        %   Default: 4
        KTheta           = 4;
        
        %KPos
        %   Gain of controlling the distance from reference path
        %
        %   Default: 1
        KPos             = 1;
        
        %GoalDisTolerance
        %   Acceptable distance tolerance from the goal in checking if
        %   reaching the goal
        %
        %   Default: 0.1 (m)
        GoalDisTolerance = 0.1;
    end
    
    properties (SetAccess = private)
        %ReachGoal
        %   Flag signifying if the vehicle reaches the goal
        %
        %   Default: false
        ReachGoal       = false
    end
    
    properties (Access = private, Dependent)
        %MaxSteeringAngle Maximum steering angle of the vehicle
        %
        MaxSteeringAngle
    end
    
    properties (Access = private, Hidden)
        %ReferencePathData Matrix stores the reference path data and the
        %   corresponding desired speed
        %
        ReferencePathData = zeros(1000, 6);
    end
    
    methods
        %------------------------------------------------------------------
        function controller = HelperFeedbackController(splineData, refSpeeds, vehicleDims)
            
            % Validate spline data
            validateattributes(splineData, {'struct'}, {'scalar'}, ...
                'splineData', mfilename);
            
            % Validate reference speed
            validateattributes(refSpeeds, {'double'}, {'column'}, ...
                'refSpeeds', mfilename);
            
            % Validate vehicle dimensions
            validateattributes(vehicleDims, {'vehicleDimensions'}, ...
                {'scalar'}, 'vehicleDims', mfilename);
            
            % Extract spline data from struct and store in a matrix
            controller.ReferencePathData = [splineData.x, splineData.y, splineData.dx, ...
                splineData.dy, splineData.kappa, refSpeeds];
            
            % Set wheelbase
            controller.Wheelbase = vehicleDims.Wheelbase;
        end
        
        %------------------------------------------------------------------
        function setProfile(controller, splineData, refSpeeds)
            
            % Validate spline data
            validateattributes(splineData, {'struct'}, {'scalar'}, ...
                'splineData', mfilename);
            
            % Validate reference speed
            validateattributes(refSpeeds, {'double'}, {'column'}, ...
                'refSpeeds', mfilename);
             
            % Extract spline data from struct and store in a matrix
            controller.ReferencePathData = [splineData.x, splineData.y, splineData.dx, ...
                splineData.dy, splineData.kappa, refSpeeds];
        end
        
        %------------------------------------------------------------------
        function [vf,delta] = updateControlCommands(controller,pose)
            
            % Get current pose
            xr = pose(1);
            yr = pose(2);
            theta = deg2rad(pose(3)); % radian
            
            % Get the reference path data
            pathData = controller.ReferencePathData;
            
            % Compute distance to the points on reference path
            dis = sqrt((pathData(:,1)-xr).^2 + (pathData(:,2)-yr).^2);
            
            % Check if reach the tolerance region of the goal
            if dis(end) < controller.GoalDisTolerance
                controller.ReachGoal = true;
                vf    = controller.FrontWheelSpeed;
                delta = controller.SteeringAngle;
                return
            end
            
            % Find the closest point on the reference path
            [row, ~] = find(dis == min(dis), 1);
            
            % Only need this if in Simulink
            idx = row(1, 1);
            
            % Check if reaching the end of path. This is necessary
            % because the vehicle might be far away from the path but the
            % closest reference point is still the goal point
            if idx == size(pathData, 1)
                controller.ReachGoal = true;
                vf    = controller.FrontWheelSpeed;
                delta = controller.SteeringAngle;
                return
            end
            
            % Front speed command
            vf = pathData(idx, 6);
            
            % Rear wheel speed
            vr = vf * cosd(controller.SteeringAngle);
            
            % Nontrivial speed to trigger orientation controller
            if vf < 0.1
                vf = 0.1;
            end
            
            xRef  = pathData(idx, 1);
            yRef  = pathData(idx, 2);
            dxRef = pathData(idx, 3);
            dyRef = pathData(idx, 4);
            kappa = pathData(idx, 5);
            
            % Tangent direction of the reference path
            tHat = [dxRef, dyRef]/ norm([dxRef, dyRef]);
            
            % Tracking error vector
            d = [xr, yr] - [xRef, yRef];
            
            % Compute position error
            posError = -(d(1)*tHat(2) - d(2)*tHat(1));
            
            % Desired heading angle
            thetaBar = mod(atan2(dyRef, dxRef), 2*pi);
            
            % Compute heading angle error
            thetaDiff = theta - thetaBar;
            mag = min(mod(thetaDiff, 2*pi), 2*pi-mod(thetaDiff, 2*pi));
            
            % Determine turn left or turn right
            if thetaDiff < -pi || (thetaDiff>=0 && thetaDiff<=pi)
                thetaError = mag;
            else
                thetaError = -mag;
            end
            
            % Update steering angle by updating heading rate (dtheta)
            if abs(thetaError) < 1e-6 % Use L'hospital rule
                omega = vr * kappa * cos(thetaError) / (1 - kappa*posError) -...
                    controller.KTheta * abs(vr) * thetaError - ...
                    controller.KPos * vr * posError;
            else
                omega = vr * kappa * cos(thetaError) / (1 - kappa*posError) -...
                    controller.KTheta * abs(vr) * thetaError - ...
                    controller.KPos * vr * sin(thetaError)/ thetaError * posError;
            end
            
            % Calculate steering rate
            if abs(omega) > abs(sind(controller.MaxSteeringAngle)*vf /...
                    controller.Wheelbase) % exceed maximum steering rate
                % Saturate steering angle
                delta = sign(omega) * controller.MaxSteeringAngle;
            else
                delta = asind(omega * controller.Wheelbase / vf);
            end
            
            debug = false;
            if debug
                [thetaBar, theta, thetaError, posError, delta, vf, omega, ...
                    vr * kappa * cos(thetaError) / (1 - kappa*posError), ...
                    controller.KTheta * abs(vr) * thetaError, ...
                    controller.KPos * vr * posError, dis(end)]' % debug use
            end
            
            % Control commands
            controller.FrontWheelSpeed = vf;
            controller.SteeringAngle   = delta; % in degree
        end
        
        %------------------------------------------------------------------
        function resetReachGoal(controller)
            controller.ReachGoal = false;
        end
    end
    
    %----------------------------------------------------------------------
    % Accessors
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function maxSteer = get.MaxSteeringAngle(controller)
            maxSteer = atand(controller.Wheelbase/controller.MinTurningRadius);
        end
    end
end

