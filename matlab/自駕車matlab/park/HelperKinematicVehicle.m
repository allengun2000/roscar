%HelperKinematicVehicle Simulation of kinematic behavior of vehicle.
%   This is a helper class that manages pose update for a vehicle with
%   kinematics defined by the bicycle model.
%
%   NOTE: The name of this class and it's functionality may change without
%   notice in a future release, or the class itself may be removed.
%
%   vehicle = HelperKinematicVehicle(vehicleDims) returns a kinematics
%   model of vehicle. vehicleDims is a vehicleDimensions object storing
%   vehicle dimensions.
%
%   HelperKinematicVehicle properties:
%   Pose            - Current vehicle pose [x, y, theta] (read-only)
%   Speed           - Current forward velocity (in m/s) (read-only)
%   SteeringAngle   - Steering angle (read-only)
%
%   HelperKinematicVehicle methods:
%   setPose                     - Set vehicle pose
%   setControlCommand           - Set vehicle speed and steering angle
%   enableLimitedCommandTime    - Enable velocity command timeout
%   updateKinematics            - Propagate vehicle kinematic model
%
%   See also HelperVehicleSimulator, vehicleDimensions.
%

% Copyright 2017 The MathWorks, Inc.

classdef HelperKinematicVehicle < handle
    
    properties (SetAccess = private)
        %Pose Current vehicle pose [x, y, theta]
        Pose 
        
        %Speed Current forward velocity (in m/s)
        Speed   

        %Steering angle
        SteeringAngle 
    end
    
    properties
        %VehicleDimensions A vehicleDimensions object
        %   A vehicleDimensions object encapsulating dimensions of the
        %   vehicle. Vehicle dimensions must be defined in the same world
        %   units as the costmap.
        VehicleDimensions
        
        %MaxSteeringAngle Maximum steering angle
        %
        %   Default: 45 (deg)
        MaxSteeringAngle = 45
    end

    properties (Access = private, Dependent)
        %MinimumTurningRadius
        %   Minimum turning radius of vehicle
        MinTurningRadius
        
    end
    
    properties (Access = private)
        %TimeSinceLastCmd Elapsed time since the last velocity command
        %   This counts the time (in second), since the last unique
        %   velocity command was received.
        TimeSinceLastCmd = tic
                
        %MaxCommandTime The maximum time interval between velocity commands
        %   If more time (in seconds) than MaxCommandTime elapses between
        %   unique velocity commands, the vehicle is stopped.
        %   Default: 1 second
        MaxCommandTime = 1
        
        %EnableLimitedCommandTime Enable velocity command timeout.
        EnableLimitedCommandTime = true;
    end
    
    
    %----------------------------------------------------------------------
    % API
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function obj = HelperKinematicVehicle(vehicleDims)
            %HelperKinematicVehicle
            %   obj = HelperKinematicVehicle(vehicleDims) creates a
            %   kinematic vehicle simulator for a vehicle with dimensions
            %   specified in the vehicleDimensions object vehicleDims.
            
            obj.VehicleDimensions = vehicleDims;
        end
        
        %------------------------------------------------------------------
        function setPose(obj, pose)
            %setPose Set the pose of the vehicle
            %   setPose(OBJ, POSE) - Set vehicle's pose to a vector POSE in
            %   [x,y,theta] form. This also resets the linear and angular
            %   velocity to zero.
                        
            obj.Pose            = reshape(pose,1,3);
            obj.Speed           = 0;
            obj.SteeringAngle   = 0;
        end
        
        %------------------------------------------------------------------
        function enableLimitedCommandTime(obj, enableLimitedCommandTime)
            %enableLimitedCommandTime Enable velocity command timeout.
            %   enableLimitedCommandTime(obj, true) - Enable limited
            %   command time mode, vehicle's velocity is reset to zero if
            %   no new velocity commands arrive in the last MaxCommandTime
            %   seconds.
            %
            %   enableLimitedCommandTime(obj, false) - Disable limited
            %   command time mode.

            obj.EnableLimitedCommandTime = enableLimitedCommandTime;
        end
        
        %------------------------------------------------------------------
        function setControlCommand(obj, ctrlCmd)
            %setControlCommand Set vehicle speed and steering angle
            %
            %   setControlCommand(obj, [v,s]) sets the control command for
            %   the kinematic vehicle to [v,s]. v is the forward velocity
            %   in m/s and s is the steering angle in degrees.
            
            obj.Speed           = ctrlCmd(1);
            obj.SteeringAngle   = ctrlCmd(2);

            obj.TimeSinceLastCmd = tic;
        end
        
        %------------------------------------------------------------------
        function updateKinematics(obj, dt)
            %updateKinematics Propagate the kinematic model of the vehicle
            %   updateKinematics(obj, dt) - Propagate the kinematics model
            %   of the vehicle forward in time by dt.
            
            validateattributes(dt, {'numeric'}, {'scalar'}, ...
                'updateKinematics','dt');
            
            % If no velocity command is received within some time, stop 
            % the vehicle.
            if obj.EnableLimitedCommandTime && toc(obj.TimeSinceLastCmd) >...
                    obj.MaxCommandTime
                obj.SteeringAngle   = 0;
                obj.Speed           = 0;
            end

            % Limit steering angle to maximum allowable value
            delta = obj.SteeringAngle; 
            obj.SteeringAngle = sign(delta) * min(abs(delta), obj.MaxSteeringAngle);
            
            if abs(obj.SteeringAngle) < 1e-5
                obj.SteeringAngle = 0;
            end
            
            if abs(obj.Speed) < 1e-5
                obj.Speed = 0;
            end
            
            % Propagate vehicle state change based on velocities in rear
            % wheel coordinate
            delta_theta = obj.Speed/obj.VehicleDimensions.Wheelbase * sind(obj.SteeringAngle);
            dx = dt*obj.Speed*cosd(obj.Pose(3) + obj.SteeringAngle) + ...
                 dt*obj.VehicleDimensions.Wheelbase*sind(obj.Pose(3))*delta_theta;
            dy = dt*obj.Speed*sind(obj.Pose(3) + obj.SteeringAngle) - ...
                 dt*obj.VehicleDimensions.Wheelbase*cosd(obj.Pose(3))*delta_theta;
            dtheta = dt * delta_theta;
            
            % Update vehicle state accordingly
            obj.Pose = obj.Pose + [dx dy rad2deg(dtheta)];
            obj.Pose(3) = mod(obj.Pose(3), 360);
            
        end
    end
    
    %----------------------------------------------------------------------
    % Accessors
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function r = get.MinTurningRadius(obj)
            r = obj.VehicleDimensions.Wheelbase / tand(obj.MaxSteeringAngle);
        end
    end
end

