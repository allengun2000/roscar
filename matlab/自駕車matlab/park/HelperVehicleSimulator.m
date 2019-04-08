%HelperVehicleSimulator Simulator for kinematic vehicle.
%   This is a helper class which acts as a simple simulator for a kinematic
%   vehicle, by making use of a periodic timer to update kinematics
%   according to the Kinematic Bicycle model.
%
%   NOTE: The name of this class and it's functionality may change without
%   notice in a future release, or the class itself may be removed.
%
%   vehicleSim = HelperVehicleSimulator(costmap, vehicleDims) creates a
%   simulator for a kinematic vehicle whose dimensions are specified by
%   vehicleDims, on a map specified in costmap. costmap must be a
%   vehicleCostmap object. vehicleDims must be a vehicleDimensions object.
%  
%   HelperVehicleSimulator methods:
%   drive           - Drive vehicle using velocity commands
%   setVehiclePose  - Set vehicle at specific pose in world frame
%   getVehiclePose  - Return ground truth vehicle pose in world frame
%   showTrajectory  - Plot vehicle trajectory
%   
%   See also HelperKinematicVehicle, vehicleCostmap,
%   vehicleDimensions.

% Copyright 2017 The MathWorks, Inc.
classdef HelperVehicleSimulator < handle
    
    properties(Constant, Access = protected)
        %Step Integration step size (in seconds)
        Step = 0.01
        
        %PlotInterval Interval between plot updates (in seconds)
        %   The exact plot interval is also affected by MATLAB callback
        %   queue.
        PlotInterval = 0.2
        
        %FigureName Name of simulation figure window
        FigureName = 'Automated Valet Parking'
    end
    
    properties (SetAccess = private)
        %vehicle Object representing a vehicle
        Vehicle
        
        %PlotTrajectory Indicates whether vehicle's trajectory is plotted.
        PlotTrajectory = true
    end
    
    properties (Hidden)
        %Axes Handle to main plot axes
        Axes
    end
    
    %----------------------------------------------------------------------
    % Timers
    %----------------------------------------------------------------------
    properties (Access = private)
        %KinematicsTimer Timer for kinematics integration
        KinematicsTimer = timer.empty
        
        %PlotTimer Timer triggering plot updates
        PlotTimer = timer.empty
    end
    
    %----------------------------------------------------------------------
    % Simulated vehicle
    %----------------------------------------------------------------------
    properties (Access = private)
        %InitialVehiclePose Initial pose of vehicle
        %   Initial vehicle pose specified as a 3-element vector [x y
        %   theta].
        InitialVehiclePose = [0 0 0];
        
        %TrajectoryBuffer Stores trajectory of vehicle in a circular buffer
        TrajectoryBuffer
        
        %TrajectoryBufferCapacity The capacity of the circular buffer
        TrajectoryBufferCapacity = 1e4;
        
        %TrajectoryIndex Pointer to last element in the trajectory buffer
        TrajectoryIndex = 0;
        
        %TrajectoryUpdateTolerance Pose change tolerance for trajectory update
        %   Trajectory buffer only gets updated if the vehicles pose change
        %   exceeds TrajectoryUpdateTolerance.
        TrajectoryUpdateTolerance = 0.01;
    end
    
    %----------------------------------------------------------------------
    % Graphics
    %----------------------------------------------------------------------
    properties (Access = private)
        %Figure Handle to figure window
        Figure = []
        
        %vehicleBodyHandle Handle to robot body graphical representation
        VehicleBodyHandle = []
        
        %TrajectoryHandle Graphics handle for vehicle's trajectory
        TrajectoryHandle = []
    end
    
    %----------------------------------------------------------------------
    % API
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function obj = HelperVehicleSimulator(costmap, vehicleDims)
            
            % Initialize trajectory buffer
            obj.TrajectoryBuffer = NaN(obj.TrajectoryBufferCapacity,3);
            
            % Set initial state for vehicle
            obj.Vehicle = HelperKinematicVehicle(vehicleDims);
            obj.Vehicle.setPose(obj.InitialVehiclePose);
            
            % Set up the figure
            obj.setupFigure(costmap);
            
            % Start the two timing loops
            obj.KinematicsTimer = HelperTimer(obj.Step, ...
                @obj.updateKinematics);
            
            obj.PlotTimer = HelperTimer(obj.PlotInterval, ...
                @obj.updatePlot);
        end
        
        %------------------------------------------------------------------
        function showTrajectory(obj, showTrajectory)
            %showTrajectory Plot vehicle trajectory
            %   showTrajectory(vehicleSim, state) Plots the trajectory of
            %   the vehicle is state is true. If state is false, the
            %   trajectory is not plotted.
            
            validateattributes(showTrajectory, {'numeric','logical'},...
                {'scalar','nonempty','binary'}, 'showTrajectory',...
                'showTrajectory');
            
            obj.PlotTrajectory = showTrajectory;
        end
        
        %------------------------------------------------------------------
        function delete(obj)
            %delete Delete simulator object
            %   Deletes the simulator, either on object destruction or when
            %   the figure window is closed.
            
            if ~isempty(obj.KinematicsTimer) && isvalid(obj.KinematicsTimer)
                obj.KinematicsTimer.Timer.stop;
                delete(obj.KinematicsTimer);
            end
            
            if ~isempty(obj.PlotTimer) && isvalid(obj.PlotTimer)
                obj.PlotTimer.Timer.stop;
                delete(obj.PlotTimer);
            end
            
            if ~isempty(obj.Figure)
                delete(obj.Figure);
            end
        end
        
        %------------------------------------------------------------------
        function setVehiclePose(obj, pose)
            %setVehiclePose Set vehicle at specific pose in world frame.
            %   setVehiclePose(vehicleSim, vehiclePose) sets the vehicle's
            %   pose to vehiclePose, specified as [x, y, theta] in world
            %   coordinates.
            
            validateattributes(pose, {'numeric'}, ...
                {'vector', 'numel', 3, 'nonnan', 'finite', 'real'},...
                'setVehiclePose', 'pose');
            
            obj.Vehicle.setPose(pose);
            
            obj.InitialVehiclePose = reshape(pose,1,3);
            obj.TrajectoryBuffer = NaN(obj.TrajectoryBufferCapacity, 3);
            obj.TrajectoryIndex = 0;
        end
        
        %------------------------------------------------------------------
        function pose = getVehiclePose(obj)
            %getVehiclePose Return ground truth vehicle pose
            %   vehiclePose = getVehiclePose(vehicleSim) returns
            %   vehiclePose, the ground truth vehicle pose, specified as
            %   [x, y, theta] in world coordinates.
            
            if isempty(obj.Vehicle)
                pose = [];
                return;
            end
            
            pose = obj.Vehicle.Pose;
        end
        
        %------------------------------------------------------------------
        function speed = getVehicleSpeed(obj)
            %getvehicleSpeed Return vehicle speed
            % vehicleSpeed = getVehicleSpeed(vehicleSim) returns
            % vehicleSpeed, the linear speed of the vehicle.
            
            if isempty(obj.Vehicle)
                speed = [];
                return;
            end
            
            speed = obj.Vehicle.Speed;
        end
        
        %------------------------------------------------------------------
        function steer = getVehicleSteer(obj)
            %getvehicleSpeed Return vehicle speed
            % vehicleSpeed = getVehicleSpeed(vehicleSim) returns
            % vehicleSpeed, the linear speed of the vehicle.
            
            if isempty(obj.Vehicle)
                steer = [];
                return;
            end
            
            steer = obj.Vehicle.SteeringAngle;
        end
        
        %------------------------------------------------------------------
        function drive(obj, v, delta)
            %drive Drive vehicle using velocity command
            %   drive(vehicleSim, v, omega) - Drives the vehicle with
            %   linear velocity v and angular velocity omega.
            
            validateattributes(v, {'double'}, ...
                {'scalar', 'nonnan', 'finite', 'real'}, 'drive', 'v');
            
            validateattributes(delta, {'double'}, ...
                {'scalar', 'nonnan', 'finite', 'real'}, 'drive', 'omega');
            
            % Set velocity commands for vehicle. They will be executed in
            % the next simulation loop.
            obj.Vehicle.setControlCommand([v, delta]);
        end
    end
    
    %----------------------------------------------------------------------
    % Implementation
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function setupFigure(obj, costmap)
            %setupFigure Setup figure window
            
            obj.Figure = figure;
            obj.Figure.Name = obj.FigureName;
            obj.Figure.NumberTitle = 'off';
            
            ax = axes(obj.Figure);
            
            plot(costmap, 'Parent', ax, 'Inflation', 'off');
            legend off
            
            hold(ax, 'on');
            title(ax, '');
            obj.Axes = ax;
            
            obj.plotVehicle();
        end
        
        %------------------------------------------------------------------
        function updateKinematics(obj, ~, ~)
            %updateKinematicsUpdate The kinematic model of the vehicle
            
            obj.Vehicle.updateKinematics(obj.Step);
            obj.updateTrajectoryBuffer(obj.getVehiclePose());
        end
        
        %------------------------------------------------------------------
        function updateTrajectoryBuffer(obj, pose)
            %updateTrajectoryBuffer Insert pose into trajectory buffer.
            
            validateattributes(pose, {'numeric'}, ...
                {'row','numel',3}, 'updateTrajectoryBuffer', 'pose');          
            
            if obj.TrajectoryIndex == 0
                obj.TrajectoryIndex = obj.TrajectoryIndex + 1;
                obj.TrajectoryBuffer(obj.TrajectoryIndex,:) = pose;
            end
            
            if norm(pose-obj.TrajectoryBuffer(obj.TrajectoryIndex,:)) > obj.TrajectoryUpdateTolerance
                obj.TrajectoryIndex = mod(obj.TrajectoryIndex, obj.TrajectoryBufferCapacity) + 1;
                obj.TrajectoryBuffer(obj.TrajectoryIndex,:) = pose;
            end
        end
        
        %------------------------------------------------------------------
        function updatePlot(obj, ~, ~)
            %updatePlot Update the figure window with current vehicle
            
            if ~isgraphics(obj.Axes)
                return
            end
            
            obj.plotVehicle();
            
            if obj.PlotTrajectory
                obj.plotTrajectory();
            end
            
            drawnow('limitrate');
        end
        
        %------------------------------------------------------------------
        function plotVehicle(obj)
            %plotvehicle Plot vehicle
            
            vehiclePose = obj.Vehicle.Pose;
                                    
            if isempty(obj.VehicleBodyHandle) || ~all(isgraphics(obj.VehicleBodyHandle))
                obj.VehicleBodyHandle = helperPlotVehicle(...
                    vehiclePose, obj.Vehicle.VehicleDimensions,...
                    obj.Vehicle.SteeringAngle);
            else
                obj.moveVehiclePlot(vehiclePose, obj.Vehicle.SteeringAngle);
            end
            
            uistack(obj.VehicleBodyHandle, 'top');
        end
        
        %------------------------------------------------------------------
        function moveVehiclePlot(obj, vehiclePose, steer)
            
            vehicleDims = obj.Vehicle.VehicleDimensions;
            
            vehicleShapes = helperVehiclePolyshape(vehiclePose, vehicleDims, steer);
            
            vehiclePolys = obj.VehicleBodyHandle;
            for n = 1 : numel(vehicleShapes)
                vehiclePolys(n).Shape = vehicleShapes(n);
            end
        end
        
        %------------------------------------------------------------------
        function plotTrajectory(obj)
            %plotTrajectory Plot vehicle trajectory
            
            if obj.TrajectoryIndex == 0
                return
            end
            
            trajectory = [obj.TrajectoryBuffer(obj.TrajectoryIndex+1:end,:);obj.TrajectoryBuffer(1:obj.TrajectoryIndex,:)];
            
            if isempty(obj.TrajectoryHandle) || ~isgraphics(obj.TrajectoryHandle)
                obj.TrajectoryHandle = plot(obj.Axes, trajectory(:,1), trajectory(:,2),'LineWidth',3);
                obj.TrajectoryHandle.Annotation.LegendInformation.IconDisplayStyle = 'off';
            else
                obj.TrajectoryHandle.XData = trajectory(:,1);
                obj.TrajectoryHandle.YData = trajectory(:,2);
            end
            
            uistack(obj.TrajectoryHandle, 'top');
        end
        
    end
end

