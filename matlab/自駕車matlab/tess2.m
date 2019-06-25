
scenario = drivingScenario;
scenario.SampleTime = 0.01;
 
% 
% roadCenters = [220 117 0;
%     -62.9 118 0];
% marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
%     laneMarking('Solid')
%     laneMarking('DoubleSolid', 'Color', [1 1 0])
%     laneMarking('Solid')
%     laneMarking('Solid')];
% laneSpecification = lanespec(4, 'Marking', marking);
% road(scenario, roadCenters, 'Lanes', laneSpecification);
% 
% roadCenters = [217.85 205.85 0;
%     -65.05 206.85 0];
% marking = laneMarking('Solid');
% laneSpecification = lanespec(2, 'Marking', marking);
% road(scenario, roadCenters, 'Lanes', laneSpecification);
% 
% roadCenters = [222.45 306.85 0;
%     68.2 308.1 0];
% marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
%     laneMarking('Solid')
%     laneMarking('DoubleSolid', 'Color', [1 1 0])
%     laneMarking('Solid')
%     laneMarking('Solid')];
% laneSpecification = lanespec(4, 'Marking', marking);
% road(scenario, roadCenters, 'Lanes', laneSpecification);
% 
% roadCenters = [-33.9 308.3 0;
%     -68.35 309.05 0];
% marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
%     laneMarking('Solid')
%     laneMarking('DoubleSolid', 'Color', [1 1 0])
%     laneMarking('Solid')
%     laneMarking('Solid')];
% laneSpecification = lanespec(4, 'Marking', marking);
% road(scenario, roadCenters, 'Lanes', laneSpecification);
% 
% roadCenters = [29.1 360.1 0;
%     68.6 310.5 0;
%     24.7 255.1 0;
%     -33.8 309.5 0;
%     29.1 360.1 0];
% laneSpecification = lanespec(2, 'Width', 7.2);
% road(scenario, roadCenters, 'Lanes', laneSpecification);
% 
% roadCenters = [13 256 0;
%     14 -5 0];
% marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
%     laneMarking('Solid')
%     laneMarking('DoubleSolid', 'Color', [1 1 0])
%     laneMarking('Solid')
%     laneMarking('Solid')];
% laneSpecification = lanespec(4, 'Marking', marking);
% road(scenario, roadCenters, 'Lanes', laneSpecification);
% 
% roadCenters = [142.4 398.5 0;
%     139.4 -4.4 0];
% marking = laneMarking('Solid');
% laneSpecification = lanespec(2, 'Marking', marking);
% road(scenario, roadCenters, 'Lanes', laneSpecification);
% roadCenters = [-30 -1.28 0;
%     176.2 2.8 0;
%     189.6 4.2 0;
%     210.4 9.6 0;
%     218.6 39.2 0;
%     218.6 55.2 0;
%     219.2 360.7 0;
%     218 373.1 0;
%     208.4 391.3 0;
%     184.8 398.4 0;
%     173.4 399.4 0;
%     -24 397.6 0;
%     -34.8 397.6 0;
%     -55 391.3 0;
%     -59.9 372.5 0;
%     -60.8 357.2 0;
%     -58 37.3 0;
%     -57.6 32.1 0;
%     -52.7 11.5 0;
%     -36.9 -0.1 0;
%     -30 -1.28 0;
%     -30 -1.28 0];
% marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
%     laneMarking('Solid')
%     laneMarking('DoubleSolid', 'Color', [1 1 0])
%     laneMarking('Solid')
%     laneMarking('Solid')];
% laneSpecification = lanespec(4, 'Marking', marking);
% road(scenario, roadCenters, 'Lanes', laneSpecification);

roadCenters = [-35 20 0; -20 -20 0; 0 0 0; 20 20 0; 35 -20 0 ; 100 0 0 ];

lm = [laneMarking('Solid','Color','w'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Solid','Color','w')];
ls = lanespec(3,'Marking',lm);
road(scenario, roadCenters,'Lanes',ls);


%% 

egoCar = vehicle(scenario, 'ClassID', 1,'Width', 0.45);
% trajectory(egoCar, roadCenters(1:end,1:end) , 100); % On right lane

% Add a car in front of the ego vehicle
leadCar = vehicle(scenario, 'ClassID', 2,'Position',[100 -20 0]);
person_ = actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [30.7 3.2 0], ...
    'RCSPattern', [-8 -8;-8 -8]);
% path(leadCar, roadCenters(2:end,2:end) , 101); % On right lane
% 
% % Add a car that travels at 35 m/s along the road and passes the ego vehicle
% passingCar = vehicle(scenario, 'ClassID', 1);
% waypoints = [0 -1.8; 50 1.8; 100 1.8; 250 21.8; 400 32.2; 500 38.2];
% path(passingCar, waypoints, 35);
% 
% % Add a car behind the ego vehicle
% chaseCar = vehicle(scenario, 'ClassID', 1);
% path(chaseCar, [25 0; roadCenters(2:end,1:2)] - [0 1.8], 25); % On right lane

sensors = cell(8,1);
% Front-facing long-range radar sensor at the center of the front bumper of the car.
sensors{1} = radarDetectionGenerator('SensorIndex', 1, 'Height', 0.2, 'MaxRange', 174, ... 
    'SensorLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0], 'FieldOfView', [20, 5]);

% Rear-facing long-range radar sensor at the center of the rear bumper of the car.
sensors{2} = radarDetectionGenerator('SensorIndex', 2, 'Height', 0.2, 'Yaw', 180, ...
    'SensorLocation', [-egoCar.RearOverhang, 0], 'MaxRange', 174, 'FieldOfView', [20, 5]);

% Rear-left-facing short-range radar sensor at the left rear wheel well of the car.
sensors{3} = radarDetectionGenerator('SensorIndex', 3, 'Height', 0.2, 'Yaw', 120, ...
    'SensorLocation', [0, egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Rear-right-facing short-range radar sensor at the right rear wheel well of the car.
sensors{4} = radarDetectionGenerator('SensorIndex', 4, 'Height', 0.2, 'Yaw', -120, ...
    'SensorLocation', [0, -egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Front-left-facing short-range radar sensor at the left front wheel well of the car.
sensors{5} = radarDetectionGenerator('SensorIndex', 5, 'Height', 0.2, 'Yaw', 60, ...
    'SensorLocation', [egoCar.Wheelbase, egoCar.Width/2], 'MaxRange', 30, ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Front-right-facing short-range radar sensor at the right front wheel well of the car.
sensors{6} = radarDetectionGenerator('SensorIndex', 6, 'Height', 0.2, 'Yaw', -60, ...
    'SensorLocation', [egoCar.Wheelbase, -egoCar.Width/2], 'MaxRange', 30, ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Front-facing camera located at front windshield.
sensors{7} = visionDetectionGenerator('SensorIndex', 7, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], 'Height', 1.1);

% Rear-facing camera located at rear windshield.
sensors{8} = visionDetectionGenerator('SensorIndex', 8, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.2*egoCar.Wheelbase 0], 'Height', 1.1, 'Yaw', 180);
%% Create a Tracker
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationParameters', [4 5]);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

% Create the display and return a handle to the bird's-eye plot
BEP = createDemoDisplay(egoCar, sensors);

%% Simulate the Scenario
load('data_vehicle');
x0_ego=egoCar.Position(1);
y0_ego=egoCar.Position(2);
yaw0_ego=egoCar.Yaw;

toSnap = true;
% rosinit
sub = rossubscriber('/car_info','geometry_msgs/Twist');
pub = rospublisher('/cmd','geometry_msgs/Pose2D');
pub1 = rospublisher('/state_reward','std_msgs/Float64MultiArray');
msg = rosmessage(pub);
state_reward=rosmessage(pub1);
msg.X=0;msg.Theta=0.03;
send(pub,msg);
% sim('sim_car_wheel')
            egoCar.Position=[-35 20 0];
            egoCar.Yaw=-100;
while advance(scenario)   
% while 1 
    % Get the scenario time
    time = scenario.SimulationTime;
    
    msg2 = receive(sub,1);
    dt=0.1;
    
output_xy=[msg2.Angular.X , msg2.Angular.Y]'; 
output_Yaw= degtorad(-msg2.Linear.Z); 
egoCar.Yaw=egoCar.Yaw+msg2.Angular.Z*dt; 
output_xy=[cos(degtorad(egoCar.Yaw)) -sin(degtorad(egoCar.Yaw));sin(degtorad(egoCar.Yaw)) cos(degtorad(egoCar.Yaw))]* ... 
([cos(output_Yaw) -sin(output_Yaw);sin(output_Yaw) cos(output_Yaw)]*output_xy); 
egoCar.Velocity=[output_xy(1) , output_xy(2) ,0]; 
egoCar.Position=egoCar.Position+egoCar.Velocity*dt; 
%%%% 
% egoCar.Position(1) = msg2.Linear.X; 
% egoCar.Position(2) = msg2.Linear.Y; 
% egoCar.Yaw= msg2.Linear.Z;


     
     
    ta = targetPoses(egoCar);
    lanes = laneBoundaries(egoCar, 'XDistance', 100);
    % Simulate the sensors
%     detections = {};
%     laneDetections   = [];
%     isValidTime = false(1,8);
%     for i = 1:8
%         [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
%         if numValidDets
%             for j = 1:numValidDets
%                 % Vision detections do not report SNR. The tracker requires
%                 % that they have the same object attributes as the radar
%                 % detections. This adds the SNR object attribute to vision
%                 % detections and sets it to a NaN.
%                 if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
%                     sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
%                 end
%             end
%             detections = [detections; sensorDets]; %#ok<AGROW>
%         end
%     end

        reward=0;
    
        laneBound = laneBoundaries(egoCar,'XDistance',0:5:30,'LocationType','Center','AllBoundaries',true);
        offset=laneBound(1).LateralOffset;
        offset=abs(offset-5.4);
        reward=reward-offset;
        
       if isnan(reward)
         reward=-10
         egoCar.Position=[-35 20 0];
         egoCar.Yaw=-100;
         continue
       end
        
        
       refpoint=[(laneBound(2).Coordinates(2,1)+laneBound(3).Coordinates(2,1))/2,...
           (laneBound(2).Coordinates(2,2)+laneBound(3).Coordinates(2,2))/2 ,laneBound(2).HeadingAngle]
       corpoint=[0,0,0];
       
       chang_point=[([cos(degtorad(egoCar.Yaw-90)) -sin(degtorad(egoCar.Yaw-90));sin(degtorad(egoCar.Yaw-90)) cos(degtorad(egoCar.Yaw-90))]*[-refpoint(2);refpoint(1)])',0];
      person_.Position =[chang_point(1),chang_point(2),0]+egoCar.Position;
       stree_angle=degtorad(lateralControllerStanley(refpoint,corpoint,30))
        
        msg.X=15;msg.Theta=stree_angle;
        send(pub,msg);
        
        
        send(pub1,state_reward);

        
        
        
        state_reward.Data=[offset ,reward,0 ,egoCar.Position];
    % Update the tracker if there are new detections
%     if any(isValidTime)
%         vehicleLength = sensors{1}.ActorProfiles.Length;
%         detectionClusters = clusterDetections(detections, vehicleLength);
%         confirmedTracks = updateTracks(tracker, detectionClusters, time);
%         
%         % Update bird's-eye plot
%         updateBEP(BEP, egoCar, detections, confirmedTracks, positionSelector, velocitySelector,laneBound);
%     end
    
    % Snap a figure for the document when the car passes the ego vehicle
    if ta(1).Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
    end
end

%% Supporting Functions

function filter = initSimDemoFilter(detection)
% Use a 2-D constant velocity model to initialize a trackingKF filter.
% The state vector is [x;vx;y;vy]
% The detection measurement vector is [x;y;vx;vy]
% As a result, the measurement model is H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
filter = trackingKF('MotionModel', '2D Constant Velocity', ...
    'State', H' * detection.Measurement, ...
    'MeasurementModel', H, ...
    'StateCovariance', H' * detection.MeasurementNoise * H, ...
    'MeasurementNoise', detection.MeasurementNoise);
end


function detectionClusters = clusterDetections(detections, vehicleSize)
N = numel(detections);
distances = zeros(N);
for i = 1:N
    for j = i+1:N
        if detections{i}.SensorIndex == detections{j}.SensorIndex
            distances(i,j) = norm(detections{i}.Measurement(1:2) - detections{j}.Measurement(1:2));
        else
            distances(i,j) = inf;
        end
    end
end
leftToCheck = 1:N;
i = 0;
detectionClusters = cell(N,1);
while ~isempty(leftToCheck)    
    % Remove the detections that are in the same cluster as the one under
    % consideration
    underConsideration = leftToCheck(1);
    clusterInds = (distances(underConsideration, leftToCheck) < vehicleSize);
    detInds = leftToCheck(clusterInds);
    clusterDets = [detections{detInds}];
    clusterMeas = [clusterDets.Measurement];
    meas = mean(clusterMeas, 2);
    meas2D = [meas(1:2);meas(4:5)];
    i = i + 1;
    detectionClusters{i} = detections{detInds(1)};
    detectionClusters{i}.Measurement = meas2D;
    leftToCheck(clusterInds) = [];    
end
detectionClusters(i+1:end) = [];

% Since the detections are now for clusters, modify the noise to represent
% that they are of the whole car
for i = 1:numel(detectionClusters)
    measNoise(1:2,1:2) = vehicleSize^2 * eye(2);
    measNoise(3:4,3:4) = eye(2) * 100 * vehicleSize^2;
    detectionClusters{i}.MeasurementNoise = measNoise;
end
end


function BEP = createDemoDisplay(egoCar, sensors)
    % Make a figure
    hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top    

    % Add a car plot that follows the ego vehicle from behind
    hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot);

    % Add a car plot that follows the ego vehicle from a top view
    hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
    hCarPlot = axes(hTopViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);
    
    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');
    
    % Create bird's-eye plot for the ego car and sensor coverage
    hBEVPlot = axes(hBEVPanel);
    frontBackLim = 60;
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);
    
    % Plot the coverage areas for radars
    for i = 1:6
        cap = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
        plotCoverageArea(cap, sensors{i}.SensorLocation,...
            sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
    end
    
    % Plot the coverage areas for vision sensors
    for i = 7:8
        cap = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
        plotCoverageArea(cap, sensors{i}.SensorLocation,...
            sensors{i}.MaxRange, sensors{i}.Yaw, 45);
    end
    
    % Create a vision detection plotter put it in a struct for future use
    detectionPlotter(BEP, 'DisplayName','vision', 'MarkerEdgeColor','blue', 'Marker','^');
    
    % Combine all radar detections into one entry and store it for later update
    detectionPlotter(BEP, 'DisplayName','radar', 'MarkerEdgeColor','red');
    
    % Add road borders to plot
    laneMarkingPlotter(BEP, 'DisplayName','lane markings');
    
    % Add the tracks to the bird's-eye plot. Show last 10 track updates.
    trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);
    
    laneBoundaryPlotter(BEP, 'DisplayName','lane boundariesR', 'Color',[.0 .9 0]);
    laneBoundaryPlotter(BEP, 'DisplayName','lane boundariesL', 'Color',[.0 .9 .0]);

    
    
    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-40 40]);
    
    % Add an outline plotter for ground truth
    outlinePlotter(BEP, 'Tag', 'Ground truth');
end


function updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel ,lb)
    % Update road boundaries and their display
    [lmv, lmf] = laneMarkingVertices(egoCar);
    plotLaneMarking(findPlotter(BEP,'DisplayName','lane markings'),lmv,lmf);
    
    % update ground truth data
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);
    
    % Prepare and update detections display
    N = numel(detections);
    detPos = zeros(N,2);    
    isRadar = true(N,1);
    for i = 1:N
        detPos(i,:) = detections{i}.Measurement(1:2)';
        if detections{i}.SensorIndex > 6 % Vision detections            
            isRadar(i) = false;
        end        
    end

    plotDetection(findPlotter(BEP,'DisplayName','vision'), detPos(~isRadar,:));
    plotDetection(findPlotter(BEP,'DisplayName','radar'), detPos(isRadar,:));    
%     
    plotLaneBoundary(findPlotter(BEP,'DisplayName','lane boundariesR'), {lb(1).Coordinates});
    plotLaneBoundary(findPlotter(BEP,'DisplayName','lane boundariesL'), {lb(2).Coordinates});

    % Prepare and update tracks display
    trackIDs = {confirmedTracks.TrackID};
    labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
    [tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
    tracksVel = getTrackVelocities(confirmedTracks, vsel);
    plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, tracksVel, tracksCov, labels);
end