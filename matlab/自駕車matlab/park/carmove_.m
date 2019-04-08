




costmap=vehicleCostmap(200,200,0.5);
% costmap.MapExtent=[0,75,0,50];

currentPose = [4 12 0]; 
data = load('routeplan.mat');
routePlan = data.routePlan;
% routePlan(2:end,:) = [];
vehicleDims  = vehicleDimensions;


scenario = drivingScenario;
egoCar = vehicle(scenario, 'ClassID', 1,'Position', [4 12 0]);

vehicleDims.FrontOverhang=egoCar.FrontOverhang;
vehicleDims.Height=egoCar.Height;
vehicleDims.Length=egoCar.Length;
vehicleDims.RearOverhang=egoCar.RearOverhang;
vehicleDims.Width=egoCar.Width;
vehicleDims.Wheelbase=egoCar.Wheelbase;

costmap.VehicleDimensions = vehicleDims;
maxSteeringAngle=35;

hold on
% helperPlotVehicle(currentPose, vehicleDims, 'DisplayName', 'Current Pose');
legend;

for n = 1 : height(routePlan)
    % Extract the goal waypoint
    vehiclePose = routePlan{n, 'EndPose'};
    
    % Plot the pose
    legendEntry = sprintf('Goal %i', n);
    helperPlotVehicle(vehiclePose, vehicleDims, 'DisplayName', legendEntry);
end
hold off


behavioralPlanner = HelperBehavioralPlanner(costmap, routePlan, vehicleDims, maxSteeringAngle)

motionPlanner = pathPlannerRRT(costmap, 'MinIterations', 1000, ...
    'ConnectionDistance', 10, 'MinTurningRadius', 20);

goalPose = routePlan{1, 'EndPose'};
refPath = plan(motionPlanner, currentPose, goalPose);

