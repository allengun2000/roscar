close
scenario = drivingScenario;
scenario.SampleTime = 0.01;
% roadCenters = [-35 20 0; -20 -20 0; 0 0 0; 20 20 0; 35 -20 0 ; 100 0 0 ];
roadCenters = [-35 20 0;
    -20 -20 0;
    0 0 0;
    20 20 0;
    35 -20 0;
    40.3 -60 0;
    35 -100 0;
    30 -140 0;
    40 -180.3 0;
    -0.2 -220 0;
    -50.1 -179.5 0;
    -27.3 -121.5 0;
    -35 20 0];
lm = [laneMarking('Solid','Color','w'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Solid','Color','w')];
ls = lanespec(3,'Marking',lm,'Width', [1.95 2.7 1.95]);
road(scenario, roadCenters,'Lanes',ls);
egoCar = vehicle(scenario, 'ClassID', 1,'Width', 1.15,'Length', 2.40);
leadCar = vehicle(scenario, 'ClassID', 2,'Position',[100 -20 0]);
person_ = actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [30.7 3.2 0], ...
    'RCSPattern', [-8 -8;-8 -8]);
BEP = createDemoDisplay(egoCar);

load('golf_car');
x0_ego=egoCar.Position(1);
y0_ego=egoCar.Position(2);
yaw0_ego=egoCar.Yaw;

toSnap = true;
sub = rossubscriber('/car_info','geometry_msgs/Twist');
sub_cmd = rossubscriber('/cmd','geometry_msgs/Pose2D');
pub = rospublisher('/cmd','geometry_msgs/Pose2D');
pub1 = rospublisher('/state_reward','std_msgs/Float64MultiArray');
msg = rosmessage(pub);
state_reward=rosmessage(pub1);
msg.X=0;msg.Theta=0.03;
send(pub,msg);
egoCar.Position=[40.3 -60 0];
egoCar.Yaw=-92.875;
wheel_angle=0;
while advance(scenario)   
%     time = scenario.SimulationTime;
    
    car_point_msg = receive(sub,1);
    dt=0.1;
    
output_xy=[car_point_msg.Angular.X , car_point_msg.Angular.Y]'; 
output_Yaw= degtorad(-car_point_msg.Linear.Z); 
egoCar.Yaw=egoCar.Yaw+car_point_msg.Angular.Z*dt; 
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


        reward=0;
    
        laneBound = laneBoundaries(egoCar,'XDistance',0:5:30,'LocationType','Center','AllBoundaries',true);
        offset=laneBound(1).LateralOffset;
        
        offset=abs(offset-1.95-1.35);
%         reward=reward-offset;
       if isnan(laneBound(2).Coordinates(3,2))
%          egoCar.Position=[person_.Position(1:2) 0];
%          egoCar.Yaw=egoCar.Yaw+headangle;
% egoCar.Position=[-35 20 0];
% egoCar.Yaw=-92.875;
egoCar.Position=[40.3 -60 0];
egoCar.Yaw=-92.875;
         wheel_angle=0;
         reward=-5;
       state_reward.Data=[pos_e,radtodeg(angle_e),radtodeg(cmd.Theta),ref_angle,reward,0]; %
%        state_reward.Data=[pos_e,radtodeg(angle_e),wheel_angle,ref_angle,reward,1]; %
        send(pub1,state_reward);
         continue
       end
       headangle=radtodeg(atan(((laneBound(2).Coordinates(3,2)+laneBound(3).Coordinates(3,2))/2-(laneBound(2).Coordinates(2,2)+laneBound(3).Coordinates(2,2))/2)/ ...
           ((laneBound(2).Coordinates(3,1)+laneBound(3).Coordinates(3,1))/2-(laneBound(2).Coordinates(2,1)+laneBound(3).Coordinates(2,1))/2)));

       corpoint=[0,0,0];
       refpoint=[(laneBound(2).Coordinates(2,1)+laneBound(3).Coordinates(2,1))/2,...
           (laneBound(2).Coordinates(2,2)+laneBound(3).Coordinates(2,2))/2 ,headangle];

       
       chang_point=[([cos(degtorad(egoCar.Yaw-90)) -sin(degtorad(egoCar.Yaw-90));sin(degtorad(egoCar.Yaw-90)) cos(degtorad(egoCar.Yaw-90))]*[-refpoint(2);refpoint(1)])',0];
      person_.Position =[chang_point(1),chang_point(2),0]+egoCar.Position;
      [ref_angle,pos_e ,angle_e]=later_car(refpoint, corpoint, 10, 1 ,2.5 ,2.15,28);
        
       if wheel_angle <ref_angle
       wheel_angle=wheel_angle+0.3;
       else
           wheel_angle=wheel_angle-0.3;
       end
       cmd = receive(sub_cmd,1);
       stree_angle=mapfun(wheel_angle,-35,35,-720,720);
       
%         if offset<0.775 && pos_e<-0.5
%             reward=1-abs(radtodeg(angle_e))/100;
%         else
           reward=-abs(pos_e)/5-abs(radtodeg(angle_e))/80;
%         end
       state_reward.Data=[pos_e,radtodeg(angle_e),radtodeg(cmd.Theta),ref_angle,reward,0]; %
%        state_reward.Data=[pos_e,radtodeg(angle_e),wheel_angle,ref_angle,reward,0]; %

        msg.X=10;msg.Theta=degtorad(wheel_angle);
%         send(pub,msg);
        send(pub1,state_reward);


end

%% Supporting Functions
function output = mapfun(value,fromLow,fromHigh,toLow,toHigh)
narginchk(5,5)
nargoutchk(0,1)
output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end
function BEP = createDemoDisplay(egoCar)
    % Make a figure
    hFigure = figure('Position', [0, 0, 640, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top    

    % Add a car plot that follows the ego vehicle from behind
    hCarViewPanel = uipanel(hFigure, 'Position', [0 0 1 0.5], 'Title', 'Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot);

    % Add a car plot that follows the ego vehicle from a top view
    hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 1 0.5], 'Title', 'Top View');
    hCarPlot = axes(hTopViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);
BEP=1;
end