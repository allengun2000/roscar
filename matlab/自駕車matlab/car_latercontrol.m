

% rosinit('127.0.0.1');
sub = rossubscriber('/Do_you_like_ice_cream','std_msgs/Float64MultiArray');
% sub1 = rossubscriber('/duino_velocity','std_msgs/Float64');
sub1 = rossubscriber('/current_velocity','geometry_msgs/TwistStamped');
pub = rospublisher('/cmd','geometry_msgs/Pose2D');
stree_msg = rosmessage(pub);
keepGoing = true;

while keepGoing
way_point_info = receive(sub,1);
currpoint=[way_point_info.Data(1),way_point_info.Data(2),radtodeg(way_point_info.Data(3))];
refpoint=[way_point_info.Data(4),way_point_info.Data(5),radtodeg(way_point_info.Data(3))+way_point_info.Data(8)];
speed=receive(sub1,1);
speed_now=speed.Twist.Linear.X*5;

wheel_angle=lateralControllerStanley(refpoint,currpoint,speed_now,'Wheelbase',2.15,'PositionGain',10);
% if wheel_angle>27
%     wheel_angle=27;
% end
% 
% if wheel_angle<-27
%     wheel_angle=-27;
% end
    % start_go = rosparam("get",'/statego');
start_go=1;
if start_go==1
stree_msg.X=3;
stree_msg.Theta=-mapfun(wheel_angle,-35,35,-719,719);
stree_msg.Y=3;
send(pub,stree_msg);
else
stree_msg.X=0;
stree_msg.Theta=0;
stree_msg.Y=1;
send(pub,stree_msg);
end

end

function output = mapfun(value,fromLow,fromHigh,toLow,toHigh)
narginchk(5,5)
nargoutchk(0,1)
output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end
% msg.X=15;   msg.Theta=stree_angle; msg.Y=1;
% send(pub,msg);
% rosshutdown;