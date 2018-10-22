
sub = rossubscriber('/chatter');
motorpub = rospublisher('/MotorSpeed', 'std_msgs/Int32');
motormessage=rosmessage(motorpub);
s=0;
while(1)
a=sub.LatestMessage;
a.Data
motormessage.Data=s;
send(motorpub,motormessage)
s=s+1;
pause(1)
end
