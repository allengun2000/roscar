////////////////////////////////////////////////////////////////////
var cmdVelTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/turtle1/cmd_vel',
    messageType : 'geometry_msgs/Twist'
});
function pubMessage(a) {
var twist = new ROSLIB.Message({
    linear : {
        x : 0.0,
        y : 0.0,
        z : 0.0
    },
    angular : {
        x : 0.0,
        y : 0.0,
        z : 0.0
    }
});
    var linearX = 0.0;
    var angularZ = 0.0;

switch(a){
case 0:
    linearX = 0 + Number(document.getElementById('linearXText').value);
    angularZ = 0 + Number(document.getElementById('angularZText').value);
case 1:
    linearX = 2;
    angularZ = 0; 
break;
case 2:
    linearX = -2;
    angularZ = 0; 
break;
case 3:
    linearX = 0;
    angularZ = 2; 
break;
case 4:
    linearX = 0;
    angularZ = -2; 
break;
case 5:
    linearX = 0;
    angularZ = 0; 
break;
}
    twist.linear.x = linearX;
    twist.angular.z = angularZ;
    cmdVelTopic.publish(twist);
return
}
/////////////////////////////////////////////////////////////////////////////////
var wheelTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/car_wheel',
    messageType : 'std_msgs/Float32'
});
function pubwheel(a) {
var wheel_msg = new ROSLIB.Message({
   data:0.0
});
switch(a){
case 0:
wheel_msg.data=parseFloat(50);
break;
case 1:
wheel_msg.data=parseFloat(80);
break;
case 2:
wheel_msg.data=parseFloat(100);
break;
case 3:
wheel_msg.data=parseFloat(120);
break;
case 4:
wheel_msg.data=parseFloat(150);
break;
}
  wheelTopic.publish(wheel_msg);
return
}
///////
var speedTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/car_speed',
    messageType : 'std_msgs/Int32'
});
function pubspeed(a) {
var speed_msg = new ROSLIB.Message({
   data:0
});
switch(a){
case 0:
speed_msg.data=Number(189);
break;
case 1:
speed_msg.data=Number(193);
break;
}
  speedTopic.publish(speed_msg);
return
}
/////////////////////////////////////////////////////////////////////////////////
var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/MotorFB',
  messageType : 'std_msgs/Float32'
});
var listener1 = new ROSLIB.Topic({
  ros : ros,
  name : '/speedFB',
  messageType : 'std_msgs/Int32'
});
var listener2 = new ROSLIB.Topic({
  ros : ros,
  name : '/MotorA',
  messageType : 'std_msgs/Float32'
});
listener.subscribe(function(message) {
	document.getElementById("MotorFb").innerText= message.data;
	});
listener1.subscribe(function(message) {
	document.getElementById("SpeedFB").innerText= message.data;
	});
listener2.subscribe(function(message) {
	document.getElementById("MotorA").innerText= message.data;
	});
//////////////////////////////////////////////////////////////
var PuuTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/Puu_control',
    messageType : 'std_msgs/Int32'
});
function pubPUU(value_puu) {
var puu = new ROSLIB.Message({
        data:Number(value_puu)
    });
PuuTopic.publish(puu);
}
////////////////////////////////////////////
var ParmTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/ParmIsChange',
    messageType : 'std_msgs/Bool'
});
function pubParmTopic(value) {
var change = new ROSLIB.Message({
        data:value
    });
ParmTopic.publish(change);
}

////////////////////////////////////////////////



