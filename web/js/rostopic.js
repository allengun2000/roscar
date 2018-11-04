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



