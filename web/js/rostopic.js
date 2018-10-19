// These lines create a topic object as defined by roslibjs
var cmdVelTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/turtle1/cmd_vel',
    messageType : 'geometry_msgs/Twist'
});

// These lines create a message that conforms to the structure of the Twist defined in our ROS installation
// It initalizes all properties to zero. They will be set to appropriate values before we publish this message.
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

/* This function:
 - retrieves numeric values from the text boxes
 - assigns these values to the appropriate values in the twist message
 - publishes the message to the cmd_vel topic.
 */
function pubMessage(a) {
    /**
    Set the appropriate values on the twist message object according to values in text boxes
    It seems that turtlesim only uses the x property of the linear object 
    and the z property of the angular object
    **/
    var linearX = 0.0;
    var angularZ = 0.0;

    // get values from text input fields. Note for simplicity we are not validating.

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
    // Set the appropriate values on the message object
    twist.linear.x = linearX;
    twist.angular.z = angularZ;
    // Publish the message 
    cmdVelTopic.publish(twist);
return
}


