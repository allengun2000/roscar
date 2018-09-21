#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "goood/num.h"
#include <iostream>
#include "simple_ros.hpp"
#include <image_transport/image_transport.h>
using namespace std;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
void numCallback(const goood::num::ConstPtr& msg)
{
	
	s=msg->x;
	 cout<<s;
	
}

int main(int argc, char *argv[])
{
	 ros::init(argc, argv, "hello_ros") ;
	
    ros::NodeHandle n;
 
	ros::Subscriber sub_chatter = n.subscribe("chatter", 1000, chatterCallback);
	ros::Subscriber sub_num = n.subscribe("num", 1000, numCallback);

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher num_pub = n.advertise<goood::num>("num", 1000);
	ros::Rate loop_rate(10);


	int count = 0;
	while (ros::ok())
	{
		
		std_msgs::String msg;
        goood::num ss_msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		ss_msg.y=0;
		ss_msg.x=count;
		ROS_INFO("%s", msg.data.c_str());
		ros::spinOnce();
		chatter_pub.publish(msg);
        num_pub.publish(ss_msg);
		
		n.setParam("/ss_parm",s);
		loop_rate.sleep();
		++count;
	}

	
	return 0;
}
