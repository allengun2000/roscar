#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "goood/num.h"
#include <iostream>
#include "simple_ros.hpp"
#include <image_transport/image_transport.h>
#include <fstream>
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
void ParmIsChangeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ros::NodeHandle n;
	if(msg->data){
    n.getParam("/golf/high",high);
	n.getParam("/golf/speedlimit",speedLimit);
	cout<<high<<endl;
	cout<<speedLimit[0]<<endl;
	cout<<speedLimit[1]<<endl;}else{
		system("rosparam dump ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
	}
}
void Parameter_getting(){
	ros::NodeHandle n;
if(ifstream("Parameter.yaml")){
    system("rosparam load ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
    cout<<"read the YAML file"<<endl;
  }else{
	  speedLimit.push_back(500);
	  speedLimit.push_back(20);
	n.setParam("/golf/high",500);
	n.setParam("/golf/speedlimit",speedLimit);
	system("rosparam dump ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
  }
}
int main(int argc, char *argv[])
{
	 ros::init(argc, argv, "hello_ros") ;
	ros::NodeHandle n;
	ros::Subscriber sub_chatter = n.subscribe("chatter", 1000, chatterCallback);
	ros::Subscriber sub_num = n.subscribe("num", 1000, numCallback);
	ros::Subscriber sub_parm = n.subscribe("/ParmIsChange", 1000, ParmIsChangeCallback);

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher num_pub = n.advertise<goood::num>("num", 1000);
	ros::Rate loop_rate(10);
	Parameter_getting();
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
