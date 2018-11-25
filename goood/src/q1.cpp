#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <sstream>
using namespace std;

 void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	cout<<msg->data;
}
 void callback_int(const std_msgs::Int32::ConstPtr& msg)
{
	cout<<msg->data<<endl;
}
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "test");


	ros::NodeHandle n;


	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Publisher int_pp = n.advertise<std_msgs::Int32>("/fuckyou", 1000);

	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::Subscriber sub1 = n.subscribe("/fuckyou", 1000, callback_int);
	ros::Rate loop_rate(10);

	
	int count = 0;
	while (ros::ok())
	{
		
		std_msgs::String msg;

		std_msgs::Int32 num;
		num.data=count;
		int_pp.publish(num);

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

	
		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}