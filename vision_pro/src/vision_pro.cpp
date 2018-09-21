#include "vision_pro.hpp"
void chatterCallback(const vision::image_cv::ConstPtr& msg)
{
	img_w=msg->weight;
	img_h=msg->hight;
	Main_frame=cv::Mat::zeros(img_h,img_w,CV_8UC3);
	if(img_w>4){
	 for(int i=0;i<Main_frame.rows;i++)
		 		for(int j=0;j<Main_frame.cols;j++)
				 		for(int k=0;k<3;k++){
		Main_frame.data[(i*Main_frame.cols*3)+(j*3)+k]=msg->data[(i*Main_frame.cols*3)+(j*3)+k];
				 }
	}
}

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "image_get");


	ros::NodeHandle n;


	ros::Subscriber sub = n.subscribe("pic_source", 1000, chatterCallback);

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	cv::namedWindow("Images");
	ros::Rate loop_rate(1000);
	int count=0;
	while (ros::ok())
	{
		
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);
if(Main_frame.cols>2){cv::imshow("Images", Main_frame);}
cv::waitKey(10);
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}