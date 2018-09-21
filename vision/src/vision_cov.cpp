#include "vision_cov.hpp"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {

    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		Main_frame=cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
	
}
int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "image_listen");


	ros::NodeHandle n;


	// ros::Subscriber sub_chatter = n.subscribe("chatter", 1000, chatterCallback);

	// ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(100);

  // cv::namedWindow("Images");
	// cv::startWindowThread();
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_image = it.subscribe("usb_cam/image_raw", 1, imageCallback);
  image_transport::Publisher pub_image = it.advertise("camera/image", 1);
	ros::Publisher pic_pub = n.advertise<vision::image_cv>("pic_source", 1);
  sensor_msgs::ImagePtr msg_image;
	vision::image_cv pic_source;
	int count = 0;
while (ros::ok())
	{
		
		// std_msgs::String msg;

		// std::stringstream ss;
		// ss << "hello world " << count;
		// msg.data = ss.str();

		// ROS_INFO("%s", msg.data.c_str());
		
    msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Main_frame).toImageMsg();
		// chatter_pub.publish(msg);
		pub_image.publish(msg_image);
		ros::spinOnce();
		pic_source.weight=Main_frame.cols;//x
		pic_source.hight=Main_frame.rows;//y
		std_msgs::Int32MultiArray array;
		array.data.clear();
		pic_source.data.clear();
		 for(int i=0;i<Main_frame.rows;i++)
		 		for(int j=0;j<Main_frame.cols;j++)
				 		for(int k=0;k<3;k++){
		array.data.push_back(Main_frame.data[(i*Main_frame.cols*3)+(j*3)+k]);
		// pic_source.data[(i*Main_frame.cols*3)+(j*3)+1]=Main_frame.data[(i*Main_frame.cols*3)+(j*3)+1];
		// pic_source.data[(i*Main_frame.cols*3)+(j*3)+2]=Main_frame.data[(i*Main_frame.cols*3)+(j*3)+2];
				 }
				 
		pic_source.data=array.data;
		pic_pub.publish(pic_source);
		if(Main_frame.cols>2){cv::imshow("Images", Main_frame);}
    cv::waitKey(10);
		loop_rate.sleep();
		++count;
	}
cv::destroyWindow("Images");
	return 0;
}