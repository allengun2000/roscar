#include "vision_cov.hpp"
//rosrun usb_cam usb_cam_node _video_device:=/dev/video0

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
void chatterCallback(const vision::image_cv::ConstPtr& msg)
{
    
int	img_w=msg->weight;
int	img_h=msg->hight;
    Main_frame.release();
	Main_frame=cv::Mat::zeros(img_h,img_w,CV_8UC3);
	if(img_w>100){
	 for(int i=0;i<Main_frame.rows;i++)
		 		for(int j=0;j<Main_frame.cols;j++)
				 		for(int k=0;k<3;k++){
		Main_frame.data[(i*Main_frame.cols*3)+(j*3)+k]=msg->data[(i*Main_frame.cols*3)+(j*3)+k];
				 }
	}
    
}
int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "image_listen");


	ros::NodeHandle n;


	ros::Rate loop_rate(100);

  // cv::namedWindow("Images");
	// cv::startWindowThread();
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_image = it.subscribe("usb_cam/image_raw", 1, imageCallback);
	ros::Subscriber sub = n.subscribe("pic_source", 10, chatterCallback);
	// image_transport::Subscriber sub_image = it.subscribe("/camera2/usb_cam2/image_raw", 1, imageCallback,ros::VoidPtr(),image_transport::TransportHints("compressed"));
  image_transport::Publisher pub_image = it.advertise("car_line/image_raw", 1);
	ros::Publisher pic_pub = n.advertise<vision::image_cv>("bag_picture", 1);
  sensor_msgs::ImagePtr msg_image;
	vision::image_cv pic_source;
	int count = 0;
while (ros::ok())
	{
		ros::spinOnce();
		while(Main_frame.cols<2){
			ros::spinOnce();
		
		}
		cvtColor(Main_frame,gray_image,CV_BGR2GRAY);
    msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Main_frame).toImageMsg();
		pub_image.publish(msg_image);

		pic_source.weight=Main_frame.cols;//x
		pic_source.hight=Main_frame.rows;//y
		std_msgs::Int32MultiArray array;
		array.data.clear();
		pic_source.data.clear();
		 for(int i=0;i<Main_frame.rows;i++)
		 		for(int j=0;j<Main_frame.cols;j++)
				 		for(int k=0;k<3;k++){
		array.data.push_back(Main_frame.data[(i*Main_frame.cols*3)+(j*3)+k]);
				 }
				 
		pic_source.data=array.data;
		pic_pub.publish(pic_source);
		// if(gray_image.cols>2){cv::imshow("Images", gray_image);}
		// if(Main_frame.cols>2){cv::imshow("Images", Main_frame);}
    cv::waitKey(1);
		loop_rate.sleep();
		++count;
	}
cv::destroyWindow("Images");
	return 0;
}