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
	// int i=0;
	//  for(std::vector<int>::const_iterator it = array->data.begin() ; it != array->data.end(); ++it)
    // {   
	// 	mcssl_send2motor(i,*it);
    // }
	}
}

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "image_get");


	ros::NodeHandle n;


	ros::Subscriber sub = n.subscribe("pic_source", 1000, chatterCallback);

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher pub1 = n.advertise<vision_pro::line_inform>("line_info", 1000);
	vision_pro::line_inform line_msg;
	cv::namedWindow("Imagess");
	ros::Rate loop_rate(1000);
	int count=0;
	while (ros::ok())
	{
		cv::Mat img_ = cv::imread("/home/iris/iriscar.png");
		if(img_.cols>2){cv::imshow("Imagess", img_);}
		cv::waitKey(10);
// line_msg.state=1;
// line_msg.dot_x.clear();
// line_msg.dot_x.push_back(1);
// line_msg.dot_x.push_back(2);
// line_msg.dot_x.push_back(10);

// line_msg.dot_y.clear();
// line_msg.dot_y.push_back(line_msg.dot_x[2]);
// line_msg.dot_y.push_back(line_msg.dot_x[1]);
// line_msg.dot_y.push_back(line_msg.dot_x[0]);

// line_msg.dot_num=line_msg.dot_x.size();
// pub1.publish(line_msg);
// if(Main_frame.cols>2){cv::imshow("Images", Main_frame);}

		// ros::spinOnce();

		// loop_rate.sleep();
		++count;
	}
	cv::Mat img_cordinate(510,240, CV_8UC3);
	// for(int j=0;j<img_.rows;j++){
	// 	for(int i=0;j<img_.cols;i++){

	// 	}
	// }
	return 0;
}