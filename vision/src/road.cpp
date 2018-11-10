#include "ros/ros.h"
#include "vision/image_cv.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Bool.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace std;
using namespace cv;

cv::Mat  Main_frame;
cv::Mat  gray_image;
cv::Mat  img;
int img_h;
int img_w;
int world_x = 60;
int world_y = 800;
float heigh = 13.5;
float angle = 2.5;
int Kx = 1650;
int Ky = 1650;
std::vector<double> HSV;
std::vector<double> blackItem_pixel;
cv::Mat worldcoordinate(cv::Mat img){
int xsize = img.cols;
int ysize = img.rows;

	Mat ipm_img = Mat::zeros(cv::Size(world_x, world_y), CV_8UC3);
	
	for (uint16_t i = 0; i < world_y; i++) {
		uchar* ipm_data = ipm_img.ptr<uchar>(i);
		for (uint16_t j = 0; j < world_x; j++) {
			float xx = xsize / 2 + Kx * (j - (world_x / 2)) / ((world_y - i)*cos(angle*M_PI / 180) + heigh * sin(angle * M_PI / 180));
			float yy = ysize / 2 - Ky * ((world_y - i)*sin(angle * M_PI / 180) - heigh * cos(angle * M_PI / 180)) / ((world_y - i)*cos(angle * M_PI / 180) - heigh * sin(angle * M_PI / 180));
			if (xx > 0 && xx < xsize && yy>0 && yy < ysize) {
				ipm_data[j * 3 + 0] = img.at<Vec3b>(yy, xx)[0];
				ipm_data[j * 3 + 1] = img.at<Vec3b>(yy, xx)[1];
				ipm_data[j * 3 + 2] = img.at<Vec3b>(yy, xx)[2];
			}
		}
	}

	return ipm_img;
}


int Frame_area(int num,int range){
  if(num < 0) num = 0;
  else if(num >= range) num = range-1;
  return num;
}

cv::Mat dis_dot(cv::Mat img){
int y_start=0;
for(int y=0;y<=img.rows;y++){
if(!img.at<Vec3b>(y, img.cols/2)[0]){
	break;
}
y_start=y;
}
int black_angle=10;
int center_outer=100;
blackItem_pixel.clear();
         for(int angle = 0; angle < 180; angle = angle + black_angle){
              int angle_be = angle;

              if(angle_be >= 360) angle_be -= 360;

              double x_ = cos(angle_be*M_PI/180);
              double y_ = sin(angle_be*M_PI/180);
              for(int r = 0; r <= center_outer; r++){
                  int dis_x = x_*r;
                  int dis_y = y_*r;

                  int image_x = Frame_area(img.cols/2+dis_x,img.cols);
                  int image_y = Frame_area(y_start-dis_y,img.rows);

                  if( img.data[(image_y*img.cols + image_x)*3+0] == 255
                    &&img.data[(image_y*img.cols + image_x)*3+1] == 0
                    &&img.data[(image_y*img.cols + image_x)*3+2] == 0){
                      blackItem_pixel.push_back(hypot(dis_x,dis_y));
                      break;
                  }else{
                      img.data[(image_y*img.cols + image_x)*3+0] = 0;
                      img.data[(image_y*img.cols + image_x)*3+1] = 0;
                      img.data[(image_y*img.cols + image_x)*3+2] = 255;
                  }
                  if(r==center_outer){
                    blackItem_pixel.push_back(hypot(dis_x,dis_y));
                  }
              }
          }
	return img;
}
void ParmIsChangeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ros::NodeHandle n;
	if(msg->data){
    n.getParam("/golf/high",heigh);
	n.getParam("/golf/angle",angle);
	n.getParam("/golf/Kx",Kx);
	n.getParam("/golf/Ky",Ky);
	n.getParam("/golf/world_x",world_x);
	n.getParam("/golf/world_y",world_y);
	n.getParam("/golf/hsv",HSV);
	// cout<<"read parm"<<endl;
}else{
		system("rosparam dump ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
	}
}
void Parameter_getting(bool file){
	ros::NodeHandle n;
if(file){
    system("rosparam load ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
	n.getParam("/golf/high",heigh);
	n.getParam("/golf/angle",angle);
	n.getParam("/golf/Kx",Kx);
	n.getParam("/golf/Ky",Ky);
	n.getParam("/golf/world_x",world_x);
	n.getParam("/golf/world_y",world_y);
	n.getParam("/golf/hsv",HSV);
    cout<<"read the YAML file"<<endl;
  }else{
	n.setParam("/golf/high",13.5);
	n.setParam("/golf/angle",2.5);
	n.setParam("/golf/Kx",1650);
	n.setParam("/golf/Ky",1650);
	n.setParam("/golf/world_x",60);
	n.setParam("/golf/world_y",800);
	system("rosparam dump ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
  }
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
void calcLinesP(const Mat &input, std::vector<Vec4i> &lines) {
	Mat contours;
	// Canny(input, contours, 100, 150);
	// cv::imshow("canny", contours);
	lines.clear();
	HoughLinesP(input, lines, 1, CV_PI / 180, 50, 10, 70);
}

void drawLinesP(Mat &input, const std::vector<Vec4i> &lines) {
	for (int i = 0; i<lines.size(); i++) {
		line(input, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 3);
	}
}
int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "image_listen");


	ros::NodeHandle n;


	ros::Rate loop_rate(100);
    Parameter_getting(1);
    ros::Subscriber sub_parm = n.subscribe("/ParmIsChange", 10, ParmIsChangeCallback);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_image = it.subscribe("usb_cam/image_raw", 1, imageCallback,ros::VoidPtr(),image_transport::TransportHints("compressed"));
  std_msgs::Int32MultiArray disLine;
  ros::Publisher dis_pub = n.advertise<std_msgs::Int32MultiArray>("/roadDis", 1);
  sensor_msgs::ImagePtr msg_image;
	int count = 0;
while (ros::ok())
	{
		ros::spinOnce();
		while(Main_frame.cols<2){
			ros::spinOnce();
			if(!ros::ok()){
				return 1;
			}
		}
		img=Main_frame.clone();
		vector<Vec4i> linesP;
		
		Mat hsv;
		cvtColor(Main_frame, hsv, COLOR_BGR2HSV);
    	    Mat1b mask, mask2;
    inRange(hsv, Scalar(HSV[1], HSV[3], HSV[5]), Scalar(HSV[0], HSV[2], HSV[4]), mask);


		calcLinesP(mask, linesP);
		drawLinesP(img, linesP);
		
		Mat world(img,cv::Rect(0,160,640,195));
	    world=worldcoordinate(world);
		Mat worldtodot;
		worldtodot= dis_dot(world);
		// cvtColor(Main_frame,gray_image,CV_BGR2GRAY);



		cv::imshow("Images", Main_frame);
		cv::imshow("img", img);
		cv::imshow("line", mask);
		cv::imshow("word",world);
   	    cv::waitKey(1);
		
		////////////dis pub
		disLine.data.clear();
		disLine.data.push_back(1);
		disLine.data.push_back(2);
		dis_pub.publish(disLine);


		loop_rate.sleep();
	}
cv::destroyWindow("Images");
Main_frame.release();
img.release();
	return 0;
}