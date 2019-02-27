#include "ros/ros.h"
#include "vision/image_cv.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
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
int black_angle=10;
int center_outer=400;
int hof=150;
int y_start=99999;
int y_end=0;
int car_vec=0;
std::vector<double> HSV;
std::vector<double> blackItem_pixel;
void drawLines(Mat &input);


cv::Mat worldcoordinate(cv::Mat img){
int xsize = img.cols;
int ysize = img.rows;
Mat ipm_img = Mat::zeros(cv::Size(world_x, world_y), CV_8UC3);
	y_start=99999;

	for (uint16_t i = 0; i < world_y; i++) {
		uchar* ipm_data = ipm_img.ptr<uchar>(i);
		for (uint16_t j = 0; j < world_x; j++) {
			float xx = xsize / 2 + Kx * (j - (world_x / 2)) / ((world_y - i)*cos(angle*M_PI / 180) + heigh * sin(angle * M_PI / 180));
			float yy = ysize / 2 - Ky * ((world_y - i)*sin(angle * M_PI / 180) - heigh * cos(angle * M_PI / 180)) / ((world_y - i)*cos(angle * M_PI / 180) - heigh * sin(angle * M_PI / 180));
			if (xx > 0 && xx < xsize && yy>0 && yy < ysize) {
				
				if(img.at<Vec3b>(yy, xx)[0]==255 && img.at<Vec3b>(yy, xx)[1]==0 && img.at<Vec3b>(yy, xx)[2]==0){
				ipm_data[j * 3 + 0] = 255;
				ipm_data[j * 3 + 1] = 255;
				ipm_data[j * 3 + 2] = 255;}
				else{
				// ipm_data[j * 3 + 0] = 0;
				// ipm_data[j * 3 + 1] = 0;
				// ipm_data[j * 3 + 2] = 0;
				// ipm_data[j * 3 + 0] = img.at<Vec3b>(yy, xx)[0];
				// ipm_data[j * 3 + 1] = img.at<Vec3b>(yy, xx)[1];
				// ipm_data[j * 3 + 2] = img.at<Vec3b>(yy, xx)[2];
				}
			}else{
				if (yy > ysize && i<y_start){
				 y_start=i;
				}
			}
		}
	}
Mat cat_roi(ipm_img,cv::Rect(0,y_start-131,world_x-1,world_y-y_start+131));
drawLines(cat_roi); //將畫面直線延伸

Mat imgroi = ipm_img(cv::Rect(0,y_start-131,world_x-1,world_y-y_start));
cat_roi.copyTo(imgroi,cat_roi);

	return ipm_img;
}


int Frame_area(int num,int range){
  if(num < 0) num = 0;
  else if(num >= range) num = range-1;
  return num;
}

cv::Mat dis_dot(cv::Mat img){


blackItem_pixel.clear();
         for(int angle = 0; angle < 181; angle = angle + black_angle){
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
                    &&img.data[(image_y*img.cols + image_x)*3+1] == 255
                    &&img.data[(image_y*img.cols + image_x)*3+2] == 255){
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
		//   cout<<(CV_PI/2+(car_vec-100)*0.02*(CV_PI/6))*180/CV_PI<<endl;
		  line(img,Point(img.cols/2,y_start),Point(img.cols/2-50*cos(CV_PI/2+(car_vec-100)*0.02*(CV_PI/6)),y_start-50*sin(CV_PI/2)+(car_vec-100)*0.02*(CV_PI/6)),Scalar(255,0,0),5);
		  
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
	n.getParam("/golf/black_angle",black_angle);
	n.getParam("/golf/center_outer",center_outer);
	n.getParam("/golf/hof",hof);
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
	n.getParam("/golf/black_angle",black_angle);
	n.getParam("/golf/center_outer",center_outer);
	n.getParam("/golf/hof",hof);
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
void carwheel_vec(const std_msgs::Float32::ConstPtr& msg){
car_vec=msg->data;

}
void drawLinesP(Mat &input,Mat &deaw_pic) {
	std::vector<Vec4i> lines;
	HoughLinesP(input, lines, 1, CV_PI / 180, 50, 10, 70);
	for (int i = 0; i<lines.size(); i++) {
		line(deaw_pic, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 3);
	}
	lines.clear();
}
void drawLines(Mat &input){ 
    std::vector<Vec2f> lines;
    Mat roi;
    // Canny(input,roi,50,200);
	cvtColor(input, roi, COLOR_BGR2GRAY);
		// cv::imshow("woqrsd",input);
   	    // cv::waitKey(1);
	std::vector<Vec4i> liness;
	std::vector<Point> linedott;
	Point xy_line_temp;
	HoughLinesP(roi, liness, 1, CV_PI / 180, hof, 10, 70);
	for (int i = 0; i<liness.size(); i++) {
		// line(input, Point(liness[i][0], liness[i][1]), Point(liness[i][2], liness[i][3]), Scalar(255, 0, 0), 3);
		
		int m=(liness[i][2]-liness[i][0] != 0)?(liness[i][3]-liness[i][1])/(liness[i][2]-liness[i][0]):99999;
		int b=-liness[i][2]*m+liness[i][3];
		
		for (int j=0;j<input.cols*3;j++){
			int x=j/3;
			int y=m*x+b;
			if (m==0)continue;
			if(y<input.rows-1 && y>0){
				xy_line_temp.x=x;
				xy_line_temp.y=y;
				linedott.push_back(xy_line_temp);
			}
		}
		if(linedott.size()==0)continue;
		
		line(input, Point(linedott[0].x, linedott[0].y), Point(linedott[linedott.size()-1].x, linedott[linedott.size()-1].y), Scalar(255, 255, 255), 3);
					// input.at<Vec3b>(y,x)[0]=255;//blue 通道
					// input.at<Vec3b>(y,x)[1]=255;//green 通道
					// input.at<Vec3b>(y,x)[2]=255;//red 通道
		linedott.clear();
		// cv::imshow("worsd",input);
		//  cv::waitKey(1);
	}
	liness.clear();
	
		
}

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "road");


	ros::NodeHandle n;


	ros::Rate loop_rate(100);
    Parameter_getting(1);
    ros::Subscriber sub_parm = n.subscribe("/ParmIsChange", 10, ParmIsChangeCallback);
	ros::Subscriber sub_car_wheel = n.subscribe("/car_wheel", 10, carwheel_vec);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_image = it.subscribe("usb_cam/image_raw", 1, imageCallback,ros::VoidPtr(),image_transport::TransportHints("compressed"));
//   image_transport::Publisher pub_image = it.advertise("camera/image", 1);
  std_msgs::Float64MultiArray disLine;
  ros::Publisher dis_pub = n.advertise<std_msgs::Float64MultiArray>("/roadDis", 1);
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

cv::imshow("Images", mask);
		drawLinesP(mask,img);
		
		
		Mat world;
		
	    world=worldcoordinate(img);
		// cv::imshow("line", world);
		Mat worldtodot;
		worldtodot= dis_dot(world);


		
		cv::imshow("img", worldtodot);
		// cv::imshow("line", mask);
		// cv::imshow("word",world);
   	    cv::waitKey(1);
		
		////////////dis pub
		disLine.data.clear();
		disLine.data=blackItem_pixel;
		dis_pub.publish(disLine);

		// msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		// pub_image.publish(msg_image);

		loop_rate.sleep();
	}
cv::destroyWindow("Images");
Main_frame.release();
img.release();
	return 0;
}