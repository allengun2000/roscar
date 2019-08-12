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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace cv;
using namespace Eigen; 
typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
cv::Mat  Main_frame;
cv::Mat  image_roi;
cv::Mat  img;
Mat1b mask;Mat1b mask2;
Mat1b red_light,greenlight;
std::vector<double> HSV;
ros::Publisher point_pub;
ros::Publisher disdanger_pub;
VPointCloud alln_point_msg;  
float tfl_x=0,tfl_y=0;  

int world_x = 120;
int world_y = 1000;
float heigh =15.7;
float angle = 2;
int Kx = 640;
int Ky = 2000;
int y_start=99999;
bool light_sign=0;
// rosbag play -s 23 -u 9 -l tf2.bag 
// rosbag play -s 23 -u 9 --pause -l tf2.bag
int dis_model[21][2]={{51,276},
                      {49,271},
                      {47,269},
                      {45,268},
                      {43,266},
                      {42,260},
                      {40,258},
                      {29,230},
                      {28,227},
                      {27,223},
                      {25,211},
                      {24,207},
                      {22,195},
                      {21,187},
                      {20,178},
                      {19,173},
                      {17,158},
                      {15,131},
                      {14,108},
                      {12,75}};

int img_h_roi[2]={200,210};
void ParmIsChangeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ros::NodeHandle n;
	if(msg->data){
	n.getParam("/golf/hsv",HSV);
  n.getParam("/golf/high",heigh);
	n.getParam("/golf/angle",angle);
	n.getParam("/golf/Kx",Kx);
	n.getParam("/golf/Ky",Ky);
	n.getParam("/golf/world_x",world_x);
	n.getParam("/golf/world_y",world_y);
}else{
		system("rosparam dump ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
	}
}
void Parameter_getting(bool file){
	ros::NodeHandle n;
if(file){
    system("rosparam load ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
	n.getParam("/golf/hsv",HSV);
    cout<<"read the YAML file"<<endl;
  }else{
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
    image_roi=Main_frame;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void tf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
Mat rf_mat=Main_frame.clone();
  int redcount=0;int greencount=0;
      Mat hsv;
		cvtColor(rf_mat, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(0, 145, 138), Scalar(5, 255, 255), red_light);
    inRange(hsv, Scalar(42, 145, 69), Scalar(90, 255, 255), greenlight);
if(msg->data[0]==4){
    for(int i=0;i<21;i++){ //dismode總共21比資料
  if(  msg->data[1]<dis_model[i][0] && msg->data[1]>dis_model[i+1][0]){ //////算比較遠的紅綠燈
    for(int k=370;k<604;k++){
    for(int j=dis_model[i+1][1]-10;j<dis_model[i][1]+10;j++){
        // redcount=(red_light.at<uchar>(i,j)==255)?redcount+1:redcount;
        if(red_light.at<uchar>(j,k)==255){
          redcount++;
          red_light.at<uchar>(j,k)=125;}else{
            red_light.at<uchar>(j,k)=50;
          }
         if(greenlight.at<uchar>(j,k)==255){
          greencount++;
          red_light.at<uchar>(j,k)=200;}
      }
    }
    if(redcount>greencount){
      rectangle(rf_mat, Point(604,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(0,0,255), 1);
      light_sign=1;
    }else if(redcount==0 && greencount==0){
      rectangle(rf_mat, Point(604,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(255,0,0), 1);
      light_sign=0;
    }else{
      rectangle(rf_mat, Point(604,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(0,255,0), 1);
      light_sign=0;
    }
  }
  }

std_msgs::Float64MultiArray rad_light_msg; //[radlight 1 gogo0  dis disx]
rad_light_msg.data.resize(3);
rad_light_msg.data[1]=sqrt(pow(msg->data[1], 2) + pow(msg->data[2], 2));
rad_light_msg.data[2]=msg->data[1];
  //////////////////////////////////////////////////////////////////////////
      for(int i=0;i<21;i++){ //dismode總共21比資料
  if(  msg->data[4]<dis_model[i][0] && msg->data[4]>dis_model[i+1][0]){//////算比較近的紅綠燈
    for(int k=185;k<370;k++){ //////距離模左右寬度
    for(int j=dis_model[i+1][1]-10;j<dis_model[i][1]+10;j++){   //////距離模上下寬度
        if(red_light.at<uchar>(j,k)==255){ //紅燈幾個點
          redcount++;
          red_light.at<uchar>(j,k)=125;}else{
            red_light.at<uchar>(j,k)=50;
          }
         if(greenlight.at<uchar>(j,k)==255){//綠燈幾個點
          greencount++;
          red_light.at<uchar>(j,k)=200;}
      }
    }
    if(redcount>greencount){
      rectangle(rf_mat, Point(185,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(0,0,255), 1); //紅燈上色
      light_sign=1;
    }else if(redcount==0 && greencount==0){
      rectangle(rf_mat, Point(185,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(255,0,0), 1); //黃燈上色
      light_sign=0;
    }else{
      rectangle(rf_mat, Point(185,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(0,255,0), 1); //綠燈上色
      light_sign=0;
    }
  }
  }

      rad_light_msg.data[0]=light_sign;
    disdanger_pub.publish(rad_light_msg);
}

//////////////////////////////前後相反id
if(msg->data[0]==5){
    for(int i=0;i<21;i++){
  if(  msg->data[4]<dis_model[i][0] && msg->data[4]>dis_model[i+1][0])
rectangle(rf_mat, Point(604,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(0,0,255), 1);
  }
      for(int i=0;i<21;i++){
  if(  msg->data[1]<dis_model[i][0] && msg->data[1]>dis_model[i+1][0])
rectangle(rf_mat, Point(185,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(0,0,255), 1);
  }
}
  cv::imshow("line1", rf_mat);
  // cv::imshow("red", red_light);
}
int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "road");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
    Parameter_getting(1);
    ros::Subscriber sub_parm = n.subscribe("/ParmIsChange", 10, ParmIsChangeCallback);
  point_pub = n.advertise<VPointCloud>("allen_point", 1);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_image = it.subscribe("/camera452/image_raw", 1, imageCallback);
  disdanger_pub = n.advertise<std_msgs::Float64MultiArray>("Tf_light_info", 1);
	  //  ros::Subscriber velodyne_scan_ = n.subscribe("/points_raw", 10, pointCallback, ros::TransportHints().tcpNoDelay(true));
     ros::Subscriber sub1 = n.subscribe("/Can_I_GO_Ma", 10, tf_callback);
//   image_transport::Publisher pub_image = it.advertise("camera/image", 1);
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
		////////////////
		img=Main_frame.clone();
		vector<Vec4i> linesP;
		




    Mat hsv;
		cvtColor(img, hsv, COLOR_BGR2HSV);
          if(HSV[1]<HSV[0]){
    inRange(hsv, Scalar(HSV[1], HSV[3], HSV[5]), Scalar(HSV[0], HSV[2], HSV[4]), mask);}else{
    inRange(hsv, Scalar(0, HSV[3], HSV[5]), Scalar(HSV[0], HSV[2], HSV[4]), mask);
    inRange(hsv, Scalar(HSV[1], HSV[3], HSV[5]), Scalar(360, HSV[2], HSV[4]), mask2);
    mask=mask|mask2;
    }
    // inRange(hsv, Scalar(0, 145, 138), Scalar(5, 255, 255), red_light);
    // inRange(hsv, Scalar(42, 145, 69), Scalar(90, 255, 255), greenlight);
	// cv::imshow("Images", mask);
	// 		cv::imshow("red", red_light);
  //     cv::imshow("green", greenlight);
		// cv::imshow("line", Main_frame);
      // cv::imshow("line1", image_roi);
		// Mat worldtodot;red_light
		// worldtodot= dis_dot(world);
		// cv::imshow("line", mask);
		// cv::imshow("word",img);
   	    cv::waitKey(1);
		
		// msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		// pub_image.publish(msg_image);
//////

		// loop_rate.sleep();
	}
Main_frame.release();
img.release();
	return 0;
}



/*
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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <std_msgs/Float64MultiArray.h>
using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace cv;
using namespace Eigen; 
typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
cv::Mat  Main_frame;
cv::Mat  image_roi;
cv::Mat  img;
std::vector<double> HSV;
ros::Publisher point_pub;
VPointCloud alln_point_msg;  
float tfl_x=0,tfl_y=0;  

int world_x = 120;
int world_y = 1000;
float heigh =15.7;
float angle = 2;
int Kx = 640;
int Ky = 2000;
int y_start=99999;
// rosbag play -s 23 -u 9 -l tf2.bag 
// rosbag play -s 23 -u 9 --pause -l tf2.bag
int dis_model[14][2]={{29,230},
                      {28,227},
                      {27,223},
                      {25,211},
                      {24,207},
                      {22,195},
                      {21,187},
                      {20,178},
                      {19,173},
                      {17,158},
                      {15,131},
                      {14,108},
                      {12,75}};
int img_h_roi[2]={200,210};
void ParmIsChangeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ros::NodeHandle n;
	if(msg->data){
	n.getParam("/golf/hsv",HSV);
  n.getParam("/golf/high",heigh);
	n.getParam("/golf/angle",angle);
	n.getParam("/golf/Kx",Kx);
	n.getParam("/golf/Ky",Ky);
	n.getParam("/golf/world_x",world_x);
	n.getParam("/golf/world_y",world_y);
}else{
		system("rosparam dump ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
	}
}
void Parameter_getting(bool file){
	ros::NodeHandle n;
if(file){
    system("rosparam load ~/linux/catkin_ws/src/a_launch/config/Parameter.yaml /golf");
	n.getParam("/golf/hsv",HSV);
    cout<<"read the YAML file"<<endl;
  }else{
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
    Main_frame.copyTo(image_roi);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

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
				ipm_data[j * 3 + 0] = img.at<Vec3b>(yy, xx)[0];
				ipm_data[j * 3 + 1] = img.at<Vec3b>(yy, xx)[1];
				ipm_data[j * 3 + 2] = img.at<Vec3b>(yy, xx)[2];
				}
			}else{
				if (yy > ysize && i<y_start){
				 y_start=i;
				}
			}
		}
	}
  // Mat cat_roi(ipm_img,cv::Rect(0,0,world_x-1,world_y-y_start));
	return ipm_img;
}
void constructGridClouds(const VPointCloud::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count) {
 int grid_dim_=120;
 float m_per_cell_=0.5;
 float height_diff_threshold_=0.5;
 float height_max = 2.5;
  float min[grid_dim_][grid_dim_];
  float max[grid_dim_][grid_dim_];
  float num_obs[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      init[x][y] = false;
      num_obs[x][y] = 0;
    }
  }

  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_ / 2) + scan->points[i].x / m_per_cell_);
    int y = ((grid_dim_ / 2) + scan->points[i].y / m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = scan->points[i].z;
        max[x][y] = scan->points[i].z;
        num_obs[x][y] = 0;
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], scan->points[i].z);
        max[x][y] = MAX(max[x][y], scan->points[i].z);
      }
    }
  }

  // calculate number of obstacles in each cell
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_ / 2) + scan->points[i].x / m_per_cell_);
    int y = ((grid_dim_/ 2) + scan->points[i].y / m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      // if (((max[x][y] - min[x][y]) > height_diff_threshold_) &&
      //     ((max[x][y] - min[x][y]) < height_max)) {
        num_obs[x][y]++;
      // }
    }
  }
int temp_middle=999999999;

  // create clouds from grid
  double grid_offset = grid_dim_ / 2.0 * m_per_cell_;
  // for (int x = 0; x < grid_dim_; x++) {
  for (int x = grid_dim_ / 2; x < grid_dim_; x++) { 
    for (int y = grid_dim_ / 2-20; y < grid_dim_ / 2+20; y++) {
      if (num_obs[x][y] > 0) {
        if(abs(y-grid_dim_ / 2)<temp_middle){
        alln_point_msg.points[obs_count].x =
            -grid_offset + (x * m_per_cell_ + m_per_cell_ / 2.0);
        alln_point_msg.points[obs_count].y =
            -grid_offset + (y * m_per_cell_ + m_per_cell_ / 2.0);
        alln_point_msg.points[obs_count].z =0.3;
        if( alln_point_msg.points[obs_count].y!=0.25 &&  alln_point_msg.points[obs_count].x!=0.25){
          temp_middle=abs(y-grid_dim_/2);
        // obs_count++;
        }
        }
      }
    }
  }

  
  cout<<alln_point_msg.points[obs_count].x <<"ss  "<<alln_point_msg.points[obs_count].y<<endl;
  if(alln_point_msg.points[obs_count].x<dis_model[0][0] && alln_point_msg.points[obs_count].x>dis_model[13][0]){

//   for(int i=0;i<13;i++){
//   if(  alln_point_msg.points[obs_count].x<dis_model[i][0] && alln_point_msg.points[obs_count].x>dis_model[i+1][0])
// rectangle(image_roi, Point(604,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(0,0,255), 1);
// cv::imshow("line1", image_roi);
//   }
  }
    obs_count=1;
}

Eigen::Matrix4f CreateRoatationMatrix(Vector3f angle_before, Vector3f angle_after)
{
 
       angle_before.normalize();
       angle_after.normalize();
       float angle = acos(angle_before.dot(angle_after));
       Vector3f p_rotate = angle_before.cross(angle_after);
       p_rotate.normalize();
       Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
       rotationMatrix(0, 0)=cos(angle)+ p_rotate[0] * p_rotate[0] * (1 - cos(angle));
       rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));//这里跟公式比多了一个括号，但是看实验结果它是对的。
       rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
       rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
       rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
       rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
       rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
       rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
       rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));
       return rotationMatrix;
}

void pointCallback(const VPointCloud::ConstPtr& msg)
{

  if (point_pub.getNumSubscribers() == 0)
    return;

 pcl::PointCloud<pcl::PointXYZ>::Ptr transform_point(new pcl::PointCloud<pcl::PointXYZ>);
 
 /////////////////// 找出平面的垂直向量並且校正
// pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);
//     //添加在各字段上的比较算子  
//     //GT greater than
//     //EQ equal
//     //LT less than
//     //GE greater than or equal
//     //LE less than
//     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -5.0f));
//     range_cond->addComparison(cond_z1);
//     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, -3.0f));
//     range_cond->addComparison(cond_z2);
//     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y1(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 3.0f));
//     range_cond->addComparison(cond_y1);
//     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y2(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 6.0f));
//     range_cond->addComparison(cond_y2);
//     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x1(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.0f));
//     range_cond->addComparison(cond_x1);
//     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x2(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 20.0f));
//     range_cond->addComparison(cond_x2);
//     //创建滤波器并用条件定义对象初始化
//     pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//     condrem.setCondition(range_cond);
//     condrem.setInputCloud(msg);
//     //condrem.setKeepOrganized(true);//设置保持为结构点云   
//     // apply filter应用滤波器   
//     condrem.filter(*pointCloud_filter);
// // point_pub.publish(pointCloud_filter);

// pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
//     pcl::PointIndices::Ptr plane_inliers ( new pcl::PointIndices );
//     pcl::ModelCoefficients::Ptr plane_coefficients ( new pcl::ModelCoefficients );
//     plane_seg.setOptimizeCoefficients (true);
//     plane_seg.setModelType ( pcl::SACMODEL_PLANE );
//     plane_seg.setMethodType ( pcl::SAC_RANSAC );
//     plane_seg.setDistanceThreshold ( 0.05 );
//     plane_seg.setInputCloud ( msg );
//     plane_seg.segment (*plane_inliers, *plane_coefficients);//得到平面系数，进而得到平面法向量
// cout<< plane_coefficients->values[0] << " "
//            << plane_coefficients->values[1] << " "
//            << plane_coefficients->values[2] << " " << endl;
//////////////////////////////////////////////////////////////////////////
// pcl::transformPointCloud(*msg, *transform_point, CreateRoatationMatrix({0, 0, 0},{0.0f,0.0f,1.0f}));
                                                                         //    x          y       z 影像校正成水平

 pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
/////////濾掉地面訊號
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 3.3f));
    range_cond->addComparison(cond_z1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 5.5f));
    range_cond->addComparison(cond_z2);
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(msg);
    condrem.filter(*pointCloud_filter);

// point_pub.publish(pointCloud_filter);

  // pass along original time stamp and frame ID
  alln_point_msg.header.stamp = msg->header.stamp;
  alln_point_msg.header.frame_id = msg->header.frame_id;

  size_t npoints = msg->points.size();
  alln_point_msg.points.resize(npoints);

  size_t obs_count = 0;
  size_t empty_count = 0;
  // either return full point cloud or a discretized version
constructGridClouds(pointCloud_filter, npoints, obs_count, empty_count);
  alln_point_msg.points.resize(obs_count);


  if (point_pub.getNumSubscribers() > 0) {
    point_pub.publish(alln_point_msg);
  }


}

void tf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
cout<<msg->data[0]<<endl;
Mat rf_mat=image_roi;
if(msg->data[0]==5){
  msg->data[1];
    for(int i=0;i<13;i++){
  if(  msg->data[1]<dis_model[i][0] && msg->data[1]>dis_model[i+1][0])
rectangle(rf_mat, Point(604,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(0,0,255), 1);
  }
}
if(msg->data[0]==6){
  msg->data[1];
    for(int i=0;i<13;i++){
  if(  msg->data[1]<dis_model[i][0] && msg->data[1]>dis_model[i+1][0])
rectangle(rf_mat, Point(604,dis_model[i][1]+10), Point(370,dis_model[i+1][1]-10), Scalar(0,0,255), 1);
  }
}
  cv::imshow("line1", rf_mat);
// }

}
int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "road");


	ros::NodeHandle n;


	ros::Rate loop_rate(100);
    Parameter_getting(1);
    ros::Subscriber sub_parm = n.subscribe("/ParmIsChange", 10, ParmIsChangeCallback);
  point_pub = n.advertise<VPointCloud>("allen_point", 1);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_image = it.subscribe("/camera452/image_raw", 1, imageCallback);
	   ros::Subscriber velodyne_scan_ = n.subscribe("/points_raw", 10, pointCallback, ros::TransportHints().tcpNoDelay(true));
     ros::Subscriber sub1 = n.subscribe("/Can_I_GO_Ma", 10, tf_callback);
//   image_transport::Publisher pub_image = it.advertise("camera/image", 1);
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
		////////////////
		img=Main_frame.clone();
		vector<Vec4i> linesP;
		
		Mat hsv;
    		Mat imgroi = Main_frame(cv::Rect(0,0,800,300));
        // Mat imgroi_flip;
        // cv::flip(imgroi, imgroi_flip, 0); 
    	  //  cv::Mat tf_img=worldcoordinate(imgroi_flip);
		cvtColor(Main_frame, hsv, COLOR_BGR2HSV);
    	    Mat1b mask;
    inRange(hsv, Scalar(HSV[1], HSV[3], HSV[5]), Scalar(HSV[0], HSV[2], HSV[4]), mask);

	cv::imshow("Images", mask);



		// Mat world;
		
		cv::imshow("line", Main_frame);
      // cv::imshow("line1", image_roi);
		// Mat worldtodot;
		// worldtodot= dis_dot(world);
		// cv::imshow("line", mask);
		// cv::imshow("word",tf_img);
   	    cv::waitKey(1);
		
		// msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		// pub_image.publish(msg_image);
//////

		loop_rate.sleep();
	}
// cv::destroyWindow("Images");
Main_frame.release();
img.release();
	return 0;
}*/