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
#include <pcl/filters/conditional_removal.h>
using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace cv;
typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
cv::Mat  Main_frame;
cv::Mat  gray_image;
cv::Mat  img;
std::vector<double> HSV;
ros::Publisher point_pub;
	VPointCloud alln_point_msg;  
void ParmIsChangeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ros::NodeHandle n;
	if(msg->data){
	n.getParam("/golf/hsv",HSV);
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
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
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
    int y = ((grid_dim_ / 2) + scan->points[i].y / m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if (((max[x][y] - min[x][y]) > height_diff_threshold_) &&
          ((max[x][y] - min[x][y]) < height_max)) {
        num_obs[x][y]++;
      }
    }
  }

  // create clouds from grid
  double grid_offset = grid_dim_ / 2.0 * m_per_cell_;
  // for (int x = 0; x < grid_dim_; x++) {
  for (int x = grid_dim_ / 2; x < grid_dim_; x++) { 
    for (int y = 0; y < grid_dim_; y++) {
      if (num_obs[x][y] > 0) {

        alln_point_msg.points[obs_count].x =
            -grid_offset + (x * m_per_cell_ + m_per_cell_ / 2.0);
        alln_point_msg.points[obs_count].y =
            -grid_offset + (y * m_per_cell_ + m_per_cell_ / 2.0);
        alln_point_msg.points[obs_count].z = max[x][y];
        obs_count++;
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_raw, int flag)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_filter(new pcl::PointCloud<pcl::PointXYZ>);//创建点云对象，用以存储滤波后点云
    cout << "条件滤波" << endl;
    //创建条件限定下的滤波器
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);
    //添加在各字段上的比较算子  
    //GT greater than
    //EQ equal
    //LT less than
    //GE greater than or equal
    //LE less than
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -0.75f));
    range_cond->addComparison(cond_z1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, -0.68f));
    range_cond->addComparison(cond_z2);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y1(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 0.0f));
    range_cond->addComparison(cond_y1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y2(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 10.0f));
    range_cond->addComparison(cond_y2);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x1(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -8.0f));
    range_cond->addComparison(cond_x1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x2(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 8.0f));
    range_cond->addComparison(cond_x2);
    //创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(pointCloud_raw);
    //condrem.setKeepOrganized(true);//设置保持为结构点云   
    // apply filter应用滤波器   
    condrem.filter(*pointCloud_filter);
    return pointCloud_filter;
}

void pointCallback(const VPointCloud::ConstPtr& msg)
{

  if (point_pub.getNumSubscribers() == 0)
    return;

  // pass along original time stamp and frame ID
  alln_point_msg.header.stamp = msg->header.stamp;
  alln_point_msg.header.frame_id = msg->header.frame_id;

  size_t npoints = msg->points.size();
  alln_point_msg.points.resize(npoints);

  size_t obs_count = 0;
  size_t empty_count = 0;
  // either return full point cloud or a discretized version
constructGridClouds(msg, npoints, obs_count, empty_count);
  alln_point_msg.points.resize(obs_count);


  if (point_pub.getNumSubscribers() > 0) {
    point_pub.publish(alln_point_msg);
  }


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
  image_transport::Subscriber sub_image = it.subscribe("usb_cam/image_raw", 1, imageCallback,ros::VoidPtr(),image_transport::TransportHints("compressed"));
	   ros::Subscriber velodyne_scan_ = n.subscribe("/velodyne_points", 10, pointCallback, ros::TransportHints().tcpNoDelay(true));
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
		cvtColor(Main_frame, hsv, COLOR_BGR2HSV);
    	    Mat1b mask;
    inRange(hsv, Scalar(HSV[1], HSV[3], HSV[5]), Scalar(HSV[0], HSV[2], HSV[4]), mask);
	cv::imshow("Images", mask);

		


		// Mat world;
		
	    // world=worldcoordinate(img);
		cv::imshow("line", Main_frame);
		// Mat worldtodot;
		// worldtodot= dis_dot(world);

		// cv::imshow("line", mask);
		// cv::imshow("word",world);
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
}