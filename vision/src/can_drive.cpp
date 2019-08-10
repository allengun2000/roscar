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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>
using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace cv;
using namespace Eigen; 
typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
pcl::visualization::CloudViewer viewer("Cloud Viewer");
cv::Mat  Main_frame;
cv::Mat  image_roi;
cv::Mat  img;
std::vector<double> HSV;
ros::Publisher point_pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr alln_point_msg(new pcl::PointCloud<pcl::PointXYZI>); 
VPointCloud::Ptr obs_msg(new VPointCloud); 
// rosbag play -s 23 -u 9 -l tf2.bag 
// rosbag play -s 23 -u 9 --pause -l tf2.bag

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


  // memset(&init, 0, grid_dim_*grid_dim_);

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
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      if (num_obs[x][y] > 0) {
        obs_msg->points[obs_count].x =
            -grid_offset + (x * m_per_cell_ + m_per_cell_ / 2.0);
        obs_msg->points[obs_count].y =
            -grid_offset + (y * m_per_cell_ + m_per_cell_ / 2.0);
        obs_msg->points[obs_count].z = height_diff_threshold_;
        obs_count++;
      }
    }
  }
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
 /////////////////// 找出平面的垂直向量並且校正
pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.1f));
    range_cond->addComparison(cond_z1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 3.0f));
    range_cond->addComparison(cond_z2);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y1(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -6.0f));
    range_cond->addComparison(cond_y1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y2(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 6.0f));
    range_cond->addComparison(cond_y2);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x1(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.2f));
    range_cond->addComparison(cond_x1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x2(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 20.0f));
    range_cond->addComparison(cond_x2);
    //创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(msg);
    // apply filter应用滤波器   
    condrem.filter(*pointCloud_filter);

//////////////////////////////////////////////////////////////////////////



  size_t npoints = msg->points.size();
  obs_msg->points.resize(npoints);

  size_t obs_count = 0;
  size_t empty_count = 0;
  // either return full point cloud or a discretized version
constructGridClouds(pointCloud_filter, npoints, obs_count, empty_count);
  obs_msg->points.resize(obs_count);
// viewer.showCloud(alln_point_msg);


  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (obs_msg); //创建点云索引向量，用于存储实际的点云信息
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.75); //距離周圍距離 單位m
  ec.setMinClusterSize (1);//最少點數
  ec.setMaxClusterSize (100);//最大點數
  ec.setSearchMethod (tree);//设置点云的搜索机制
  ec.setInputCloud (obs_msg);
  ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

  int j = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_show (new pcl::PointCloud<pcl::PointXYZI>);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    double x1=99999,x2=-99999,y1=99999,y2=-99999;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
    pcl::PointXYZI point_color;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (obs_msg->points[*pit]);
      point_color.x=obs_msg->points[*pit].x;point_color.y=obs_msg->points[*pit].y;point_color.z=obs_msg->points[*pit].z;
      point_color.intensity=j;
        x1 = MIN(x1, point_color.x);
        x2 = MAX(x2, point_color.x);
        y1 = MIN(y1, point_color.y);
        y2 = MAX(y2, point_color.y);
        point_show->points.push_back(point_color);
      }
      point_color.x=x1;point_color.y=y1;
      point_show->points.push_back(point_color);
      point_color.x=x2;point_color.y=y1;
      point_show->points.push_back(point_color);
      point_color.x=x1;point_color.y=y2;
      point_show->points.push_back(point_color);
      point_color.x=x2;point_color.y=y2;
      point_show->points.push_back(point_color);

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
    // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    j++;
    // cout<<j<<endl;
  }
  // viewer.showCloud(point_show);

    pcl::PointCloud<pcl::PointXYZI>::Ptr road_cango_model (new pcl::PointCloud<pcl::PointXYZI>);
double theta=-M_PI/3;
double car_x=0;
double car_y=0;
double car_weight=0.5;
double car_height=0.5;
double z=0;
    pcl::PointXYZI point_road;
for(double x=0;x<20;x=x+0.5){
  for(double y=-6;y<7;y=y+0.5){
    z=0;
    for(int i=0;i< point_show->points.size();i++){
      car_x=point_show->points[i].x;
      car_y=point_show->points[i].y;
      z=z+exp(-(((x-car_x)*cos(theta)-(y-car_y)*sin(theta))/car_weight)*(((x-car_x)*cos(theta)-(y-car_y)*sin(theta))/car_weight))* 
        exp(-(((y-car_y)*cos(theta)+(x-car_x)*sin(theta))/car_height)*(((y-car_y)*cos(theta)+(x-car_x)*sin(theta))/car_height));
    }
    point_road.x=x;point_road.y=y;point_road.z=z;point_road.intensity=z;
    road_cango_model->points.push_back(point_road);
  }
}

    // point_road.x=3;point_road.y=0;point_road.z=z;point_road.intensity=z;
    // point_show->points.push_back(point_road);

  // pass along original time stamp and frame ID
  point_show->header.stamp = msg->header.stamp;
  point_show->header.frame_id = msg->header.frame_id;
  if (point_pub.getNumSubscribers() > 0) {
    point_pub.publish(point_show);
  }

}


int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "road_can_go");

	ros::NodeHandle n;
	ros::Rate loop_rate(100);
  point_pub = n.advertise<VPointCloud>("allen_point", 1);
	ros::Subscriber velodyne_scan_ = n.subscribe("/points_raw", 10, pointCallback, ros::TransportHints().tcpNoDelay(true));
  sensor_msgs::ImagePtr msg_image;
	int count = 0;
while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}