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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <ctime>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
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
pcl::PointCloud<pcl::PointXYZ>::Ptr back_ground(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::CloudViewer viewer("Cloud Viewer");
int world_x = 120;
int world_y = 1000;
float heigh =15.7;
float angle = 2;
int Kx = 640;
int Ky = 2000;
int y_start=99999;



pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>); //濾除 roi以外的點雲
pcl::ConditionalRemoval<pcl::PointXYZ> condrem; //空間濾波器
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; //建構樹木

// rosbag play -s 23 -u 9 -l tf2.bag 
// rosbag play -s 23 -u 9 --pause -l tf2.bag
void constructGridClouds(const VPointCloud::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count) {
 int grid_dim_=120;
 float m_per_cell_=0.1;
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

  // create clouds from grid
  double grid_offset = grid_dim_ / 2.0 * m_per_cell_;
  for (int x =0; x < grid_dim_; x++) { 
    for (int y =0; y < grid_dim_ ; y++) {
      if (num_obs[x][y] > 0) {
        alln_point_msg.points[obs_count].x =
            -grid_offset + (x * m_per_cell_ + m_per_cell_ / 2.0);
        alln_point_msg.points[obs_count].y =
            -grid_offset + (y * m_per_cell_ + m_per_cell_ / 2.0);
        // alln_point_msg.points[obs_count].z =max[x][y];
        alln_point_msg.points[obs_count].z=0.3;
        obs_count++;
      }
    }
  }
}
void two_point_filter(const VPointCloud::ConstPtr &scan,const VPointCloud::ConstPtr &inputpoint,
                                    unsigned npoints,unsigned nin_points, size_t &obs_count,
                                    size_t &empty_count) {
 int grid_dim_=120;
 float m_per_cell_=0.1;
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
        init[x][y] = true;
      } 
    }
  }
  for (unsigned i = 0; i < nin_points; ++i) {
    int x = ((grid_dim_ / 2) + inputpoint->points[i].x / m_per_cell_);
    int y = ((grid_dim_ / 2) + inputpoint->points[i].y / m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        num_obs[x][y]++;
      } 
    }
  }
  for (int x =1; x < grid_dim_-1; x++) { 
    for (int y =1; y < grid_dim_-1 ; y++) {
      if(num_obs[x][y]>0 && num_obs[x-1][y]==0&& num_obs[x+1][y]==0&& num_obs[x][y-1]==0&& num_obs[x][y+1]==0)
      num_obs[x][y]=0;
    }
  }

  // create clouds from grid
  double grid_offset = grid_dim_ / 2.0 * m_per_cell_;
  for (int x =0; x < grid_dim_; x++) { 
    for (int y =0; y < grid_dim_ ; y++) {
      if (num_obs[x][y] > 0) {
        alln_point_msg.points[obs_count].x =
            -grid_offset + (x * m_per_cell_ + m_per_cell_ / 2.0);
        alln_point_msg.points[obs_count].y =
            -grid_offset + (y * m_per_cell_ + m_per_cell_ / 2.0);
        // alln_point_msg.points[obs_count].z =max[x][y];
        alln_point_msg.points[obs_count].z=0.3;
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
bool customCondition(const pcl::PointXYZ& seedPoint, const pcl::PointXYZ& candidatePoint, float squaredDistance)
{
    // Do whatever you want here.做你想做的条件的筛选
    if (candidatePoint.y < seedPoint.y)  //如果候选点的Y的值小于种子点的Y值（就是之前被选择为聚类的点），则不满足条件，返回假
        return false;

    return true;
}

void pointCallback(const VPointCloud::ConstPtr& msg)
{

///////////////////////roi以外的點濾除
    condrem.setInputCloud(msg); 
    condrem.filter(*cloud);

  ////////////////將點做採樣///////////////////
  pcl::VoxelGrid<pcl::PointXYZ> vg; //体素栅格下采样对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.1f, 0.1f, 0.1f); //设置采样的体素大小
  vg.filter (*cloud_filtered);  //执行采样保存数据

    // viewer.showCloud(cloud_filtered);
// //////////////////////找出圓 濾除雜訊
  pcl::SACSegmentation<pcl::PointXYZ> seg;//创建分割对象
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  seg.setOptimizeCoefficients (true);  //设置对估计的模型参数进行优化处理
  seg.setModelType (pcl::SACMODEL_CIRCLE2D);//设置分割模型类别
  // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);//设置用哪个随机参数估计方法
  
  seg.setMaxIterations (100);  //设置最大迭代次数
  seg.setDistanceThreshold (0.02);    //设置判断是否为模型内点的距离阈值

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      return ;
    }
    //移去平面局内点，提取剩余点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;   //创建点云提取对象
    extract.setInputCloud (cloud_filtered);    //设置输入点云
    extract.setIndices (inliers);   //设置分割后的内点为需要提取的点集
    extract.setNegative (false); //设置提取内点而非外点
    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);   //提取输出存储到cloud_plane
    // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    // *cloud_filtered = *cloud_f;
//////////////////////////////////////////這裡完成一次跌帶
/////////////////////////////////////////////把距離小的濾雜訊
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //创建滤波器
    outrem.setInputCloud(cloud_f);    //设置输入点云
    outrem.setRadiusSearch(0.2);     //设置半径为0.2的范围内找临近点
    outrem.setMinNeighborsInRadius(4); //设置查询点的邻域点集数小于2的删除
    // apply filter
    outrem.filter (*cloud_filtered);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
         
  }
  



  pcl::PointCloud<pcl::PointXYZ>::Ptr point_mis (new pcl::PointCloud<pcl::PointXYZ>);
////////////////////////////////////////和背景影像剪
  for(int i=0;i<cloud_filtered->points.size();i++){
		int K = 1;                               //设搜索一个点，即：K为1.
		std::vector<int> pointIdxNKNSearch(K);   //设置一个类似数组的东西，（我对于C++中vector的理解直接把它当成了数组），pointIdxNKNSearch（K）的长度为K，用于存放点索引
		std::vector<float> pointNKNSquaredDistance(K);//同样一个长度为K的数组pointNKNSquaredDistance，用于存放距离的平方
 
		if (kdtree.nearestKSearch(cloud_filtered->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)//前面定义了kdtree作为搜索对象，然后就可以用它来调用系统的nearestKSearch函数了，注意后面的参数，分别是上面已经定义好的。
		{
			if(pointNKNSquaredDistance[0]>0.01){ //r距離大於1公分
        point_mis->points.push_back(cloud_filtered->points[i]);
      }
    }
  }


  // viewer.showCloud(point_mis);


    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //创建滤波器
    outrem.setInputCloud(point_mis);    //设置输入点云
    outrem.setRadiusSearch(0.2);     //设置半径为0.2的范围内找临近点
    outrem.setMinNeighborsInRadius(4); //设置查询点的邻域点集数小于2的删除
    // apply filter
    outrem.filter (*cloud_filtered);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
     viewer.showCloud(cloud_filtered);


//////////////////////////////////聚類開始
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered); //创建点云索引向量，用于存储实际的点云信息
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.4); //距離周圍距離 單位m
  ec.setMinClusterSize (4);//最少點數
  ec.setMaxClusterSize (100);//最大點數
  ec.setSearchMethod (tree);//设置点云的搜索机制
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

  int j = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_show (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
      pcl::PointXYZI point_color;
      point_color.x=cloud_filtered->points[*pit].x;point_color.y=cloud_filtered->points[*pit].y;point_color.z=cloud_filtered->points[*pit].z;
      point_color.intensity=j;
      point_show->points.push_back(point_color);
      }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    j++;
  }
    // viewer.showCloud(point_show);
   cout<<"number people"<<j<<endl;
  point_show->header.frame_id ="velodyne";
  point_pub.publish(point_show);
}



int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "road");


	ros::NodeHandle n;


	ros::Rate loop_rate(100);
  point_pub = n.advertise<VPointCloud>("allen_point", 1);
  ros::Subscriber velodyne_scan_ = n.subscribe("/velodyne_points", 10, pointCallback, ros::TransportHints().tcpNoDelay(true));
	int count = 0;

    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y1(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -5.0f));
    range_cond->addComparison(cond_y1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y2(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 5.0f));
    range_cond->addComparison(cond_y2);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x1(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -5.0f));
    range_cond->addComparison(cond_x1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x2(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 5.0f));
    range_cond->addComparison(cond_x2);

    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -0.9f));
    range_cond->addComparison(cond_z1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, -0.3f));
    range_cond->addComparison(cond_z2);

    //创建滤波器并用条件定义对象初始化

    condrem.setCondition(range_cond);

  pcl::io::loadPCDFile ("/home/allen/logs/pcd_libss.pcd", *back_ground);

    condrem.setInputCloud(back_ground); 
    condrem.filter(*back_ground);
		kdtree.setInputCloud(back_ground);

  pcl::io::loadPCDFile ("/home/allen/logs/pcd_libPEO.pcd", *cloud);
 
while (ros::ok())
	{


		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

//////
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //创建滤波器
    
    // outrem.setInputCloud(cloud);    //设置输入点云
    // outrem.setRadiusSearch(0.8);     //设置半径为0.8的范围内找临近点
    // outrem.setMinNeighborsInRadius (2); //设置查询点的邻域点集数小于2的删除
    // // apply filter
    // outrem.filter (*cloud_filtered);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存

///////////////////////////serch point
  // srand (time (NULL));

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // // Generate pointcloud data
  // cloud->width = 1000;
  // cloud->height = 1;
  // cloud->points.resize (cloud->width * cloud->height);

  // for (size_t i = 0; i < cloud->points.size (); ++i)
  // {
  //   cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  //   cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  //   cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  // }

  // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  // kdtree.setInputCloud (cloud);

  // pcl::PointXYZ searchPoint;

  // searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  // searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  // searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  // // K nearest neighbor search

  // int K = 10;

  // std::vector<int> pointIdxNKNSearch(K);
  // std::vector<float> pointNKNSquaredDistance(K);

  // std::cout << "K nearest neighbor search at (" << searchPoint.x 
  //           << " " << searchPoint.y 
  //           << " " << searchPoint.z
  //           << ") with K=" << K << std::endl;

  // if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  // {
  //   for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
  //     std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
  //               << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
  //               << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
  //               << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  // }

  // // Neighbors within radius search

  // std::vector<int> pointIdxRadiusSearch;
  // std::vector<float> pointRadiusSquaredDistance;

  // float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  // std::cout << "Neighbors within radius search at (" << searchPoint.x 
  //           << " " << searchPoint.y 
  //           << " " << searchPoint.z
  //           << ") with radius=" << radius << std::endl;


  // if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  // {
  //   for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
  //     std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
  //               << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
  //               << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
  //               << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  // }


  // return 0;



///////////////////////////////////
// vector<int>Lab;
// int NoObj = partition(LIDAR_cooridate, Lab);
// private:bool predicate(Pt P1, Pt P2)
// 	{
// 		double distant = Math::Sqrt(Math::Pow((P1.x - P2.x), 2) + Math::Pow((P1.y - P2.y), 2));
// 		return  distant <= PartitionValue;
// 	}
// 	private:int partition(cv::vector<Pt>& _vec, cv::vector<int>& labels)
// 	{
// 		int i, j, N = _vec.size();
// 		const Pt* vec = &_vec[0];

// 		const int PARENT = 0;
// 		const int RANK = 1;

// 		cv::vector<int> _nodes(N * 2);
// 		int(*nodes)[2] = (int(*)[2])&_nodes[0];

// 		for (i = 0; i < N; i++)
// 		{
// 			nodes[i][PARENT] = -1;
// 			nodes[i][RANK] = 0;
// 		}
// 		for (i = 0; i < N; i++)
// 		{
// 			int root = i;

// 			// find root
// 			while (nodes[root][PARENT] >= 0)
// 				root = nodes[root][PARENT];

// 			for (j = 0; j < N; j++)
// 			{
// 				if (i == j || !predicate(vec[i], vec[j]))
// 					continue;
// 				int root2 = j;

// 				while (nodes[root2][PARENT] >= 0)
// 					root2 = nodes[root2][PARENT];

// 				if (root2 != root)
// 				{
// 					// unite both trees
// 					int rank = nodes[root][RANK], rank2 = nodes[root2][RANK];
// 					if (rank > rank2)
// 						nodes[root2][PARENT] = root;
// 					else
// 					{
// 						nodes[root][PARENT] = root2;
// 						nodes[root2][RANK] += rank == rank2;
// 						root = root2;
// 					}
// 					//assert(nodes[root][PARENT] < 0);

// 					int k = j, parent;

// 					// compress the path from node2 to root
// 					while ((parent = nodes[k][PARENT]) >= 0)
// 					{
// 						nodes[k][PARENT] = root;
// 						k = parent;
// 					}

// 					// compress the path from node to root
// 					k = i;
// 					while ((parent = nodes[k][PARENT]) >= 0)
// 					{
// 						nodes[k][PARENT] = root;
// 						k = parent;
// 					}
// 				}
// 			}
// 		}
// 		for (unsigned int i = 0; i < N; i++)
// 			labels.push_back(0);
// 		int nclasses = 0;

// 		for (i = 0; i < N; i++)
// 		{
// 			int root = i;
// 			while (nodes[root][PARENT] >= 0)
// 				root = nodes[root][PARENT];
// 			if (nodes[root][RANK] >= 0)
// 				nodes[root][RANK] = ~nclasses++;
// 			labels[i] = ~nodes[root][RANK];
// 		}
// 		return nclasses;
// 	}

///////////////////////////////////////////////////////////////////////
//   if (point_pub.getNumSubscribers() == 0)
//     return;

//  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_point(new pcl::PointCloud<pcl::PointXYZ>);
 
//  /////////////////// 找出平面的垂直向量並且校正
// // pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);
// //     //添加在各字段上的比较算子  
// //     //GT greater than
// //     //EQ equal
// //     //LT less than
// //     //GE greater than or equal
// //     //LE less than
// //     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -5.0f));
// //     range_cond->addComparison(cond_z1);
// //     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, -3.0f));
// //     range_cond->addComparison(cond_z2);
// //     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y1(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 3.0f));
// //     range_cond->addComparison(cond_y1);
// //     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_y2(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 6.0f));
// //     range_cond->addComparison(cond_y2);
// //     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x1(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.0f));
// //     range_cond->addComparison(cond_x1);
// //     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_x2(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 20.0f));
// //     range_cond->addComparison(cond_x2);
// //     //创建滤波器并用条件定义对象初始化
// //     pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
// //     condrem.setCondition(range_cond);nodes
// //     condrem.setInputCloud(msg);
// //     //condrem.setKeepOrganized(true);//设置保持为结构点云   
// //     // apply filter应用滤波器   
// //     condrem.filter(*pointCloud_filter);
// // // point_pub.publish(pointCloud_filter);

// // pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
// //     pcl::PointIndices::Ptr plane_inliers ( new pcl::PointIndices );
// //     pcl::ModelCoefficients::Ptr plane_coefficients ( new pcl::ModelCoefficients );
// //     plane_seg.setOptimizeCoefficients (true);
// //     plane_seg.setModelType ( pcl::SACMODEL_PLANE );
// //     plane_seg.setMethodType ( pcl::SAC_RANSAC );
// //     plane_seg.setDistanceThreshold ( 0.05 );
// //     plane_seg.setInputCloud ( msg );
// //     plane_seg.segment (*plane_inliers, *plane_coefficients);//得到平面系数，进而得到平面法向量
// // cout<< plane_coefficients->values[0] << " "
// //            << plane_coefficients->values[1] << " "
// //            << plane_coefficients->values[2] << " " << endl;
// //////////////////////////////////////////////////////////////////////////
// // pcl::transformPointCloud(*msg, *transform_point, CreateRoatationMatrix({0, 0, 0},{0.0f,0.0f,1.0f}));
//                                                                          //    x          y       z 影像校正成水平

//  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
// /////////濾掉地面訊號
//     pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);
//     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 3.3f));
//     range_cond->addComparison(cond_z1);
//     pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_z2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 5.5f));
//     range_cond->addComparison(cond_z2);
//         pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//     condrem.setCondition(range_cond);
//     condrem.setInputCloud(msg);
//     condrem.filter(*pointCloud_filter);

// // point_pub.publish(pointCloud_filter);

//   // pass along original time stamp and frame ID
//   alln_point_msg.header.stamp = msg->header.stamp;
//   alln_point_msg.header.frame_id = msg->header.frame_id;

//   size_t npoints = msg->points.size();
//   alln_point_msg.points.resize(npoints);

//   size_t obs_count = 0;
//   size_t empty_count = 0;
//   // either return full point cloud or a discretized version
// constructGridClouds(pointCloud_filter, npoints, obs_count, empty_count);
//   alln_point_msg.points.resize(obs_count);


//   if (point_pub.getNumSubscribers() > 0) {
//     point_pub.publish(alln_point_msg);
//   }

