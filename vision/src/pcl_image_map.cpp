#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Eigen>
using std::cout;
using std::endl;
using std::string;
tf::StampedTransform camera_lidar_tf_;

cv::Size image_size_;
cv::Mat camera_instrinsics_;
cv::Mat distortion_coefficients_;
cv::Mat current_frame_;

std::string image_frame_id_;

bool processing_;
bool camera_info_ok_;
bool camera_lidar_tf_ok_;
tf::TransformListener*  transform_listener_;
float fx_, fy_, cx_, cy_;
pcl::PointCloud<pcl::PointXYZRGB> colored_cloud_;
ros::Publisher publisher_fused_cloud_;
pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point,
                             const tf::StampedTransform &in_transform);

void ImageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg);

void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg);
namespace std {
template <> class hash<cv::Point> {
public:
  size_t operator()(const cv::Point &pixel_cloud) const {
    return hash<std::string>()(std::to_string(pixel_cloud.x) + "|" +
                               std::to_string(pixel_cloud.y));
  }
};
};
namespace velodyne_pointcloud {
struct PointXYZIR {
  PCL_ADD_POINT4D; // quad-word XYZ
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace velodyne_pointclou

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)                 //
                                  (float, y, y)                 //
                                  (float, z, z)                 //
                                  (float, intensity, intensity) //
                                  (uint16_t, ring, ring))       //
// pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud;
int count = 0;
pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point,
                             const tf::StampedTransform &in_transform) {
  tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
  tf::Vector3 tf_point_t = in_transform * tf_point;
  return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

tf::StampedTransform FindTransform(const std::string &in_target_frame,
                                   const std::string &in_source_frame) {
  tf::StampedTransform transform;
  camera_lidar_tf_ok_ = false;
  try {
    transform_listener_->lookupTransform(in_target_frame, in_source_frame,
                                         ros::Time(0), transform);
    camera_lidar_tf_ok_ = true;
    ROS_INFO(" Camera-Lidar TF obtained");
  } catch (tf::TransformException ex) {
     ROS_ERROR("[%s] %s", "fds", ex.what());
    std::cerr << "Transform lookup failed" << std::endl;
  }
  return transform;
}
void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg) {

  if (current_frame_.empty()) {
    ROS_INFO(" Waiting for Image frame to be available.");
    return;
  }
  if (!camera_lidar_tf_ok_) {

    camera_lidar_tf_ = FindTransform("camera", "velodyne");
  }
  if (!camera_info_ok_ || !camera_lidar_tf_ok_) {
    ROS_INFO(" Waiting for Camera-Lidar TF and Intrinsics to be available.");
    return;
  }

  // tf::StampedTransform camera_lidar_tf_;
  // camera_lidar_tf_
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*in_cloud_msg, *in_cloud);
  std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;

  std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());
  for (size_t i = 0; i < in_cloud->points.size(); i++) {
    cam_cloud[i] = TransformPoint(in_cloud->points[i], camera_lidar_tf_);
    int u = int(cam_cloud[i].x * fx_ / cam_cloud[i].z + cx_);
    int v = int(cam_cloud[i].y * fy_ / cam_cloud[i].z + cy_);
    if ((u >= 0) && (u < image_size_.width) && (v >= 0) &&
        (v < image_size_.height) && cam_cloud[i].z > 0) {
      projection_map.insert(std::pair<cv::Point, pcl::PointXYZ>(
          cv::Point(u, v), in_cloud->points[i]));
    }
  }

  out_cloud->points.clear();

  for (int row = 0; row < image_size_.height; row++) {
    for (int col = 0; col < image_size_.width; col++) {
      std::unordered_map<cv::Point, pcl::PointXYZ>::const_iterator
          iterator_3d_2d;
      pcl::PointXYZ corresponding_3d_point;
      pcl::PointXYZRGB colored_3d_point;
      iterator_3d_2d = projection_map.find(cv::Point(col, row));
      if (iterator_3d_2d != projection_map.end()) {
        corresponding_3d_point = iterator_3d_2d->second;
        cv::Vec3b rgb_pixel = current_frame_.at<cv::Vec3b>(row, col);
        colored_3d_point.x = corresponding_3d_point.x;
        colored_3d_point.y = corresponding_3d_point.y;
        colored_3d_point.z = corresponding_3d_point.z;
        colored_3d_point.r = rgb_pixel[2];
        colored_3d_point.g = rgb_pixel[1];
        colored_3d_point.b = rgb_pixel[0];
        out_cloud->points.push_back(colored_3d_point);
      }
    }
  }
  // Publish PC
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*out_cloud, cloud_msg);
  cloud_msg.header = in_cloud_msg->header;
  publisher_fused_cloud_.publish(cloud_msg);
}
// static const std::string OPENCV_WINDOW = "Image window";
static int image_count = 0;
cv_bridge::CvImagePtr cv_ptr;
void imageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg) {

  if (!camera_info_ok_) {
    ROS_INFO(" Waiting for Intrinsics to be available.");
    return;
  }
  if (processing_)
    return;

  cv_bridge::CvImagePtr cv_image =
      cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
  //   cv::Mat in_image = cv_image->image;
  cv::Mat in_image;

  cv::flip(cv_image->image, in_image, -1);
  
  cv::Mat undistorted_image;
  // current_frame_=in_image;
  cv::undistort(in_image, current_frame_, camera_instrinsics_,
                distortion_coefficients_);

  image_frame_id_ = "camera";
  image_size_.height = current_frame_.rows;
  image_size_.width = current_frame_.cols;
}
static int image_counting = 0;
cv_bridge::CvImagePtr cv_ptr2;
void imageCallback2(const sensor_msgs::ImageConstPtr &input) {

  try {
    cv_ptr2 = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception");
    return;
  }

  cv_ptr2->image;
}

main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_con");
  ros::NodeHandle nh;
  ros::Subscriber bat_sub = nh.subscribe("/velodyne_points", 1, cloudCB);
  ros::Subscriber img = nh.subscribe("cam0/image_raw", 1, imageCallback);
  publisher_fused_cloud_ =
      nh.advertise<sensor_msgs::PointCloud2>("/points_fused", 1);

  // ros::Subscriber img2 = nh.subscribe("cam1/image_raw", 1000,
  // imageCallback2);
  // pcl::io::savePCDFile (ss.str (), cloud);
  tf::TransformListener transform_listener;
// rosrun tf static_transform_publisher 0 0 0.5 0 0 1.4 velodyne camera 100
  transform_listener_ = &transform_listener;
  double d[5] = {-0.2625343749866142, 0.04682597339167545,
                 -0.006855668257370209, -0.0012440852638838321, 0.0};
  double K[9] = {353.963809687024,
                 0.0,
                 317.7884752033716,
                 0.0,
                 373.02542724366293,
                 313.85238366217703,
                 0.0,
                 0.0,
                 1.0};
  //   double P[12] = {245.2896270751953, 0.0, 311.51817259659583, 0.0, 0.0,
  //   308.9870300292969, 332.44782567305447, 0.0, 0.0, 0.0, 1.0, 0.0};
  double P[12] = {345.2896270751953,
                  0.0,
                  311.51817259659583,
                  0.0,
                  0.0,
                  308.9870300292969,
                  332.44782567305447,
                  0.0,
                  0.0,
                  0.0,
                  1.0,
                  0.0};
  camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      camera_instrinsics_.at<double>(row, col) = K[row * 3 + col];
    }
  }

  distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++) {
    // distortion_coefficients_.at<double>(col) = in_message.D[col];
    distortion_coefficients_.at<double>(col) = d[col];
  }

  fx_ = static_cast<float>(P[0]);
  fy_ = static_cast<float>(P[5]);
  cx_ = static_cast<float>(P[2]);
  cy_ = static_cast<float>(P[6]);
  camera_lidar_tf_ok_ = false;
  camera_info_ok_ = true;
  processing_ = false;
  image_frame_id_ = "camera";
  ros::spin();

  return 0;
}
