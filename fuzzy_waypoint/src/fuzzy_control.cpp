#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"

#define steering_commend_filter_parameter 5

float lateral_error_array[5] = {};
float header_error_array[5] = {};
float lateral_error = 0;
float header_error = 0;
float steering_commend = 0;
float speed_commend = 0;
bool braker_commend = 0;
unsigned int speed_commend_from_open_planner = 255;
float steering_commend_array[steering_commend_filter_parameter] = {};
std_msgs::String plan_type;

// best_parameter 1 20
// fuzzy_membersip_function
//偏移正數為右 偏移負數為左
//橫向偏移量歸屬函數 單位：公尺
float l_1 = -1.5;
float l_2 = 0;
float l_3 = 1.5;
//角度偏移量歸屬函數 單位：度
float h_1 = -20;
float h_2 = 0;
float h_3 = 20;
//方向盤輸出量歸屬函數 單位：度
float output_right = 540;
float output_left = -540;

float fuzzy_control(float header_error, float lateral_error) {
  float uh_left, uh_mid, uh_right, ul_left, ul_mid, ul_right, control_signal;

  //橫向偏移量模糊化
  if (lateral_error <= l_1) {
    ul_left = 1;
    ul_mid = 0;
    ul_right = 0;
  } else if ((lateral_error > l_1) && (lateral_error <= l_2)) {
    ul_left = (l_2 - lateral_error) / (l_2 - l_1);
    ul_mid = (lateral_error - l_1) / (l_2 - l_1);
    ul_right = 0;
  } else if ((lateral_error >= l_2) && (lateral_error < l_3)) {
    ul_left = 0;
    ul_mid = (l_3 - lateral_error) / (l_3 - l_2);
    ul_right = (lateral_error - l_2) / (l_3 - l_2);
  } else if (lateral_error >= l_3) {
    ul_left = 0;
    ul_mid = 0;
    ul_right = 1;
  } else {
    ul_left = 0;
    ul_mid = 0;
    ul_right = 0;
  }

  //頭角偏移量模糊化
  if (header_error <= h_1) {
    uh_left = 1;
    uh_mid = 0;
    uh_right = 0;
  } else if ((header_error > h_1) && (header_error <= h_2)) {
    uh_left = (h_2 - header_error) / (h_2 - h_1);
    uh_mid = (header_error - h_1) / (h_2 - h_1);
    uh_right = 0;
  } else if ((header_error >= h_2) && (header_error < h_3)) {
    uh_left = 0;
    uh_mid = (h_3 - header_error) / (h_3 - h_2);
    uh_right = (header_error - h_2) / (h_3 - h_2);
  } else if (header_error >= h_3) {
    uh_left = 0;
    uh_mid = 0;
    uh_right = 1;
  } else {
    uh_left = 0;
    uh_mid = 0;
    uh_right = 0;
  }

  if (((lateral_error > l_1) && (lateral_error <= l_2)) &&
      ((header_error > h_1) && (header_error <= h_2))) {
    float z1 = ul_left * output_right;
    float z2 = uh_left * output_right;
    float min = (z1 + z2 - fabs(z1 - z2)) / 2;
    control_signal = z1 + z2 - min;
  } else if ((lateral_error < l_2) && (header_error > h_2)) {
    float z1 = pow(ul_left, 2) * output_right;
    float z2 = pow(uh_right, 2) * output_left;
    control_signal = (z1 + z2) / (ul_left + uh_right);
  } else if ((lateral_error > l_2) && (header_error < h_2)) {
    float z1 = pow(ul_right, 2) * output_left;
    float z2 = pow(uh_left, 2) * output_right;
    control_signal = (z1 + z2) / (ul_right + uh_left);
  } else if (((lateral_error >= l_2) && (lateral_error < l_3)) &&
             ((header_error >= h_2) && (header_error < h_3))) {
    float z1 = ul_right * output_left;
    float z2 = uh_right * output_left;
    control_signal = (z1 + z2 - fabs(z1 - z2)) / 2;
  } else if ((lateral_error <= l_1) || (header_error <= h_1)) {
    control_signal = output_right;
  } else if ((lateral_error >= l_3) || (header_error >= h_3)) {
    control_signal = output_left;
  } else {
    control_signal = 0;
  }

  return control_signal;
}

// void ch_header_error(const std_msgs::Float64::ConstPtr &msg) {
//   header_error = msg->data;
// }

// void ch_lateral_error(const std_msgs::Float64::ConstPtr &msg) {
//   lateral_error = msg->data;
// }

void ch_do_you_like_ice_cream(
    const std_msgs::Float64MultiArray::ConstPtr &msg) {
  lateral_error = msg->data[7];
  header_error = msg->data[8];
}

void ch_open_plan_speed_command(const std_msgs::UInt8::ConstPtr &msg) {
  speed_commend_from_open_planner = msg->data;
}

void ch_best_plan_type(const std_msgs::String::ConstPtr &msg) {
  plan_type.data = msg->data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fuzzy_control");
  ros::NodeHandle n;
  // ros::Subscriber sub_header_error =
  //     n.subscribe("/header_error", 1000, ch_header_error);
  // ros::Subscriber sub_lateral_error =
  //     n.subscribe("/lateral_error", 1000, ch_lateral_error);
  ros::Subscriber sub_do_you_like_ice_cream =
      n.subscribe("/Do_you_like_ice_cream", 1000, ch_do_you_like_ice_cream);
  ros::Subscriber sub_open_plan_speed_command =
      n.subscribe("/open_plan_speed_commend", 1000, ch_open_plan_speed_command);
  ros::Subscriber sub_best_plan_type =
      n.subscribe("/best_plan_type", 1000, ch_best_plan_type);

  ros::Publisher pub_steering_commend =
      n.advertise<geometry_msgs::Pose2D>("/cmd", 1000);
  ros::Publisher pub_speed_commend =
      n.advertise<std_msgs::UInt8>("/to_duino_gas", 1000);
  ros::Publisher pub_braker_commend =
      n.advertise<std_msgs::Bool>("/to_duino_braker", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    //steering angle commend
    steering_commend = fuzzy_control(header_error, lateral_error);
    steering_commend_array[steering_commend_filter_parameter - 1] =
        steering_commend;
    float connend_sum = 0;
    for (int i = 0; i < steering_commend_filter_parameter - 1; i++) {
      steering_commend_array[i] = steering_commend_array[i + 1];
      connend_sum += steering_commend_array[i];
    }
    steering_commend = connend_sum / (steering_commend_filter_parameter - 1);

    geometry_msgs::Pose2D steering_commend_for_pub;
    steering_commend_for_pub.x = 0;
    steering_commend_for_pub.y = 5;
    steering_commend_for_pub.theta = steering_commend;
    pub_steering_commend.publish(steering_commend_for_pub);

    std_msgs::UInt8 speed_commend_for_pub;
    std_msgs::Bool braker_commend_for_pub;
    
    //speed commend
    if (speed_commend_from_open_planner == 0) {
      speed_commend = 0;
      // braker_commend = 1;
    } else if (speed_commend_from_open_planner == 170) {
      speed_commend = 170;
      braker_commend = 0;
    } else if (speed_commend_from_open_planner == 255) {
      if (fabs(steering_commend) > 360) {
        speed_commend = 170;
      } else if (fabs(steering_commend) > 120) {
        speed_commend = 170;
      } else {
        speed_commend = 170;
      }
      braker_commend = 0;
    }

    speed_commend_for_pub.data = speed_commend;
    braker_commend_for_pub.data = braker_commend;

    pub_braker_commend.publish(braker_commend_for_pub);
    pub_speed_commend.publish(speed_commend_for_pub);

    //info_print
    ROS_INFO("header_error:[%f], lateral_error:[%f], steering_commend:[%f], "
             "speed_commend:[%f], plan_type:[%s]",
             header_error, lateral_error, steering_commend, speed_commend,
             plan_type.data.c_str());
    loop_rate.sleep();
  }
  return 0;
}
