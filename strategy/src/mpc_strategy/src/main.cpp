#include "MPC.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/TwistStamped.h"
#include "json.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <chrono>
#include <iostream>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

// for convenience
using namespace std;
typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
ros::Publisher waypoint_pub;
ros::Publisher pub_speed_commend;
geometry_msgs::Point p;
visualization_msgs::Marker way_point_plot_msg;
ros::Publisher waypointRef_pub;
ros::Publisher cmd_pub;
ros::Publisher break_pub;
visualization_msgs::Marker way_pointRef_plot_msg;
std::vector<double> obs_xy;
std::vector<double> ptsx;
std::vector<double> ptsy;
bool stop_flag = 0;
bool slow_flag = 0;
double U_oil = 0;

int count_stop = 0;
double px = 0;
double py = 0;
double psi = 0;
double v = 1;
double delta = 0;
double a = 0;
const double Lf = 1.2;
const double Lr = 1.6;
double error_v_I[10] = {0};
double average_v[10] = {0};
double v_average = 0;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
bool need_stop = 0;
double dis_needstop = 999;
double back_point[8] = {0};
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
void callback3(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  v = msg->twist.linear.x;
  double tem_v = 0;
  for (int i = 0; i < 9; i++) {
    average_v[i] = average_v[i + 1];
    tem_v += average_v[i];
  }
  average_v[9] = v;
  tem_v /= 9;
  v_average = tem_v;
}
void stop_go_data(const std_msgs::Bool::ConstPtr &msg) {
  stop_flag = msg->data;
}
void callback2(const std_msgs::Float32::ConstPtr &msg) { delta = msg->data; }
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}
void pointCallback(const VPointCloud::ConstPtr &msg) {
  obs_xy.resize(2 * msg->points.size() + 1);
  obs_xy[0] = msg->points.size();
  for (unsigned i = 0; i < msg->points.size(); i++) {
    obs_xy[i * 2 + 1] = msg->points[i].x;
    obs_xy[i * 2 + 2] = msg->points[i].y;
  }
}
void call_tfstop(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  need_stop = msg->data[0];
  dis_needstop = msg->data[2];
}
////////////plan a vel_control PID control
// double vel_control(double target_speed) {
//   // 100-220
//   // 90-150
//   // 150-220
//   double error_I_sum = 0;
//   for (int i = 0; i < 9; i++) {
//     error_v_I[i] = error_v_I[i + 1];
//     error_I_sum += error_v_I[i];
//   }

//   error_v_I[9] = target_speed - v_average;
//   error_I_sum += error_v_I[9];
//   double p = 0.25;
//   double I = 0.0001;
//   double D = -4;
//   // double oil_cmd = p * (target_speed - v_average) + I * error_I_sum;
//   double oil_cmd = p * (target_speed - v_average) + D * (v - v_average);

//   U_oil += oil_cmd;
//   cout << "  target_v  " << target_speed << " VV " << v_average << " V " << v
//        << "oid_cmd " << oil_cmd << endl;
//   // oil_cmd =(130 * oil_cmd + 930)/6 ; //座標轉換

//   if (U_oil < 90) {
//     U_oil = 90;
//   }
//   if (U_oil > 220) {
//     U_oil = 220;
//   }

//   return U_oil;
// }
////////////plan b vel_control
double vel_control(double target_speed) {
  double plan_v[16][2] = {{100, 0},   {110, 0.5}, {120, 1},   {130, 1.3},
                          {140, 1.6}, {150, 2},   {160, 2.4}, {170, 2.6},
                          {180, 3.4}, {190, 3.8}, {200, 4.3}, {210, 4.7},
                          {220, 5.3}, {230, 5.6}, {240, 5.8}, {250, 6}};
  double middle_oil;
  if (target_speed < 0.1) {
    middle_oil = 100;
  } else if (target_speed > 5.9) {
    middle_oil = 250;
  } else {
    for (size_t i = 0; i < 14; i++) {
      if (target_speed > plan_v[i][1] && target_speed < plan_v[i + 1][1]) {
        middle_oil = plan_v[i][0] +
                     10 * ((target_speed - plan_v[i][1]) /
                           (plan_v[i + 1][1] - plan_v[i][1]));
      }
    }
  }
  double p = 10;
  double oil_cmd = p * (target_speed - v_average)+middle_oil;
  cout<< "  target_v  " << target_speed << "  current_v  " << v << " error " << (target_speed - v_average) << " middleoil " << middle_oil
       << "oid_cmd " << oil_cmd << endl;
  return oil_cmd;
}
void waypoint_callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  cout << "-----------------------------------------" << endl;
  MPC mpc;
  ////refwaypoint back
  if (ptsx[3] != msg->data[9]) {
    for (int i = 0; i < 6; i++) {
      back_point[i] = back_point[i + 2];
    }
    back_point[6] = msg->data[9];
    back_point[7] = msg->data[10];
  }
  ptsx.clear();
  ptsy.clear();
  for (int i = 0; i < 6; i += 2) {
    ptsx.push_back(back_point[i]);
    ptsy.push_back(back_point[i + 1]);
  }

  for (size_t i = 9; i < 32; i += 2) {
    ptsx.push_back(msg->data[i]);
    ptsy.push_back(msg->data[i + 1]);
  }

  size_t long_size = ptsx.size() - 1;
  for (size_t i = 0; i < long_size; i++) {
    double x_mis = ptsx[i + 1] - ptsx[i];
    double y_mis = ptsy[i + 1] - ptsy[i];
    for (size_t j = 0; j < 5; j++) {
      ptsx.push_back(ptsx[i] + j * x_mis / 5);
      ptsy.push_back(ptsy[i] + j * y_mis / 5);
    }
    // ptsx.push_back(ptsx[i + 1] / 2 + ptsx[i] / 2);
    // ptsy.push_back(ptsy[i + 1] / 2 + ptsy[i] / 2);
  }

  px = msg->data[0];
  py = msg->data[1];
  psi = msg->data[2];

  ///////////////////////////  Sumulate control delay
  double latency = 0;
  double beta = atan((Lr / (Lr + Lf)) * tan(delta));
  psi = psi + (v / Lf) * sin(beta) * latency;
  px = px + v * cos(psi + beta) * latency;
  py = py + v * sin(psi + beta) * latency;

  Eigen::VectorXd xvals(ptsx.size());
  Eigen::VectorXd yvals(ptsy.size());
  for (size_t i = 0; i < ptsx.size(); i++) {
    double x_shift = ptsx[i] - px;
    double y_shift = ptsy[i] - py;
    xvals[i] = x_shift * cos(0 - psi) - y_shift * sin(0 - psi);
    yvals[i] = x_shift * sin(0 - psi) + y_shift * cos(0 - psi);
  }
  // waypoint_pub.publish(way_point_plot_msg);
  auto coeffs = polyfit(xvals, yvals, 3);

  double cte = polyeval(coeffs, 0); //車子offset

  double epsi = -atan(coeffs[1]); //車頭角度

  Eigen::VectorXd x0(6);
  x0 << 0, 0, 0, v_average, cte, epsi;

  ////////////////////////////算曲緑

  // Display the waypoints/reference line
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  int num_points = 25;
  double ploy_inc = 0.4;

  way_point_plot_msg.points.clear();
  for (int i = 1; i < num_points; i++) {
    next_x_vals.push_back(i * ploy_inc);
    next_y_vals.push_back(polyeval(coeffs, i * ploy_inc));
    p.x = i * ploy_inc;
    p.y = polyeval(coeffs, i * ploy_inc);
    way_point_plot_msg.points.push_back(p);
  }
  waypoint_pub.publish(way_point_plot_msg);

  double line_sum = 0;
  for (size_t i = 0; i < next_x_vals.size() - 1; i++) {
    line_sum += sqrt(pow(next_x_vals[i + 1] - next_x_vals[i], 2) +
                     pow(next_y_vals[i + 1] - next_y_vals[i], 2));
  }
  double line_curve =
      line_sum /
      sqrt(pow(next_x_vals[next_x_vals.size() - 1] - next_x_vals[0], 2) +
           pow(next_y_vals[next_y_vals.size() - 1] - next_y_vals[0], 2));

  line_curve = (line_curve - 1) * 100;
  auto solution = mpc.Solve(x0, coeffs, obs_xy, line_curve);

  //////////////////////////////////////////////////////cmd here
  double steer_value = solution[0];
  double throttle_value = solution[1]; // a
  geometry_msgs::Pose2D cmd_msg;

  cmd_msg.y = 3;
  cmd_msg.theta = (-rad2deg(steer_value) + 40) * (720 + 720) / (40 + 40) - 720;
  cmd_pub.publish(cmd_msg);
  ///////////////////////////////speed cmd
  std_msgs::UInt8 speed_commend;
  std_msgs::Bool break_com;
  if (need_stop == 0) {
    /// mpc plan a
    // speed_commend.data = vel_control(throttle_value);
    // cout << "mpc v" << throttle_value << endl;
    // speed_commend.data=180;
    /////plan b
    // if (fabs(cmd_msg.theta) > 360) {
    //   speed_commend.data = 130;
    // } else if (fabs(cmd_msg.theta) > 120) {
    //   speed_commend.data = 130;
    // } else {
    //   speed_commend.data = 200;
    // }
    //////plan c
    if (line_curve < 1) {
      speed_commend.data = vel_control(4);
    } else {
      speed_commend.data = vel_control(2);
      cout << "turn slowwwwww douwn" << line_curve << endl;
    }
  } else {
    if (dis_needstop < 6) {
      speed_commend.data = 0;
      U_oil = 0;
      cout << "000000000000000000000000000stoppppppppppppppppppppppp" << endl;
      break_com.data = 1;
    } else {
      speed_commend.data = vel_control(1);
      cout << "tftfslowwwwwwwwwwwwwwwwwwwwwwwwwwwww" << endl;
    }
  }

  // 15.5745,28.9548
  // -9.1115,9.7077 //
  // cout<<"xx"<<msg->data[3]<<"yy"<<msg->data[4]<<endl;
  //  (msg->data[3]==-9.8191 && msg->data[4]==18.6311)
  if (stop_flag == 1 || slow_flag == 1 ||
      (msg->data[9] == 8.7055 && msg->data[10] == 2.8850)) {

    slow_flag = 1;
    if (stop_flag != 1) {
      cout << "station_slowwwwwww" << endl;
      speed_commend.data = vel_control(1);
    }
    if ((msg->data[9] == 15.7674 && msg->data[10] == 26.6180) ||
        stop_flag == 1) {
      cout << "stoppppppppppp" << endl;
      U_oil = 0;
      speed_commend.data = 0;
      break_com.data = 1;
      stop_flag = 1;
    }

    if (v < 0.3) {
      count_stop++;
    }

    if (count_stop > 25) {
      break_com.data = 0;
      cout << "breakuppppppppppp" << endl;
    }

    if (count_stop > 100) {
      stop_flag = 0;
      slow_flag = 0;
    }
  } else {
    count_stop = 0;
  }

  pub_speed_commend.publish(speed_commend);
  break_pub.publish(break_com);
  cout << "steer" << rad2deg(steer_value) << "  oil" << (int)speed_commend.data
       << endl;

  // Display the MPC predicted trajectory
  vector<double> mpc_x_vals;
  vector<double> mpc_y_vals;

  //.. add (x,y) points to list here, points are in reference to the vehicle's
  // coordinate system
  // the points in the simulator are connected by a Green line

  double predict_x = v * latency;
  double predict_y = 0;
  way_pointRef_plot_msg.points.clear();
  for (size_t i = 2; i < solution.size(); i += 2) {
    mpc_x_vals.push_back(solution[i] + predict_x);
    mpc_y_vals.push_back(solution[i + 1] + predict_y);
    p.x = solution[i] + predict_x;
    p.y = solution[i + 1];
    way_pointRef_plot_msg.points.push_back(p);
  }
  waypointRef_pub.publish(way_pointRef_plot_msg);
}
int main(int argc, char *argv[]) {

  /////***************ros
  ros::init(argc, argv, "MPC_control");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  waypoint_pub = n.advertise<visualization_msgs::Marker>("waypoint_mpc", 0);
  waypointRef_pub =
      n.advertise<visualization_msgs::Marker>("ref_waypoint_mpc", 0);
  pub_speed_commend = n.advertise<std_msgs::UInt8>("/to_duino_gas", 1000);
  cmd_pub = n.advertise<geometry_msgs::Pose2D>("/cmd", 1000);
  break_pub = n.advertise<std_msgs::Bool>("/to_duino_braker", 1000);
  way_point_plot_msg.header.frame_id = "base_link";
  way_point_plot_msg.header.stamp = ros::Time::now();
  way_point_plot_msg.ns = "lane_lines_marker";
  way_point_plot_msg.id = 0;
  way_point_plot_msg.type = visualization_msgs::Marker::LINE_STRIP;
  way_point_plot_msg.action = visualization_msgs::Marker::ADD;
  // marker.pose = msg->pose;
  way_point_plot_msg.scale.x = 0.2;
  way_point_plot_msg.color.a = 0.8;
  way_point_plot_msg.color.b = 1.0;
  way_point_plot_msg.color.g = 1.0;
  way_point_plot_msg.color.r = 1.0;

  way_point_plot_msg.frame_locked = true;
  way_pointRef_plot_msg = way_point_plot_msg;
  way_pointRef_plot_msg.color.g = 1.0;
  way_pointRef_plot_msg.color.b = 0;
  // waypoint_pub.publish(way_point_plot_msg);
  obs_xy.resize(1);
  obs_xy[0] = 0;
  //
  ptsx.push_back(0);
  ptsy.push_back(0);
  //////ros
  ros::Subscriber sub =
      n.subscribe("Do_you_like_ice_cream", 10, waypoint_callback);
  ros::Subscriber sub2 = n.subscribe("/wheelFB", 1000, callback2);
  ros::Subscriber sub3 = n.subscribe("/current_velocity", 1000, callback3);
  ros::Subscriber sub4 = n.subscribe("Tf_light_info", 10, call_tfstop);
  ros::Subscriber sub5 = n.subscribe("/stop_go", 1000, stop_go_data);
  ros::Subscriber velodyne_scan_ =
      n.subscribe("/allen_point", 10, pointCallback,
                  ros::TransportHints().tcpNoDelay(true));

  ros::spin();
  // h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t
  // length,
  //                    uWS::OpCode opCode) {
  //                      if(!ros::ok())
  //                       return 0;
  //   // "42" at the start of the message means there's a websocket message
  //   event.
  //   // The 4 signifies a websocket message
  //   // The 2 signifies a websocket event
  //   string sdata = string(data).substr(0, length);
  //   cout << sdata << endl;
  //   if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
  //     string s = hasData(sdata);
  //     if (s != "") {
  //       auto j = json::parse(s);
  //       string event = j[0].get<string>();
  //       if (event == "telemetry") {
  //         // j[1] is the data JSON object
  //         vector<double> ptsx = j[1]["ptsx"];
  //         vector<double> ptsy = j[1]["ptsy"];
  //         double px = j[1]["x"];
  //         double py = j[1]["y"];
  //         double psi = j[1]["psi"];
  //         double v = j[1]["speed"];

  //         double delta = j[1]["steering_angle"];
  //         double a = j[1]["throttle"];

  //         const double Lf = 2.67;

  //         // Sumulate control delay
  //         double latency = 0.1;
  //         px = px + v * cos(psi) * latency;
  //         py = py + v * sin(psi) * latency;

  //         // delta from simulator is in the opposite direction
  //         psi = psi - v / Lf * delta * latency;
  //         v = v + a * latency;

  //         // Response json message
  //         json msgJson;

  //         // 1. Transform ptsx, ptsy from global coordinate space into car
  //         coordinate space.
  //         // 2. Then prepare initial state x0 for MPC optimizer
  //         //    x, y, psi should all become zero after the transformation.

  //         Eigen::VectorXd xvals(ptsx.size());
  //         Eigen::VectorXd yvals(ptsy.size());

  //         for (size_t i = 0; i < ptsx.size(); i++) {
  //           double x_shift = ptsx[i] - px;
  //           double y_shift = ptsy[i] - py;

  //           xvals[i] = x_shift * cos(0-psi) - y_shift * sin(0-psi);
  //           yvals[i] = x_shift * sin(0-psi) + y_shift * cos(0-psi);

  //         }

  //         auto coeffs = polyfit(xvals, yvals, 3);

  //         double cte = polyeval(coeffs, 0);
  //         // epsi was:
  //         //   psi0 - atan(coeffs[1] + 2 * x0 * coeffs[1] + 3 * coeffs[2] *
  //         pow(x0, 2))
  //         // where x0 = 0, psi0 = 0
  //         double epsi = -atan(coeffs[1]);

  //         Eigen::VectorXd x0(6);
  //         x0 << 0, 0, 0, v, cte, epsi;

  //         auto solution = mpc.Solve(x0, coeffs);

  //         // NOTE: Remember to divide by deg2rad(25) before you send the
  //         steering value back.
  //         // Otherwise the values will be in between [-deg2rad(25),
  //         deg2rad(25)] instead of [-1, 1].

  //         // double steer_value = solution[0] / (deg2rad(25)*Lf);
  //         double steer_value = solution[0] / deg2rad(25);
  //         double throttle_value = solution[1];
  //         msgJson["steering_angle"] = -steer_value;
  //         msgJson["throttle"] = throttle_value;

  //         //Display the waypoints/reference line
  //         vector<double> next_x_vals;
  //         vector<double> next_y_vals;

  //         //.. add (x,y) points to list here, points are in reference to the
  //         vehicle's coordinate system
  //         // the points in the simulator are connected by a Yellow line

  //         int num_points = 25;
  //         double ploy_inc = 2.5;
  //                   way_point_plot_msg.points.clear();
  //         for (int i = 1; i < num_points; i++) {
  //           next_x_vals.push_back(i*ploy_inc);
  //           next_y_vals.push_back(polyeval(coeffs, i*ploy_inc));
  //           p.x=i*ploy_inc;
  //           p.y=polyeval(coeffs, i*ploy_inc);
  //           way_point_plot_msg.points.push_back(p);
  //         }
  //         waypoint_pub.publish(way_point_plot_msg);
  //         msgJson["next_x"] = next_x_vals;
  //         msgJson["next_y"] = next_y_vals;

  //         //Display the MPC predicted trajectory
  //         vector<double> mpc_x_vals;
  //         vector<double> mpc_y_vals;

  //         //.. add (x,y) points to list here, points are in reference to the
  //         vehicle's coordinate system
  //         // the points in the simulator are connected by a Green line

  //         double predict_x = v * latency;
  //         double predict_y = 0;
  //         way_pointRef_plot_msg.points.clear();
  //         for (size_t i = 2; i < solution.size(); i+=2) {
  //           mpc_x_vals.push_back(solution[i] + predict_x);
  //           mpc_y_vals.push_back(solution[i+1] + predict_y);
  //           p.x=solution[i]+ predict_x ;
  //           p.y=solution[i+1] ;
  //           way_pointRef_plot_msg.points.push_back(p);
  //         }
  //         waypointRef_pub.publish(way_pointRef_plot_msg);
  //         msgJson["mpc_x"] = mpc_x_vals;
  //         msgJson["mpc_y"] = mpc_y_vals;

  //         auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  //         // std::cout << msg << std::endl;
  //         // Latency
  //         // The purpose is to mimic real driving conditions where
  //         // the car does actuate the commands instantly.
  //         //
  //         // Feel free to play around with this value but should be to drive
  //         // around the track with 100ms latency.
  //         //
  //         // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
  //         // SUBMITTING.
  //         this_thread::sleep_for(chrono::milliseconds(100));
  //         ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  //       }
  //     } else {
  //       // Manual driving
  //       std::string msg = "42[\"manual\",{}]";
  //       ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  //     }
  //   }
  // });

  // // We don't need this since we're not using HTTP but if it's removed the
  // // program
  // // doesn't compile :-(
  // h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char
  // *data,
  //                    size_t, size_t) {
  //   const std::string s = "<h1>Hello world!</h1>";
  //   if (req.getUrl().valueLength == 1) {
  //     res->end(s.data(), s.length());
  //   } else {
  //     // i guess this should be done more gracefully?
  //     res->end(nullptr, 0);
  //   }
  // });

  // h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
  //   std::cout << "Connected!!!" << std::endl;
  // });

  // h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
  //                        char *message, size_t length) {
  //   ws.close();
  //   std::cout << "Disconnected" << std::endl;
  // });

  // int port = 4567;
  // if (h.listen(port)) {
  //   std::cout << "Listening to port " << port << std::endl;
  // } else {
  //   std::cerr << "Failed to listen to port" << std::endl;
  //   return -1;
  // }
  // h.run();
}
