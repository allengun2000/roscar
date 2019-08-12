#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/UInt8.h"
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
visualization_msgs::Marker way_pointRef_plot_msg;
std::vector<double> obs_xy;
          std::vector<double> ptsx;
          std::vector<double> ptsy;
double px = 0;
double py = 0;
double psi = 0;
double v = 1;
double delta = 0;
double a = 0;
const double Lf = 1.2;
const double Lr = 1.6;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
bool need_stop=0;
double dis_needstop=999;
// Checks if the SocketIO event has JSON data
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
 void callback3(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	v=msg->twist.linear.x;
}
 void callback2(const std_msgs::Float32::ConstPtr& msg)
{
	delta=msg->data;
}
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
void pointCallback(const VPointCloud::ConstPtr& msg){
  obs_xy.resize(2*msg->points.size()+1);
  obs_xy[0]=msg->points.size();
  for(unsigned i=0;i<msg->points.size();i++){
    obs_xy[i*2+1]=msg->points[i].x;
    obs_xy[i*2+2]=msg->points[i].y;
  }
}
void call_tfstop(const std_msgs::Float64MultiArray::ConstPtr& msg){
need_stop=msg->data[0];
dis_needstop=msg->data[1];
}
void waypoint_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  MPC mpc;
          // j[1] is the data JSON object
          ptsx.clear();
          ptsx.push_back(msg->data[3]); ptsx.push_back(msg->data[5]);ptsx.push_back(msg->data[9]); ptsx.push_back(msg->data[11]);
          ptsx.push_back(msg->data[13]); ptsx.push_back(msg->data[15]);ptsx.push_back(msg->data[17]); ptsx.push_back(msg->data[19]);
          ptsy.clear();
          ptsy.push_back(msg->data[4]);ptsy.push_back(msg->data[6]);ptsy.push_back(msg->data[10]);ptsy.push_back(msg->data[12]);
          ptsy.push_back(msg->data[14]);ptsy.push_back(msg->data[16]);ptsy.push_back(msg->data[18]);ptsy.push_back(msg->data[20]);
          px = msg->data[0];
          py = msg->data[1];
          psi = msg->data[2];


///////////////////////////  Sumulate control delay
          double latency = 0.1;
        double beta = atan((Lr / (Lr + Lf)) * tan(delta));
        psi = psi + (v / Lf) * sin(beta) * latency;
        px = px + v * cos(psi+beta) * latency;
        py = py + v * sin(psi+beta) * latency;

          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsy.size());


          for (size_t i = 0; i < ptsx.size(); i++) {
            double x_shift = ptsx[i] - px;
            double y_shift = ptsy[i] - py;

            xvals[i] = x_shift * cos(0-psi) - y_shift * sin(0-psi);
            yvals[i] = x_shift * sin(0-psi) + y_shift * cos(0-psi);

          }

          auto coeffs = polyfit(xvals, yvals, 3);

          double cte = polyeval(coeffs, 0);
          // epsi was:
          //   psi0 - atan(coeffs[1] + 2 * x0 * coeffs[1] + 3 * coeffs[2] * pow(x0, 2))
          // where x0 = 0, psi0 = 0
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd x0(6);
          x0 << 0, 0, 0, v, cte, epsi;

          auto solution = mpc.Solve(x0, coeffs,obs_xy);

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25)] instead of [-1, 1].

          // double steer_value = solution[0] / (deg2rad(25)*Lf);

          //////////////////////////////////////////////////////cmd here
          double steer_value = solution[0];
          double throttle_value = solution[1]; //a
          geometry_msgs::Pose2D cmd_msg;
          cout<<"th"<<rad2deg(steer_value)<<"  v"<<throttle_value<<endl;
          cmd_msg.y=3;
          cmd_msg.theta=(-rad2deg(steer_value) +40)* (720+ 720)/(40 +40) -720;
          cmd_pub.publish(cmd_msg);
          ///////////////////////////////speed cmd

          std_msgs::UInt8 speed_commend;
          if(need_stop==0){
          if (fabs(steer_value) > 360) {
              speed_commend.data = 190;
          } else if (fabs(steer_value) > 120) {
              speed_commend.data = 200;
          } else {
              speed_commend.data = 220;
          }
          }else{
            if(dis_needstop<16){
                speed_commend.data = 0;
            }else{
              speed_commend.data = 170;
            }
          }
          pub_speed_commend.publish(speed_commend);


          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          int num_points = 25;
          double ploy_inc = 0.8;
                    way_point_plot_msg.points.clear();
          for (int i = 1; i < num_points; i++) {
            next_x_vals.push_back(i*ploy_inc);
            next_y_vals.push_back(polyeval(coeffs, i*ploy_inc));
            p.x=i*ploy_inc;
            p.y=polyeval(coeffs, i*ploy_inc);
            way_point_plot_msg.points.push_back(p);
          }
          waypoint_pub.publish(way_point_plot_msg);



          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          double predict_x = v * latency;
          double predict_y = 0;
          way_pointRef_plot_msg.points.clear();
          for (size_t i = 2; i < solution.size(); i+=2) {
            mpc_x_vals.push_back(solution[i] + predict_x);
            mpc_y_vals.push_back(solution[i+1] + predict_y);
            p.x=solution[i]+ predict_x ;
            p.y=solution[i+1] ;
            way_pointRef_plot_msg.points.push_back(p);
          }
          waypointRef_pub.publish(way_pointRef_plot_msg);

}
int main(int argc, char *argv[]) {


  /////***************ros
  	ros::init(argc, argv, "MPC_control");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

  waypoint_pub = n.advertise<visualization_msgs::Marker>("waypoint_mpc", 0);
  waypointRef_pub = n.advertise<visualization_msgs::Marker>("ref_waypoint_mpc", 0);
  pub_speed_commend =n.advertise<std_msgs::UInt8>("/to_duino_gas", 1000);
	cmd_pub = n.advertise<geometry_msgs::Pose2D>("/cmd",1000);

  way_point_plot_msg.header.frame_id =  "base_link";
  way_point_plot_msg.header.stamp =ros::Time::now();
  way_point_plot_msg.ns = "lane_lines_marker";
  way_point_plot_msg.id = 0;
  way_point_plot_msg.type =  visualization_msgs::Marker::LINE_STRIP;
  way_point_plot_msg.action =  visualization_msgs::Marker::ADD;
 // marker.pose = msg->pose;
  way_point_plot_msg.scale.x = 0.2;
  way_point_plot_msg.color.a = 0.8;
  way_point_plot_msg.color.b = 1.0;
  way_point_plot_msg.color.g = 1.0;
  way_point_plot_msg.color.r = 1.0;

  way_point_plot_msg.frame_locked =true;
way_pointRef_plot_msg=way_point_plot_msg;
way_pointRef_plot_msg.color.g=1.0;
way_pointRef_plot_msg.color.b=0;
  // waypoint_pub.publish(way_point_plot_msg);
  obs_xy.resize(1);
  obs_xy[0]=0;
//////ros
	ros::Subscriber sub = n.subscribe("Do_you_like_ice_cream", 10, waypoint_callback);
	ros::Subscriber sub2 = n.subscribe("/wheelFB", 1000, callback2);
  ros::Subscriber sub3 = n.subscribe("/current_velocity", 1000, callback3);
  ros::Subscriber sub4 = n.subscribe("Tf_light_info", 10, call_tfstop);
  ros::Subscriber velodyne_scan_ = n.subscribe("/allen_point", 10, pointCallback, ros::TransportHints().tcpNoDelay(true));
  // uWS::Hub h;
  // MPC is initialized here!

ros::spin();
  // h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
  //                    uWS::OpCode opCode) {
  //                      if(!ros::ok())
  //                       return 0;
  //   // "42" at the start of the message means there's a websocket message event.
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

  //         // 1. Transform ptsx, ptsy from global coordinate space into car coordinate space.
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
  //         //   psi0 - atan(coeffs[1] + 2 * x0 * coeffs[1] + 3 * coeffs[2] * pow(x0, 2))
  //         // where x0 = 0, psi0 = 0
  //         double epsi = -atan(coeffs[1]);

  //         Eigen::VectorXd x0(6);
  //         x0 << 0, 0, 0, v, cte, epsi;

  //         auto solution = mpc.Solve(x0, coeffs);

  //         // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
  //         // Otherwise the values will be in between [-deg2rad(25), deg2rad(25)] instead of [-1, 1].

  //         // double steer_value = solution[0] / (deg2rad(25)*Lf);
  //         double steer_value = solution[0] / deg2rad(25);
  //         double throttle_value = solution[1];
  //         msgJson["steering_angle"] = -steer_value;
  //         msgJson["throttle"] = throttle_value;


  //         //Display the waypoints/reference line
  //         vector<double> next_x_vals;
  //         vector<double> next_y_vals;

  //         //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
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

  //         //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
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
  // h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
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
