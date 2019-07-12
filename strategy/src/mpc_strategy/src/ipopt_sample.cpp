# include <cppad/ipopt/solve.hpp>
# include <iostream>
#include <ros/ros.h>
#include "strategy/se.h"
using namespace std;
ros::ServiceClient client;
using CppAD::AD;

    class FG_eval {
    public:
        typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
        void operator()(ADvector& fg, const ADvector& x)
        {   assert( fg.size() == 3 );
            assert( x.size()  == 4 );
            // Fortran style indexing
            AD<double> x1 = x[0];
            AD<double> x2 = x[1];
            AD<double> x3 = x[2];
            AD<double> x4 = x[3];
            // f(x)
            fg[0] = x1 * x4 * (x1 + x2 + x3) + x3;
            // g_1 (x)
            fg[1] = x1 + x2;
            // g_2 (x)
            fg[2] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
            //
            return;
        }
    };
bool add(strategy::se::Request  &req,
         strategy::se::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{   bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR( double ) Dvector;
    ros::init(argc, argv, "ipop_learn");
        ros::NodeHandle n;


    ros::ServiceClient client = n.serviceClient<strategy::se>("add_two_ints");
    strategy::se srv;
    srv.request.a = 1;
    srv.request.b = 2;
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

    // number of independent variables (domain dimension for f and g)
    size_t nx = 4;
    // number of constraints (range dimension for g)
    size_t ng = 2;
    // initial value of the independent variables
    Dvector xi(nx);
    xi[0] = 1.0;
    xi[1] = 5.0;
    xi[2] = 5.0;
    xi[3] = 1.0;
    // lower and upper limits for x
    Dvector xl(nx), xu(nx);
    for(i = 0; i < nx; i++)
    {   xl[i] = 1.0;
        xu[i] = 5.0;
    }
    // lower and upper limits for g
    Dvector gl(ng), gu(ng);
    gl[0] = 25.0;     gu[0] = 1.0e19;
    gl[1] = 40.0;     gu[1] = 40.0;

    // object that computes objective and constraints
    FG_eval fg_eval;

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     10\n";
    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-6\n";
    // derivative testing
    options += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius  0.\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, xi, xl, xu, gl, gu, fg_eval, solution
    );
    //
    // Check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    //
    double check_x[]  = { 1.000000, 4.743000, 3.82115, 1.379408 };
    double check_zl[] = { 1.087871, 0.,       0.,      0.       };
    double check_zu[] = { 0.,       0.,       0.,      0.       };
    double rel_tol    = 1e-6;  // relative tolerance
    double abs_tol    = 1e-6;  // absolute tolerance
    for(i = 0; i < nx; i++)
    {   ok &= CppAD::NearEqual(
            check_x[i],  solution.x[i],   rel_tol, abs_tol
        );
        ok &= CppAD::NearEqual(
            check_zl[i], solution.zl[i], rel_tol, abs_tol
        );
        ok &= CppAD::NearEqual(
            check_zu[i], solution.zu[i], rel_tol, abs_tol
        );
	cout<<solution.x[i]<<endl;
    }
 return 0;
}

/*
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Checks if the SocketIO event has JSON data.
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

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          const double Lf = 2.67;

          // Sumulate control delay
          double latency = 0.1;
          px = px + v * cos(psi) * latency;
          py = py + v * sin(psi) * latency;

          // delta from simulator is in the opposite direction
          psi = psi - v / Lf * delta * latency;
          v = v + a * latency;

          // Response json message
          json msgJson;

          // 1. Transform ptsx, ptsy from global coordinate space into car coordinate space.
          // 2. Then prepare initial state x0 for MPC optimizer
          //    x, y, psi should all become zero after the transformation.

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

          auto solution = mpc.Solve(x0, coeffs);

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25)] instead of [-1, 1].

          // double steer_value = solution[0] / (deg2rad(25)*Lf);
          double steer_value = solution[0] / deg2rad(25);
          double throttle_value = solution[1];
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;


          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          int num_points = 25;
          double ploy_inc = 2.5;
          for (size_t i = 1; i < num_points; i++) {
            next_x_vals.push_back(i*ploy_inc);
            next_y_vals.push_back(polyeval(coeffs, i*ploy_inc));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          double predict_x = v * latency;
          double predict_y = 0;
          for (size_t i = 2; i < solution.size(); i+=2) {
            mpc_x_vals.push_back(solution[i] + predict_x);
            mpc_y_vals.push_back(solution[i+1] + predict_y);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
*/

