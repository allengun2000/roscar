#include "MPC.h"
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

size_t N = 15;
// double dt = 0.25;
double dt = 0.1;

const double Lf = 1.2;
const double Lr = 1.6;
double ref_v = 3;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  std::vector<double> road_obsPeak;
  double line_curve;
  FG_eval(Eigen::VectorXd coeffs, std::vector<double> road_obsPeak,
          double line_curve) {
    this->coeffs = coeffs;
    this->road_obsPeak = road_obsPeak;
    this->line_curve = line_curve;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector &fg, const ADvector &vars) {

    fg[0] = 0;
    for (size_t i = 0; i < N; i++) {
      fg[0] += (100 + i * 10) * CppAD::pow(vars[cte_start + i], 2);
      fg[0] += (100 + i * 10) * CppAD::pow(vars[epsi_start + i], 2);

      fg[0] += 100 * CppAD::pow(vars[v_start + i] - ref_v, 2);

      fg[0] += 10 * CppAD::abs(vars[v_start + i] * line_curve);
    }

    // Minimize actuator use
    for (size_t i = 0; i < N - 1; i++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += 50 * CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize actuator change
    for (size_t i = 0; i < N - 2; i++) {
      fg[0] += 60000 *
               CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
    // road can go
    double car_x = 0;
    double car_y = 0;
    double theta = 0;
    double car_weight = 0.5;
    double car_height = 0.5;
    for (size_t i = 0; i < N; i++) {
      for (size_t j = 0; j < road_obsPeak[0]; j++) {
        car_x = road_obsPeak[j * 2 + 1];
        car_y = road_obsPeak[j * 2 + 2];
        AD<double> x = vars[x_start + i];
        AD<double> y = vars[y_start + i];
        fg[0] += 999999 * CppAD::exp(-(((x - car_x) * CppAD::cos(theta) -
                                        (y - car_y) * CppAD::sin(theta)) /
                                       car_weight) *
                                     (((x - car_x) * CppAD::cos(theta) -
                                       (y - car_y) * CppAD::sin(theta)) /
                                      car_weight)) *
                 CppAD::exp(-(((y - car_y) * CppAD::cos(theta) +
                               (x - car_x) * CppAD::sin(theta)) /
                              car_height) *
                            (((y - car_y) * CppAD::cos(theta) +
                              (x - car_x) * CppAD::sin(theta)) /
                             car_height));
      }
    }

    // Initial state contraints are the same as state0
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (size_t t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      // AD<double> f0 = 0;
      // for (size_t i = 0; i < coeffs.size(); i++) {
      // f0+=coeffs[i] * CppAD::pow(x0, i);
      // }
      // AD<double> psides0 = 0;
      // for (size_t i = 1; i < coeffs.size(); i++){
      //   psides0+= i* coeffs[i] * CppAD::pow(x0, i-1) ;
      // }
      // psides0=CppAD::atan(psides0);
      AD<double> f0 =
          coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) +
          coeffs[3] * CppAD::pow(x0, 3);// + coeffs[4] * CppAD::pow(x0, 4);
      // coeffs[5] * CppAD::pow(x0, 5) + coeffs[6] * CppAD::pow(x0, 6) +
      // coeffs[7] * CppAD::pow(x0, 7) + coeffs[8] * CppAD::pow(x0, 8) ;
      AD<double> psides0 =
          CppAD::atan(coeffs[1] +                             //
                      2 * coeffs[2] * x0 +                    //
                      3 * coeffs[3] * CppAD::pow(x0, 2)) ;     //
                          //4 * coeffs[4] * CppAD::pow(x0, 3)); //
      // 5 * coeffs[5] * CppAD::pow(x0, 4)) ; //
      // 6 * coeffs[6] * CppAD::pow(x0, 5) * //
      // 7 * coeffs[7] * CppAD::pow(x0, 6) * //
      // 8 * coeffs[8] * CppAD::pow(x0, 7));
      AD<double> beta = CppAD::atan((Lr / (Lr + Lf)) * CppAD::tan(delta0));
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0 + beta) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0 + beta) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * CppAD::sin(beta) / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,
                          std::vector<double> road_obsP, double line_curve) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  // (x, y, psi, v, cte, e_psi), (delta, a)
  size_t n_vars = N * 6 + (N - 1) * 2;
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non actuator upper and lower limits
  // to the max negative and positive values.
  for (i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Set acceleration limit
  for (i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, road_obsP, line_curve);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  // std::cout << "cost:" << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[v_start + 5]);

  for (size_t i = 0; i < N - 1; i++) {
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }

  return result;
}
