#ifndef MPC_H
#define MPC_H

#include <Eigen/Core>
#include <vector>

using namespace std;

class MPC {
public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,
                       std::vector<double> road_obsP, double line_curve);
};

#endif /* MPC_H */
