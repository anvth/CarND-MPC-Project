#ifndef MPC_H
#define MPC_H

#define HAVE_CSTDDEF

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

const int latency_ind = 2;
const double ref_v = 120;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  double delta_prev {0};
  double a_prev {0.1};
};

#endif /* MPC_H */
