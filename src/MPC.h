#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);


  // simulate state
  static Eigen::VectorXd stateSimulation(Eigen::VectorXd initState, Eigen::VectorXd coeffs, double timeInS, double steer, double throttle );

};

#endif /* MPC_H */
