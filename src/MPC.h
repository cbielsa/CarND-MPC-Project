#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {

 public:
  
  MPC( double Lf );

  virtual ~MPC();

  // Solve the model given an initial state and polynominal
  // coefficients for the reference trajectory.
  // Return the next state and actuations as a vector.
  vector<double> Solve(
  	Eigen::VectorXd x0,
  	Eigen::VectorXd coeffs,
  	vector<double>& x_pred,  // predicted trajectory, x-values
  	vector<double>& y_pred   // predicted trajectory, y-values
  	);

  private:

  	double Lf_;
};

#endif /* MPC_H */
