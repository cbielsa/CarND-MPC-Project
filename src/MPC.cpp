#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 2./N;

// Controller reference velocity
double ref_v = 50;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;



// Class to evaluate Cost and constraints ===================================

class FG_eval {

 private:

  // Distance from front wheel axis to CoG.
  double Lf_;

  // Coefficients of the fitted polynomial.
  Eigen::VectorXd coeffs;

 public:

  FG_eval(Eigen::VectorXd coeffs, double Lf)
   : coeffs(coeffs), Lf_(Lf)
  {}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;


  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    //
    // The weights are tuned to achieve a great trade-off between
    // low state errors and a smooth drive at high speeds
    for( size_t t=1; t<N; ++t ) {

      fg[0] +=

        // penalize state errors
           1 * CppAD::pow( vars[ cte_start+t ], 2)           // cte contrib
        +  9 * CppAD::pow( vars[ epsi_start+t ], 2)          // epsi contrib
        +  0.25 * CppAD::pow( vars[v_start+t]/ref_v-1., 2 )  // vel contr

        // penalize large actuations
        +  0.018 * CppAD::pow( vars[ delta_start+t-1 ], 2 )  // steering contrib
        +  0.002 * CppAD::pow( vars[ a_start+t-1 ], 2 );     // throttle contrib

    }

    for( size_t t=1; t<N-1; ++t ) {

      fg[0] +=

            // penalize large changes in actuations
               1200.*N/10 * CppAD::pow( vars[delta_start+t]-vars[delta_start+t-1], 2 )
            +  150.*N/10 * CppAD::pow( vars[a_start+t]-vars[a_start+t-1], 2 );
    }


    //
    // Setup model constraints
    //

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // convert Lf to supported type
    AD<double> ADLf(Lf_);

    // The rest of the constraints
    for (size_t t=1; t<N; t++) {

      AD<double> x1 = vars[x_start + t];
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y1 = vars[y_start + t];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v1 = vars[v_start + t];
      AD<double> v0 = vars[v_start + t - 1];

      AD<double> cte1 = vars[cte_start + t];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi1 = vars[epsi_start + t];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];


      // calculate orientation of the reference trajectory at x1,
      // i.e. atan( f'(x1) )
      AD<double> psi1_ref = CppAD::atan( coeffs[1] + 2*coeffs[2]*x1 + 3*coeffs[3]*x1*x1 );

      // Later step constraints
      // (simplified kinematic bicycle model)
      fg[1 + x_start + t]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);    // x
      fg[1 + y_start + t]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);    // y
      fg[1 + psi_start + t]  = psi1 - (psi0 + v0*delta0/ADLf * dt);       // psi
      fg[1 + v_start + t]    = v1 - (v0 + a0 * dt);                       // v
      fg[1 + cte_start + t]  = cte1 - ( cte0 + v0*CppAD::sin(epsi0)*dt ); // cte
      fg[1 + epsi_start + t] = epsi1 - ( psi1 - psi1_ref );               // epsi

    }
  }
};



// ===================================================================
// MPC class definition
//

MPC::MPC(double Lf) : Lf_(Lf) {}

MPC::~MPC() {}


vector<double> MPC::Solve(
  Eigen::VectorXd state, Eigen::VectorXd coeffs,
  vector<double>& x_pred, vector<double>& y_pred ) {

  typedef CPPAD_TESTVECTOR(double) Dvector;

  // assign references to input initial state variables
  // to improve code readibility
  double& x    = state[0];
  double& y    = state[1];
  double& psi  = state[2];
  double& v    = state[3];
  double& cte  = state[4];
  double& epsi = state[5];

  // number of independent variables for optimization
  size_t n_vars = N*6 + (N-1)*2;
  
  // Number of constraints for optimization
  size_t n_constraints = N*6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (size_t i=0; i<n_vars; ++i) {
    vars[i] = 0.0;
  }

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i=0; i<delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (size_t i=delta_start; i<a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (size_t i=a_start; i<n_vars; ++i) {
    vars_lowerbound[i] = -1.0;  // m/s2
    vars_upperbound[i] =  1.0;  // m/s2
  }

  // Lower and upper limits for the constraints
  // All of these should be 0 except for
  // the initial state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (size_t i=0; i<n_constraints; ++i) {
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
  FG_eval fg_eval(coeffs, Lf_);

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
  //ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  bool ok = ( solution.status == CppAD::ipopt::solve_result<Dvector>::success );
  if( !ok ) {
    std::cout << "Failure in call to ipopt::solve() !" << std::endl;
  }

  // Cost
  auto cost = solution.obj_value;

  // fill in output parameters with predicted trajectory in vehicle frame
  x_pred.resize(N);
  y_pred.resize(N);
  for( size_t i=0; i<N; ++i ) {
    x_pred[i] = solution.x[x_start+i];
    y_pred[i] = solution.x[y_start+i];
  }

  // Return the first steering and throttle actuations
  return { solution.x[delta_start], solution.x[a_start] };
}
