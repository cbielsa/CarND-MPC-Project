## IMPLEMENTATION

The source code for this project is comprised by:
  * `MPC.h`/`MPC.cpp`, which declare and define a class MPC with the functionality of a Model Predictive Controller based on a Kinematic Bicycle Model.
  * main.cpp, with the implementation of the binary that drives the car in the simulator.

## THE MODEL

The state of the vehicle is given by:
  * `x`, `y`: position of the vehicle CoM in the global frame.
  * `psi`: orientation of the vehicle in [rad], measured wrt. the global x-axis and positive for rotations around cross(x,y).
  * `v`: vehicle speed norm [mpg].
  * `cte`: cross-track error, or distance from the vehicle CoM to the reference path given by the simulator.
  * `epsi`: difference between the vehicle orientation and the orientation of the reference path at the vehicle position, positive for rotational errors around cross(x,y).

The two errror components ´cte´ and ´epsi´ are added to the state vector so that they can be used in the cost function used by the optimizer.

The two actuators with which the MPC controls the movement of the vehicle in the simulator are:
  * `delta`: the steering wheel angle [rad], null if the front wheels are aligned with x-vehicle and positive for rotations around cross(x,y).
  * `a`: the throttle acceleration [m/s2], between -1 and 1, positive for acceleration, negative for brake.
  
The bicycle model:
  * the state update equations are given in the form of constraints that tie together the state at a given time step with the state at the following time step. This is implemented in lines `133` to `138` of `MPC.cpp`. 
  * note that a 3rd order polynomial `f(x) = a + b x + c x^2 + d x^3` is fit to the waypoints, hence the desired vehicle orientation at a given x-position is given by `atan( b + 2c x + 3d x^2 )`, as computed in code line ´129´.

## TIMESTEP LENGTH AND ELAPSED DURATION

The timestep length and duration is set in lines `9` and `10` of `MPC.cpp`.

A prediction horizon of 2 seconds is chosen, long enough to have the MPC plan for upcoming curvature changes but not longer than necessary.
This 2 seconds are split in `N = 20` cycles of `dt = 0.1` s duration. Higher control frequency did not seem to bring noticeable gains in performance, but did however increase the computational time of the optimizer, so I settled for 100 ms step duration.
The MPC also performs well with a prediction horizon of 1 second, but increasing it to 2 seconds allows the controller to better plan for the upcoming path.

## MODEL PREDICTIVE CONTROL WITH LATENCY

Lines `128` to `131` in `main.cpp` propagate the vehicle state read in the telemetry from the simulator for a duration equal to the nominal latency of 100 milliseconds (note that the 100 ms latency is artificially added to the pipeline in line `219` of `main.cpp`).
For the propagation, I use the kinematic bicycle model, taking last cycle steering angle and acceleration as actuations. This propagation updates `x`, `y`, `psi` and `v`. State errors `cte` and `epsi` after accounting for latency are calculated in source code lines `156` and `163`, respectively. The calculation of both state errors requires a polynomial fit to the waypoints, in vehicle frame, which is explained in the next section of this writeup. 
The entire vehicle state after correcting for latency is put together in line `166`, and is an input to the call to `MPC::Solve()` in line `171`.
The MPC modeling of latency is critical for the controller performance in the simulator. In fact, before implementing the latency correction, a mere 100 ms latency was sufficient to drive the vehicle off-road. With the correction for latency in the simulator, however, the performance is essentially as good as without latency.

## POLYNOMIAL FITTING AND MPC PREPROCESSING

The waypoint coordinates `ptsx` and `ptsy` extracted from the telemetry from the simulator are given in the global simulator frame. However, to compute `cte`and `epsi`, as well as plot the reference path in the simulator, the waypoints are needed in vehicle frame (the vehicle frame, attached to the vehicle, has origin on vehicle CoM, x pointing forward and y, leftwards). The transformation (translation and rotation) of the waypoints from global to vehicle frame is done in lines `133` to `143` of `main.cpp`. Once in vehicle frame, a 3rd order polynomial `coeffs` is fit to the waypoints in line `146`.
This is the `coeffs` variable passed to `MPC::Solve()` at each cycle, which solves the optimization problem in a frozen frame equal to the vehicle frame at the `cycle time + latency`.
If `f(x)` is the polynominal given by `coeffs`, the `cte` for a given vehicle position `{x,y}` is calculated as `-f(x)`, while the `epsi` is calculated as `-atan(f'(x))`. This assumes that at the cycle start time, the nearest point from the vehicle to the reference path is along the vehicle y-axis. Note that this is not the case if, at the cycle start time, the vehicle is not aligned with the reference path. However, the assumption simplifies the calculations and proves to be adequate for small values of `epsi`, which we manage to maintain during the entire simulator track. 

## SIMULATION AN CONCLUSIONS

Check out this video capture of the MPC driving the car at high speeds in the simulator.

https://www.youtube.com/watch?v=d2TrMWJslzs&feature=youtu.be

In particular, notice how the controller reduces speeds before and during sharp curves. Note that no dedicated code has been written to achieve this behaviour. The behaviour is the natural result of the choice of the cost function minimized by the MPC, which puts a penalty to cross-track and heading errors that during sharp curves exceeds the penalty for low speeds, hence the controller chooses to drive at high speeds during straight stretches of road and at low speeds during sharp curves.
With a redefined cost funciton, this MPC could also be used to autonomously park a car.
