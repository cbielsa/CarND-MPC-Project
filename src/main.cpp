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


// Auxiliary function for communication with simulator ==========================

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


// Polynomial interpolation ====================================================

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


// Main program =============================================================

int main() {

  uWS::Hub h;

  // actuator latency, in [s]
  const double latency = 0.100;  // [s]

  // This value assumes the model presented in the classroom is used.
  //
  // It was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG that has a similar radius.
  const double Lf = 2.67;

  // Actuations in last cycle, used to propagate state during latency
  double lastDelta = 0.;
  double lastA = 0.;

  // Construct MPC
  MPC mpc(Lf);

  h.onMessage([&mpc, &latency, &Lf, &lastDelta, &lastA] (
    uWS::WebSocket<uWS::SERVER> ws, char *data,
    size_t length, uWS::OpCode opCode ) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;

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

          // propagate vehicle state during actuator latency
          px += v*cos(psi)*latency;
          py += v*sin(psi)*latency;
          psi += lastDelta*v*latency/Lf;
          v += lastA*latency;

          // transform reference trajectory to vehicle frame
          Eigen::VectorXd ptsx_veh(ptsx.size());
          Eigen::VectorXd ptsy_veh(ptsy.size());

          double sin_psi = sin(psi);
          double cos_psi = cos(psi);

          for( size_t i=0; i<ptsx.size(); ++i ) {  
            ptsx_veh[i] =  (ptsx[i]-px)*cos_psi + (ptsy[i]-py)*sin_psi;
            ptsy_veh[i] = -(ptsx[i]-px)*sin_psi + (ptsy[i]-py)*cos_psi;
          }

          // fit waypoints in vehicle frame to 3rd order polynomial
          Eigen::VectorXd coeffs = polyfit(ptsx_veh, ptsy_veh, 3);

          // calculate initial state in vehicle frame,
          // (after propagation during actuator latency)
          Eigen::VectorXd x0(6);

          // initial cross track error is the distance from the vehicle
          // of the point where the reference trajectory intersects
          // the vehicle y-axis (assuming the reference trajectory is
          // almost parallel to the vehicle x-axis at that point)
          double cte = -polyeval(coeffs, 0.);

          // initial orientation error is the angle between the vehicle
          // x-axis and the direction tangent to the reference trajectory
          // where the reference trajectory intersects the vehicle y-axis
          //
          // note that at x=0, if f(x) is the ref. trajectory, df/dx is coeff[1]
          double epsi = -atan(coeffs[1]);

          // in vehicle frame, x, y and psi are all null
          x0 << 0., 0., 0., v, cte, epsi;

          // solve optimization problem in vehicle frame
          vector<double> x_pred;
          vector<double> y_pred;
          auto vActuations = mpc.Solve(
            x0, coeffs,
            x_pred, y_pred );

          // update actuations
          lastDelta = vActuations[0];
          lastA = vActuations[1];

          // actuation values sent to the simulator

          // we define anti-clockwise steering angles as postive,
          // but the simulator seems to follow the opposite convention,
          // hence the minus
          double steer_value = -vActuations[0]/0.436332;
          double throttle_value = vActuations[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = x_pred;
          vector<double> mpc_y_vals = y_pred;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals(ptsx_veh.data(), ptsx_veh.data()+ptsx_veh.size());
          vector<double> next_y_vals(ptsy_veh.data(), ptsy_veh.data()+ptsy_veh.size());

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(1000*static_cast<int>(latency)));
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
