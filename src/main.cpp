#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

double twiddle_best_error_ = 1000;
bool twiddle_state_ = 0;
int twiddle_idx = 0;
std::vector<double> p = {0.2, 0.004. 3.0};
std::vector<double> dp = {0.001, 0001, 0.001};


int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  // What are the best coefficients?!?!??!?!?!?!?!? Implement twiddle algo.
  pid.Init(0.2, 0.004, 3.0);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = - pid.TotalError();
          if (steer_value > 1.0) {
            steer_value = 1.0;
          }
          if (steer_value < -1.0) {
            steer_value = -1.0;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          // Twiddle
          // Implemented as a state-machine:
          //
          // python twiddle loop
          // while sum(dp) > tol:
          //   for i in range(len(p)):
          //     p[i] += dp[i]
          //     _, _, err = run(robot, p)
          //     if err < best_err:
          //         best_err = err
          //         dp[i] *= 1.1
          //     else:
          //         p[i] -= 2.0 * dp[i]
          //         _, _, err = run(robot, p)
          //         if err < best_err:
          //             best_err = err
          //             dp[i] *= 1.1
          //        else:
          //             p[i] += dp[i]
          //             dp[i] *= 0.9

          if (pid.TotalError() > twiddle_best_error_) {
            if (twiddle_state_ == 0) {
              twiddle_best_error_ = pid.TotalError();
              p[twiddle_idx] += dp[twiddle_idx];
              pid.Init(p[0], p[1], p[2]);
              twiddle_state_ = 1;
            } else if (twiddle_state_ == 1) {
              if (pid.TotalError() < twiddle_best_error_) {
                twiddle_best_error_ = pid.TotalError();
                dp[twiddle_idx] *= 1.1;
                twiddle_idx = (twiddle_idx+1)%3; //rotate over the 3 vector indices
                p[twiddle_idx] += dp[twiddle_idx];
                twiddle_state_ = 1;
                pid.Init(p[0], p[1], p[2]);
              } else {
                p[twiddle_idx] -= 2 * dp[twiddle_idx];
                twiddle_state_ = 2;
                pid.Init(p[0], p[1], p[2]);
              }
            } else { //twiddle_state_ = 3
              if (pid.TotalError() < twiddle_best_error_) {
                twiddle_best_error_ = pid.TotalError();
                dp[twiddle_idx] *= 1.1;
                twiddle_idx = (twiddle_idx+1)%3;
                p[twiddle_idx] += dp[twiddle_idx];
                twiddle_state_ = 1;
                pid.Init(p[0], p[1], p[2]);
              } else {
                p[twiddle_idx] += dp[twiddle_idx];
                dp[twiddle_idx] *= 0.9;
                twiddle_idx = (twiddle_idx+1)%3;
                p[twiddle_idx] += dp[twiddle_idx];
                twiddle_state_ = 1;
                pid.Init(p[0], p[1], p[2]);
              }
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
