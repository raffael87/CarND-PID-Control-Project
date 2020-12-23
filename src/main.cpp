#include "PID.h"
#include "json.hpp"
#include "pid_twittle.h"
#include <iostream>
#include <math.h>
#include <string>
#include <uWS/uWS.h>

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

double GetSpeed(double cte, double speed, double angle) {
  constexpr double kMaxSpeed = 60;
  constexpr unsigned int kMaxAngle = 25;

  return std::min(
      kMaxSpeed,
      std::max(0.0, kMaxSpeed * (1.0 - fabs(angle / kMaxAngle * cte) / 4)));
}

int main() {
  uWS::Hub h;

  PID pid_steering;
  pid_steering.Init(0.124, 0.00027, 3.03);

  PID pid_throttle;
  pid_throttle.Init(0.5, 0.0000, 1);

  const bool twittle{false};
  PidTwittle pid_twittle_steering(pid_steering);

  h.onMessage([&pid_twittle_steering, &pid_steering,
               &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data,
                              size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          const double cte = std::stod(j[1]["cte"].get<string>());
          const double speed = std::stod(j[1]["speed"].get<string>());
          const double angle = std::stod(j[1]["steering_angle"].get<string>());

          pid_steering.UpdateError(cte);
          const double steer_value = pid_steering.TotalError();

          double throttle_value = 0.3;
          if (false) {
            const double wanted_speed = GetSpeed(cte, speed, angle);
            pid_throttle.UpdateError(wanted_speed - speed);
            // a bit faster than normal
            std::min(0.5, fabs(pid_throttle.TotalError()));
            std::cout << "wanted speed: " << wanted_speed << std::endl;
            std::cout << "throttle: " << throttle_value << std::endl
                      << std::endl;
          }

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value
          //          << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // TWITTLE
          if (twittle) {
            pid_twittle_steering.UpdateCteError(cte);

            if (pid_twittle_steering.IsVehicleCrashed(cte, speed) ||
                pid_twittle_steering.IsLapDriven()) {
              pid_twittle_steering.Twittle();

              // restart the simulation
              const string msg = "42[\"reset\", {}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          }

        } // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection(
      [/*&h*/](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
      });

  h.onDisconnection([/*&h*/](uWS::WebSocket<uWS::SERVER> ws, int code,
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
