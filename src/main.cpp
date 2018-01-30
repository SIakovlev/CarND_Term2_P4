#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

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

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: done
  // Initialize the pid variable.

  std::vector<double> ct_error;

  std::vector<double> data_steering_clipped;
  std::vector<double> data_steering;
  std::vector<double> data_cte;
  std::vector<std::vector<double>> errors(3);
  std::vector<double> counts;
  bool stop_flag = false;

  pid.Init(0.05, 0.02, 0.1);

  h.onMessage([&pid, &data_steering, &data_steering_clipped, &counts, &stop_flag, &errors, &data_cte](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          const double steer_value_max = 1;
          const double steer_value_min = -1;

          static auto finish = std::chrono::high_resolution_clock::now();
          static auto start = finish;

          /*
          * TODO: done
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          static double counter = 0;

          finish = std::chrono::high_resolution_clock::now();
          if (counter) {
            std::chrono::duration<double> dt = finish - start;
            pid.UpdateError(cte, dt.count());
          }
          
          std::cout << "Coeffs: Kp = " << pid.K[0] << ", Ki = " << pid.K[1] << ", Kd = " << pid.K[2] << std::endl;

          double action = pid.Action();
          steer_value = clip(action, steer_value_min, steer_value_max);

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // start timer
          start = std::chrono::high_resolution_clock::now();
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          // Gather visualisation data
          
          
          counts.push_back(counter);
          ++counter;
          errors[0].push_back(pid.error[0]);
          errors[1].push_back(pid.error[1]/pid.K[1]);
          errors[2].push_back(pid.error[2]);
          data_steering.push_back(action);
          data_steering_clipped.push_back(steer_value);
          data_cte.push_back(cte);

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        /*
        plt::subplot(1, 1, 1);
        plt::named_plot("Cross track error", counts, data_cte, "r");
        plt::grid(true);
        plt::show();

        
        plt::subplot(1, 1, 1);
        plt::named_plot("PID output (steering angle)", counts, data_steering, "--r");
        plt::named_plot("Clipped steering angle", counts, data_steering_clipped, "k");
        plt::legend();
        plt::grid(true);
        plt::show();
        
        plt::subplot(2, 1, 1);
        plt::named_plot("Kp error", counts, errors[0], "r");
        plt::named_plot("Ki error", counts, errors[1], "k");
        plt::named_plot("Kd error", counts, errors[2], "b");
        plt::legend();
        plt::grid(true);
        plt::subplot(2, 1, 2);
        plt::named_plot("Clipped steering angle", counts, data_steering_clipped, "--k");
        plt::legend();
        plt::grid(true);
        plt::show();

        stop_flag = true;*/
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
  if (stop_flag) return 0;
}
