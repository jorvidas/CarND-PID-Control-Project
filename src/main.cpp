#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>

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

int main()
{
  // for debugging
  // std::ofstream ofs ("pid_output.txt", std::ios::out | std::ios::trunc);
  // ofs<<"cte "<<"delta_cte "<<"total_cte "<<"p_contrib "<<"d_contrib "<<
  //      "i_contrib "<<"speed "<<"theta_to_desired_traj"<<std::endl<<std::endl;
  // ofs.close();

  uWS::Hub h;

  // dynamic target speed for speed pid - 60.0 is for the first calculation
  // only. this will be a dynamic calculation of speed based on steering angle
  double desired_speed = 60.0;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(0.119339, 0.00881286, 0.828008);

  // create and initialize speed pid for controlling throttle
  PID speed_pid;
  speed_pid.Init(0.3, 0, 0.5);

  h.onMessage([&pid, &speed_pid, &desired_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // debugging
          std::ofstream ofs ("pid_output.txt", std::ios::out | std::ios::app);

          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double speed_error;
          double throttle;

          /*
          * TUNABLE PARAMETERS
          */

          // used by speed_pid for target speed
          double max_speed = 45.0;

          // for slowing down during turns
          double min_turn_speed = 35.0;

          // for calculating amount to slow - will target min turn speed
          // at/above this number
          double approx_max_steering_angle = 12.0;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          /*
          * CALCULATE THROTTLE
          */

          // slow down on turns
          desired_speed = max_speed - (max_speed - min_turn_speed) *
          fabs(angle/approx_max_steering_angle);
          // dont go below min speed if angle is larger than approx max
          if (desired_speed < min_turn_speed) {desired_speed = min_turn_speed;}      
          speed_error = speed - desired_speed;
          speed_pid.UpdateError(speed_error);
          throttle = speed_pid.TotalError();
          // release gas instead of breaking if you go slightly over max_speed
          // still break if steering angle requires slowing down
          if ( (throttle < 0.0) && (throttle > (3*s[1])) ) {throttle = 0.0;}
          
          // // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          // ofs<<"\r\n"<<cte<<" "<<pid.d_error<<" "<<pid.i_error<<" "<<cte*pid.Kp<<" "<<
          //      pid.d_error*pid.Kd<< " "<<pid.i_error*pid.Ki<< " "<<speed;
          // ofs.close();

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
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
