/*
Best value so far depending on speed
Max Speed: 40mph
Kp: 0.13
Ki: 0
Kd: 0.75
*/



#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>
#include <numeric>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// DEBUGGING/MANUAL PARAMETER TUNING *** COMMENT OUT WHEN NOT USING ***
// std::ofstream ofs;

// for resetting sim
void reset_sim (uWS::WebSocket<uWS::SERVER> webs) {
  // reset sim
  std::string msg = "42[\"reset\",{}]";
  webs.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
  // DEBUGGING/MANUAL PARAMETER TUNING *** COMMENT OUT WHEN NOT USING ***
  // ofs.open("pid_output.txt", std::ios::out);
  // ofs<<"cte,"<<"delta_cte,"<<"total_cte,"<<"p_contrib,"<<"d_contrib,"<<
  //      "i_contrib,"<<"speed,"<<"theta_to_desired_traj,";

  uWS::Hub h;

  // create and initialize pid object for steering
  PID pid;
  double p[3] = {0.11345, 0.0064979, 0.77021};
  pid.Init(p);
  double total_dp = 1e8;
  bool is_twiddle = true;

  h.onMessage([&pid, &total_dp, &is_twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double speed_error;
          double throttle;
          PID speed_pid;                  // only using P, doesn't need memory

          // additional parameters
          // double max_steer = M_PI / 5.0;
          double desired_speed = 40.0;
          double twiddle_steps = 1525;
          double twiddle_thres = 0.005;
          double max_allowed_cte = 2.2;
          double min_steps = 15;

          // DEBUGGING/MANUAL PARAMETER TUNING *** COMMENT OUT WHEN NOT USING ***
          // int steps_to_save = 100;
          if ((pid.steps > min_steps) && (fabs(cte) > max_allowed_cte) && (is_twiddle)) {
            pid.EarlyReset(twiddle_steps);
            reset_sim(ws);
            pid.steps = 0;
            std::cout<<"Next coefficients: (P) "<<pid.K[0]<<" (I) "<<pid.K[1]
                     <<" (D) "<<pid.K[2]<<std::endl;
            std::cout<< "Total DP:" << total_dp<< std::endl;
            // std::cout<< "Errors: (P) "<< pid.p_error<<" (I) "<<pid.i_error<<" (D) "<<pid.d_error<<std::endl;
            return;
          }
          // calculate steer_value
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          // calculate throttle
          double s[3] = {1,0,0};
          speed_pid.Init(s);
          speed_error = speed - desired_speed;
          speed_pid.UpdateError(speed_error);
          throttle = speed_pid.TotalError();

          // limit steering value
          // if (steer_value > (max_steer) ) {
          //   steer_value = max_steer;
          // } else if (steer_value < -(max_steer) ){
          //   steer_value = -max_steer;
          // }

          //twiddle
          if ((pid.steps == twiddle_steps) && (is_twiddle)) {
            pid.Twiddle();
            total_dp = (std::accumulate(std::begin(pid.dp), std::end(pid.dp), 0.0, std::plus<double>()));
            if (total_dp < twiddle_thres) {
              is_twiddle = false;
              pid.Init(pid.best_params);
              cout<<"Twiddle turned off. Running with params"
            }
            reset_sim(ws);
            std::cout<<"Next coefficients: (P) "<<pid.K[0]<<" (I) "<<pid.K[1]
                     <<" (D) "<<pid.K[2]<<std::endl;
            std::cout<< "Total DP:" << total_dp<< std::endl;
            // std::cout<< "Errors: (P) "<< pid.p_error<<" (I) "<<pid.i_error<<" (D) "<<pid.d_error<< std::endl;
          }

          // DEBUGGING/MANUAL PARAMETER TUNING *** COMMENT OUT WHEN NOT USING ***
          // int steps = 0;
          // if (steps < steps_to_save) {
          //   ofs<<"\r\n"<<cte<<" "<<pid.d_error<<" "<<pid.i_error<<" "<<
          //        cte*pid.Kp<<" "<<pid.d_error*pid.Kd<< " "<<
          //        pid.i_error*pid.Ki<< " "<<speed;
          // } else if (steps == 100){
          //   for (int i = 0; i < 25; i++) {
          //     std::cout<<"____________________________________________________________________\n";
          //   }
          //   ofs.close();
          // }
          // steps++;
          
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
