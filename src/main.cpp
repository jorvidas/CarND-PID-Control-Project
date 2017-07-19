/*
Best value so far depending on speed
Max Speed: 40mph
Kp: 0.13
Ki: 0
Kd: 0.75
*/

#include <uWS/uWS.h>
#include "json.hpp"
#include "PID.h"

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

  /*
  * TUNABLE PID PARAMTERS
  */

  // coefficients for steering pid
  // double p[3] = {0.110773, 0.00765538, 0.802286};
  double p[3] = {0.119339, 0.00881286, 0.828008};
  // coefficients for speed pid
  double s[3] = {0.3,0,0.5};
  // dynamic target speed for speed pid
  double desired_speed = 60.0;

  /*
  * OTHER INITIALIZATIONS
  */

  // create and initialize pid objectd for steering (pid) and speed (speed_pid)
  PID pid;
  pid.Init(p);
  PID speed_pid;
  speed_pid.Init(s);

  // true if you want to tune steering paramters
  bool is_twiddle = true;
  // tracks sum(dp) to turn off twiddle
  double total_dp = 1e8;


  h.onMessage([&pid, &total_dp, &is_twiddle, &speed_pid, &desired_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          /*
          * TUNABLE PARAMETERS
          */

          // used to limit steering angles
          // double max_steer = M_PI / 5.0;

          // used by speed_pid for target speed
          double max_speed = 45.0;

          // number of steps before sim restarts with new paramters
          int twiddle_steps = 1525;

          // twiddle continues until sum of dp is less than the twiddle thres
          double twiddle_thres = 0.01;

          // limits how far a car can stray from the center of the road on a
          // successful run
          double max_allowed_cte = 2.5;

          // limits the number of steps before sim will restart and change
          // paramters. this is meant to avoid multiple restarts if messages
          // come in to quickly
          int min_steps = 15;

          // for slowing down during turns
          double min_turn_speed = 35.0;

          // for calculating amount to slow - will target min turn speed
          // at/above this number
          double approx_max_steering_angle = 12.0;


          // DEBUGGING/MANUAL PARAMETER TUNING *** COMMENT OUT WHEN NOT USING ***
          // int steps_to_save = 100;

          /*
          * CALCULATE STEER VALUE
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

          /* 
          * LIMIT STEERING ANGLE
          */

          // if (steer_value > (max_steer) ) {
          //   steer_value = max_steer;
          // } else if (steer_value < -(max_steer) ){
          //   steer_value = -max_steer;
          // }

          /*
          * TWIDDLE
          */

          // filters out parameter sets that go off track but may have better
          // normalized_ss_error and parameter sets that run too long or reset
          // early
          if ((pid.steps > min_steps) && (fabs(cte) > max_allowed_cte) && (is_twiddle)) {
            pid.EarlyReset(twiddle_steps);
            reset_sim(ws);
            pid.steps = 0;
            std::cout<<"Next coefficients: (P) "<<pid.K[0]<<" (I) "<<pid.K[1]
                     <<" (D) "<<pid.K[2]<<std::endl;
            std::cout<< "Total DP :" << total_dp<< std::endl;
            return;
          }

          // sometimes theres an issue with the restart and it gives a low
          // error. this catches those
          if ((pid.steps > (twiddle_steps + 150)) && (is_twiddle)) {
            reset_sim(ws);
            pid.steps = 0;
            std::cout<<"RESTARTED EARLY - NO TWIDDLE"<<std::endl;
          }

          // twiddle after twiddle_steps until sum(dp) < twiddle_thres.
          // set to == to try to avoid additional restarts due to onMessage()
          if ( ((pid.steps == twiddle_steps) && (is_twiddle))) {
            pid.Twiddle();
            total_dp = (std::accumulate(std::begin(pid.dp), std::end(pid.dp), 0.0, std::plus<double>()));
            // stop twiddling and set to best parameters after twiddle_thres is
            // met
            if (total_dp < twiddle_thres) {
              is_twiddle = false;
              pid.Init(pid.best_coeffs);
              std::cout<<"Twiddle turned off. Running with params";
            }
            reset_sim(ws);
            std::cout<<"Next coefficients: (P) "<<pid.K[0]<<" (I) "<<pid.K[1]
                     <<" (D) "<<pid.K[2]<<std::endl;
            std::cout<< "Total DP : " << total_dp<< std::endl;
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
