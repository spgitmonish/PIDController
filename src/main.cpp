#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// For convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data, the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // PID object for controlling steering angle
  PID pidSteer;

  // PID object for controlling throttle
  PID pidThrottle;

  // Initialize the pid variable with the values for the constants of
  // the respective components
#if SGD
  pidSteer.Init(0, 0, 0, STEERING);
  pidThrottle.Init(0, 0, 0, THROTTLE);
#elif TWIDDLE
  pidSteer.Init(0.10, 0.0001, 0.75, STEERING);
  pidThrottle.Init(0.07, 0.0001, 0.5, THROTTLE);
#endif

  h.onMessage([&pidSteer, &pidThrottle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "")
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;

          // Update the total error for this iteration
          pidSteer.TotalError(cte);

          // Calculate the steer value
          steer_value = - (pidSteer.Kp * pidSteer.p_error) \
                        - (pidSteer.Ki * pidSteer.i_error) \
                        - (pidSteer.Kd * pidSteer.d_error);

          // If the steering angle is beyond +/- 1 limit the steering angle
          // to +/- 0.5 to prevent sharp turns
          if(steer_value < -1)
          {
            steer_value = -0.5;
          }
          else if(steer_value > 1)
          {
            steer_value = 0.5;
          }

          // If the cross track error is small then the steering left to do is
          // small, so if the steering angle is high, the target speed needs to
          // be lower. So Speed is the opposite of the steering angle, larger the
          // angle the lower the speed needs to be. Assume the target speed is
          // 15 miles an hour.
          cout << "S: " << speed << endl;

          double target_speed = speed - 30 *(1.-abs(steer_value)) + 20;

          cout << "T: " << target_speed << endl;

          // Update the total error for this iteration
          pidThrottle.TotalError(speed - target_speed);

          // Calculate the throttle value
          throttle_value =  pidThrottle.Kp * pidThrottle.p_error + \
                            pidThrottle.Ki * pidThrottle.i_error + \
                            pidThrottle.Kd * pidThrottle.d_error;

        #if DEBUG_VERBOSE && 0
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
        #endif

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          //msgJson["throttle"] = 0.15;
          msgJson["throttle"] = -throttle_value;
          /*if(throttle_value < 0.099)
          {
            msgJson["throttle"] = throttle_value * 10;
          }
          else if((throttle_value >= 0.25) && (throttle_value <= 0.3))
          {
            // Lower the speed
            throttle_value = 0.2;
            msgJson["throttle"] = throttle_value;
          }
          else
          {
            msgJson["throttle"] = throttle_value;
          }*/
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
        #if DEBUG_VERBOSE
          std::cout << msg << std::endl;
        #endif
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // NOTE: We don't need this since we're not using HTTP
  //       but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
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
