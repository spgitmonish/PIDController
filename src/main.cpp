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
  pidSteer.Init(0.05, 0.0001, 0.75, STEERING);
  pidThrottle.Init(0.05, 0.0001, 0.75, THROTTLE);
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

          // Create a factor which decides the speed error based on the base
          // of 20 miles an hour. If the car is driving straight i.e. the
          // steering angle is low then we can go faster but if the steering
          // angle is smaller, then the car needs to slow down because the car
          // is most likely turning.
          double new_speed = 5 * (1.0 - abs(steer_value)) + 25;
          double speed_error = speed - new_speed;

          // Update the total error for this iteration
          pidThrottle.TotalError(speed_error);

          // Calculate the throttle value
          throttle_value =  - (pidThrottle.Kp * pidThrottle.p_error) \
                            - (pidThrottle.Ki * pidThrottle.i_error) \
                            - (pidThrottle.Kd * pidThrottle.d_error);

        #if DEBUG_VERBOSE
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
        #endif

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
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
