#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
//#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "pathplanning.h"
#include "map.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  Map map;

  double ref_vel = 0.0; // miles per hour
  int ref_lane = 1;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &ref_lane, &map]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          CarState car = {j[1]["x"],
                          j[1]["y"],
                          j[1]["s"],
                          j[1]["d"],
                          j[1]["yaw"],
                          j[1]["speed"]};

          // Previous path data given to the Planner
          Path previous_path = {j[1]["previous_path_x"], j[1]["previous_path_y"]};

          // Previous path's end s and d values 
          double end_s= j[1]["end_path_s"];
          double end_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          bool road_blocked = false;
          std::cout << "===" << std::endl;
          for(int i=0; i < sensor_fusion.size(); ++i) {
            float vx = sensor_fusion[i][3];
            float vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double) 0.02 * check_speed * previous_path.size());
            float d = sensor_fusion[i][6];

            std::cout << "d = " << d << std::endl;
            // if car is on our lane

            // @TODO should use end_d instead
            if(((ref_lane*4 + 2) - 2) < d && ((ref_lane*4 + 2) +2) > d) {
              if ((check_car_s > end_s) && (abs(check_car_s - end_s) < 30)) {
                  road_blocked = true;
                  //ref_vel = 29.5;

                  std::cout << "found blocking car on lane " << (d/4) << " distance " << check_car_s << std::endl;
              }
            }
          }
          
          if (road_blocked) {
            ref_vel -= 0.244;
          } else {
            if(ref_vel < 49.5) {
              ref_vel += 0.244;
            }
          }

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          Path res_path = GeneratePath(car, previous_path, 1, ref_vel, map);

          msgJson["next_x"] = res_path.getX();
          msgJson["next_y"] = res_path.getY();

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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