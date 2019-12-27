#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "math.h"

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
   
  double auto_speed = 0.0; // vehicle speed
  bool change_lane = false;
  int lane_id = 1;
  
  h.onMessage([&auto_speed,&change_lane,&lane_id,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"]; //MPH

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          /**
           * Done: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Initialize constants
          const double MPH_to_mps = 0.447;
          const double safety_distance = 20.0;
          const double spline_distance = 35.0;
          const double lane_width = 4.0; 
          const double delta_time = 0.02; 
          const double max_speed = 45; 
          const double max_acceleration = .224;
          // Initialize variables
          vector<double> next_x_vals, next_y_vals, spline_x_vals, spline_y_vals, next_point;  
          double angle, pos_x, pos_y, pos_x2, pos_y2, shift_x, shift_y, spline_x, spline_y, next_x, next_y;
          double goal_speed = max_speed;
          double distance_increment = 0.2;
          double front_car_speed, delta_speed, x_further;
          int path_size = previous_path_x.size();
          int map_size = map_waypoints_x.size();
          int start_point;
          bool should_match_speed = false;
          bool is_left_clear = true;
          bool is_right_clear = true;
          tk::spline s;
          
          // analyze traffic situation
          // Get car speed
          car_speed = fabs(car_speed);
          
          // loop over all detected cars from sensor fusion
          for (int i = 0; i < sensor_fusion.size(); ++i){
            // check to see if there are other cars in ego car's lane slower than ego car's speed
            // check if the car is in the same lane
            if ((lane_id*lane_width < sensor_fusion[i][6]) && (sensor_fusion[i][6] < (lane_id+1)*lane_width)) {
              // check if the car is in front is closer than the end of the previous path plus a safety distance             
              if ((sensor_fusion[i][5] < (end_path_s + safety_distance)) && (sensor_fusion[i][5] > (car_s - 5.0))) {
                // calculate other cars absolute speed             
                front_car_speed = std::sqrt(pow(sensor_fusion[i][3],2.0) + pow(sensor_fusion[i][4],2.0) ); 
                const double front_car_speed_in_mph = front_car_speed* 2.23694;
                // calculate the diference in speed between the two cars
                delta_speed = car_speed - front_car_speed_in_mph;
                // if the car is going slower than ego car's speed, match its speed
                if (delta_speed > 0.0){
                  goal_speed = front_car_speed_in_mph;
                  should_match_speed = true;
                } 
              }
            }
            
            // check for cars at left lane in case of switching lanes
            if ((lane_id > 0) && ((lane_id - 1)*lane_width < sensor_fusion[i][6]) && (sensor_fusion[i][6] < lane_id*lane_width)) {
              if ((sensor_fusion[i][5] < (end_path_s + safety_distance)) && (sensor_fusion[i][5] > car_s - safety_distance/2)) { 
                is_left_clear = false;
              }            
            }
            
            // check for cars at right lane in case of switching lanes        
            if ((lane_id < 2) && ((lane_id+1)*lane_width < sensor_fusion[i][6]) && (sensor_fusion[i][6] < (lane_id+2)*lane_width) ) {
              if ((sensor_fusion[i][5] < (end_path_s + safety_distance)) && (sensor_fusion[i][5] > car_s - safety_distance/2)) {
                is_right_clear = false;
              } 
            }            
          }        
          
          // if the car in front slowed ego car down too much, change lanes if a lane change is not already in progress
          if ((should_match_speed == true) && (change_lane == false)) {
            change_lane = true;
          }
          // Changing lanes and always preferring left
          if(car_speed < (.9 * max_speed) && change_lane == true) {
            if (lane_id > 0 && is_left_clear == true) {
              lane_id = lane_id - 1;
              change_lane = false;
            } else if (lane_id < 2 && is_right_clear == true) {
              lane_id = lane_id + 1;
              change_lane = false;
            } 
          }
          
          
          // Using previous path points
          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // if path is empty, get the ego car's current position and angle
          if (path_size == 0) {
            // get car's current position
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            pos_x2 = pos_x - distance_increment * cos(angle);
            pos_y2 = pos_y - distance_increment * sin(angle);
            spline_x_vals.push_back(pos_x2);
            spline_y_vals.push_back(pos_y2);
            spline_x_vals.push_back(pos_x);
            spline_y_vals.push_back(pos_y);
          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];

            pos_x2 = previous_path_x[path_size-2];
            pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            
            spline_x_vals.push_back(pos_x2);
            spline_y_vals.push_back(pos_y2);
            spline_x_vals.push_back(pos_x);
            spline_y_vals.push_back(pos_y);
          }          
              
          // calculate spline points ahead of ego vehicle, spaced by spline_distance
          for (unsigned int i = 1; i <= 3; i++) {
            next_point = getXY(car_s + i*spline_distance,(lane_width/2 + lane_width*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            spline_x_vals.push_back(next_point[0]);
            spline_y_vals.push_back(next_point[1]);
          }
          
          // transform spline points from map coordinates to car coordinates
          for (int i = 0; i < spline_x_vals.size(); ++i) {
            shift_x = spline_x_vals[i] - pos_x;
            shift_y = spline_y_vals[i] - pos_y;
            spline_x_vals[i] = shift_x * cos(0 - angle) - shift_y * sin(0 - angle);
            spline_y_vals[i] = shift_x * sin(0 - angle) + shift_y * cos(0 - angle);
          }         
          // compute spine
          s.set_points(spline_x_vals,spline_y_vals);       
               
       
          // approach the goal speed which is either max speed or the speed of a slower car in front
          if ((car_speed > goal_speed) && (car_speed > 0.0)) {
            auto_speed -= max_acceleration;
          } else if ((car_speed < goal_speed) && (goal_speed <= max_speed)) {
            auto_speed += max_acceleration;            
          }
          
          // calculate distance increments based on desired velocity
          distance_increment = auto_speed * delta_time;  
          
          shift_x = 0.0;
          //calculate car trajectory points
          for (int i = 0; i < 30-path_size; ++i) {    
            // advance distance increment meters down the road        
            shift_x += distance_increment;
            // use spline to calculate y
            shift_y = s(shift_x);
            // transform back to map coordinates
            next_x = shift_x * cos(angle) - shift_y * sin(angle);
            next_y = shift_x * sin(angle) + shift_y * cos(angle);
            next_x += pos_x;
            next_y += pos_y;
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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
    //debugfile.close();
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