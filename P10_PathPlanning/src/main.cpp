#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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

  int lane = 1; // start in lane 1
  double ref_vel = 0.0; // mph
  bool too_close = false;
  int min_cost_lane = 1;
  bool change_lane_right = true;
  bool change_lane_left = true;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel,
               &too_close, &min_cost_lane, &change_lane_left, &change_lane_right]
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
          // j[1] is the data of JSON object
          //
          // Main car's localization Data (No Noise)
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          //
          // Previous path data given to the Planner
          // Note: 
          //    Return the previous list but with processed points removed, 
          //    can be a nice tool to show how far along the path has processed since last time.
          auto previous_path_x = j[1]["previous_path_x"]; // The previous list of x points previously given to the simulator
          auto previous_path_y = j[1]["previous_path_y"]; // The previous list of y points previously given to the simulator
          //
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"]; // The previous list's last point's frenet s value
          double end_path_d = j[1]["end_path_d"]; // The previous list's last point's frenet d value
          //
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          // ["sensor_fusion"][i][0] = car's unique ID
          // ["sensor_fusion"][i][1] = car's x position in map coordinates
          // ["sensor_fusion"][i][2] = car's y position in map coordinates
          // ["sensor_fusion"][i][3] = car's x velocity in m/s
          // ["sensor_fusion"][i][4] = car's y velocity in m/s
          // ["sensor_fusion"][i][5] = car's s position in frenet coordinates
          // ["sensor_fusion"][i][6] = car's d position in frenet coordinates.
          auto sensor_fusion = j[1]["sensor_fusion"];


          int prev_size = previous_path_x.size();
          int forward_space = 30;

          //double following_car_s = prev_size > 0 ? end_path_s : car_s;
          if (prev_size == 0)
            end_path_s = car_s;

          double costL0 = 0;
          double costL1 = 0;
          double costL2 = 0;
          change_lane_left = true;
          change_lane_right = true;

          for (int i = 0; i < sensor_fusion.size(); ++i)
          {
            if ((double)sensor_fusion[i][5] < (car_s - 30))
              continue;
              
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double)prev_size*.02*check_speed);

            float d = sensor_fusion[i][6];
            if (d < (2+4*lane+2) && d > (2+4*lane-2))
            {
              if (check_car_s > end_path_s && (check_car_s - end_path_s) < forward_space)
                too_close = true;
            }
            else if (check_car_s < end_path_s + 10 && check_car_s > end_path_s - 5)
            {
              if (d > (4*lane+4) && d < (4*lane+8))
                change_lane_right = false;
              else if (d > (4*lane-4) && d < (4*lane))
                change_lane_left = false;
            }
            
            //
            // Calculate Lane cost
            double distance_in_cm = ((double)sensor_fusion[i][5] - car_s)/100.0;
            double forward_distance_cost;
//            double backward_distance_cost;
            if (distance_in_cm >= 0)
              forward_distance_cost = -log(std::tanh(distance_in_cm));
//            else
//              backward_distance_cost = -log(abs(std::tanh(distance_in_cm)));

//            double forward_velocity_cost;
//            double backward_velocity_cost;
//            if (check_speed > car_speed)
//              forward_velocity_cost = sensor_fusion[i][5] < (car_s + 10) ? std::tanh(check_speed) : -1.0;
//            else
//              backward_velocity_cost = sensor_fusion[i][5] > (car_s - 10) ? 1 - std::tanh(check_speed) : -1.0;
            
            if (d < 4)
              costL0 += forward_distance_cost; // + forward_velocity_cost;
            else if (d < 8)
              costL1 += forward_distance_cost; // + forward_velocity_cost;
            else
              costL2 += forward_distance_cost; // + forward_velocity_cost;
          }

          if (costL0 < costL1)
            min_cost_lane = costL0 < costL2 ? 0 : 2;
          else
            min_cost_lane = costL1 < costL2 ? 1 : 2;
          std::cout << " -- Lane: " << costL0 << "\t/ "  << costL1 << "\t/ " << costL2 << "\tLCL:" << change_lane_left << "\tLCR:" << change_lane_right << std::endl;

          if (too_close)
          {
            ref_vel -= .224; // decceralate 5m/s
            too_close = false;
            if (change_lane_left && lane > min_cost_lane && lane-1 >= 0)
              --lane;
            else if (change_lane_right && lane < min_cost_lane && lane+1 <= 3)
              ++lane;
          } 
          else if (ref_vel < 49.5)
            ref_vel += .224;


          
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline 
          // and fill it in with more points that control speed.
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y,yaw states
          // either we will reference the starting point as where the car is 
          // or at the previous paths and point
          double ref_x;
          double ref_y;
          double ref_yaw;

          // Use two points that make the path tangent to the previous path's end point
          if (prev_size < 2)
          {
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            ptsx.push_back(car_x - cos(car_yaw)); // add previous car position x
            ptsy.push_back(car_y - sin(car_yaw)); // add previous car position y
            ptsx.push_back(car_x);                // add current car position x
            ptsy.push_back(car_y);                // add current car position y
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            ref_yaw = atan2(ref_y - (double)previous_path_y[prev_size-2], 
                            ref_x - (double)previous_path_x[prev_size-2]);
            ptsx.push_back(previous_path_x[prev_size-2]);
            ptsy.push_back(previous_path_y[prev_size-2]);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          }
          

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+50, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+70, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i < ptsx.size(); ++i)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points
          // so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner 
          // after filling it with previous points.
          // Here we will always output 50 points.
          for (int i = 1; i <= 50-previous_path_x.size(); ++i)
          {
            double N = target_dist / (.02*ref_vel/2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // We are in local coordinates.
            // So we need to go back to the global coordinates.
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


          json msgJson;
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