#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <ctime>
#include <signal.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "vehicle.h"
#include "road.h"
#include "constants.h"
#include "helpers.h"
#include "behavior.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}



Behavior behavior;

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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


	behavior.road.set_map(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
						double car_yaw = j[1]["yaw"]; // degree
						double car_speed = j[1]["speed"]; // mph

          	// Previous path data given to the Planner
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
						//  [ id, x, y, vx, vy, s, d]
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	// DONE: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

						double car_v = mph2mps(car_speed);
						double car_theta = deg2rad(car_yaw);

						int prev_size = previous_path_x.size();
						behavior.road.ego.update(car_x, car_y, car_s, car_d, car_theta, car_v);
						behavior.road.populate_traffic(sensor_fusion);
						int horizon = 50;
						if (prev_size == 0)
							end_path_s = car_s;
						behavior.plan_path(prev_size, end_path_s);

						double ref_x;
						double ref_y;
						double ref_x_prev;
						double ref_y_prev;
						double ref_theta;

						vector<double> ptsx;
						vector<double> ptsy;

						if (prev_size < 4) {
							ref_x = car_x;
							ref_y = car_y;

							ref_theta = car_theta;
							ref_x_prev = ref_x - cos(ref_theta);
							ref_y_prev = ref_y - sin(ref_theta);

							ptsx.push_back(ref_x_prev);
							ptsx.push_back(ref_x);

							ptsy.push_back(ref_y_prev);
							ptsy.push_back(ref_y);
						} else {
							
							ref_x = previous_path_x[prev_size-2];
							ref_y = previous_path_y[prev_size-2];

							ref_x_prev = previous_path_x[prev_size-3];
							ref_y_prev = previous_path_y[prev_size-3];

							ref_theta =  atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

							ptsx.push_back(ref_x_prev);
							ptsx.push_back(ref_x);

							ptsy.push_back(ref_y_prev);
							ptsy.push_back(ref_y);

							ref_x = previous_path_x[prev_size-1];
							ref_y = previous_path_y[prev_size-1];
							ptsx.push_back(ref_x);
							ptsy.push_back(ref_y);
						}


						for (int i = 2; i <= 3 ; i ++) {
							vector<double> next_point = getXY(car_s + 30 * i
																								, lane2d(behavior.lane)
												, map_waypoints_s, map_waypoints_x, map_waypoints_y);
							ptsx.push_back(next_point[0]);
							ptsy.push_back(next_point[1]);
						}

					 	for (int i = 0; i < ptsx.size(); i++) {
							double shift_x = ptsx[i] - ref_x;
							double shift_y = ptsy[i] - ref_y;

							ptsx[i] = (shift_x * cos(0-ref_theta) - shift_y * sin(0-ref_theta));
							ptsy[i] = (shift_x * sin(0-ref_theta) + shift_y * cos(0-ref_theta));
						}


						for (int i = 0; i < prev_size; i++){
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
						}

						tk::spline s;
						s.set_points(ptsx, ptsy);

						double target_x = 30.0;
						double target_y = s(target_x);
						double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

						double x_add_on = 0;

						double v_increment = (behavior.target_speed - car_v)/(horizon - prev_size);
						double v = car_v + v_increment;
						for (int i = 0; i < horizon - prev_size; i++) {

							double N = (target_dist/(Timestep * v));
							double x_point = x_add_on + (target_x)/N;
							double y_point = s(x_point);

							x_add_on = x_point;

							double x_ref = x_point;
							double y_ref = y_point;

							x_point = (x_ref * cos(ref_theta) - y_ref * sin(ref_theta));
							y_point = (x_ref * sin(ref_theta) + y_ref * cos(ref_theta));

							x_point += ref_x;
							y_point += ref_y;

							next_x_vals.push_back(x_point);
							next_y_vals.push_back(y_point);

							v += v_increment;
							if (v > Speed_Limit)
								v = Speed_Limit;
						}

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

						string msgJson_dump = msgJson.dump();
          	auto msg = "42[\"control\","+ msgJson_dump+"]";
          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
