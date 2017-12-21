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



clock_t prev_time;
int call_count;
Road road(Speed_Limit, {Speed_Limit, Speed_Limit, Speed_Limit});

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

	road.ego.configure({Speed_Limit, Num_Lanes, 0, 0, Max_Acceleration});

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

						static double prev_v = 0;
						double car_a = (car_v - prev_v) / Timestep;
						if (car_a < -Max_Acceleration) 
							car_a = -Max_Acceleration;
						else if (car_a > Max_Acceleration)
							car_a = Max_Acceleration;
						prev_v = car_v;


						static vector<Vehicle> previous_path;
            int path_size = previous_path_x.size();
						if (path_size == 0)
							cout << "NO PREV QUEUE......................................................................" << endl;

						previous_path.erase(previous_path.begin(), previous_path.begin() + previous_path.size() - path_size);
/*
						if (path_size >= 4)
							path_size = 4;
*/



						cout << endl << "P:U:" << call_count << " [" << car_s << ", " << car_d << ", " << car_v << ", " << car_a 
									<< "], (theta=" << car_theta << ")" << endl;

						vector<double> spline_x;
						vector<double> spline_y;
						int j = 0;
						//cout << "P: ";
            for (j = 0; j < path_size; j++)
            {
								//cout << "[" << previous_path[j].s << ", " << previous_path[j].d << "<" << previous_path[j].intended_lane << ">, " 
								//						<< previous_path[j].v << ", " << previous_path[j].a << ", theta=" 
								//						<< previous_path[j].theta << ", psi=" << previous_path[j].psi << ", delta=" << previous_path[j].delta << "], ";

								next_x_vals.push_back(previous_path_x[j]);
								next_y_vals.push_back(previous_path_y[j]);

								spline_x.push_back(previous_path_x[j]);
								spline_y.push_back(previous_path_y[j]);
            }
						cout << endl;



						int horizon = 50;
						road.populate_traffic(sensor_fusion);

						int cw;
						int nw;
						Vehicle end_path;
						if (path_size == 0) {
							end_path.s = car_s;
							end_path.d = car_d;
							end_path.lane = d2lane(car_d);
							end_path.intended_lane = d2lane(car_d);
							end_path.x = car_x;
							end_path.y = car_y;
							end_path.v = car_v;
							end_path.a = car_a;
							end_path.state = "KL";
							end_path.theta = car_theta;
							cw = ClosestWaypoint(car_x, car_y, map_waypoints_x, map_waypoints_y);
							end_path.psi = car_theta - atan2(map_waypoints_dy[cw], map_waypoints_dx[cw]);
						} else
							end_path = previous_path[path_size - 1];


						road.ego = end_path;
						cw = ClosestWaypoint(road.ego.x, road.ego.y, map_waypoints_x, map_waypoints_y);
						nw = NextWaypoint(road.ego.x, road.ego.y, road.ego.theta, map_waypoints_x, map_waypoints_y);
						road.ego.goal_lane = 1; 
						road.ego.goal_s = map_waypoints_s[nw + 2];
						if ((nw + 2)== 0) {
							road.ego.goal_s = Max_S;
						}


            vector<Vehicle> traj = road.advance(path_size, horizon);

						cout << "P:J:" << call_count << " " << traj[0].state 
										<< " [" << traj[0].s << ", " << traj[0].d << "<" << traj[0].intended_lane << ">, " << traj[0].v << ", " << traj[0].a 
										<< "] -> " << traj[1].state << " [" 
										<< traj[1].s << ", " << traj[1].d << "<" << traj[1].intended_lane << ">, " << traj[1].v << ", " << traj[1].a << "]"
										//<< "... [theta=" << traj[0].theta << ", psi=" << traj[0].psi << ", delta=" << traj[0].delta 
										//<< "] -> [" << traj[1].theta << ", " << traj[1].psi << ", " << traj[1].delta << "]" 
										<< endl;


            double t = 0;

						vector<double> xy;
						double way_s;
						double way_d;
						cout << "P:N:" << call_count << " ";
						for(int i = 1; i <= horizon - path_size; i++)
						{
									traj[i].psi = atan2((traj[i].s - traj[i - 1].s), (traj[i].d - traj[i - 1].d));
									if (traj[i].s > Max_S) {
										traj[i].s -= Max_S;
									}

									way_s = traj[i].s;
									way_d = traj[i].d;
									xy = getXY(way_s, way_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

									traj[i].x = xy[0];
									traj[i].y = xy[1];
									auto cw = ClosestWaypoint(xy[0], xy[1], map_waypoints_x, map_waypoints_y);
									double delta = atan2(map_waypoints_dy[cw], map_waypoints_dx[cw]);
									traj[i].theta = atan2((traj[i].y - traj[i - 1].y), (traj[i].x - traj[i - 1].x));
									traj[i].delta = delta;
									//cout << "[" << way_s << ", " << way_d << ", theta=" << traj[i].theta << ", psi=" 
									//		<< traj[i].psi << ", delta=" << traj[i].delta << "(" << cw << ")], ";

									spline_x.push_back(traj[i].x);
									spline_y.push_back(traj[i].y);
						}

						tk::spline sx;
						tk::spline sy;
						vector<double> spline_t;
						vector<double> spline_x2;
						vector<double> spline_y2;



						spline_t = {0
									, 1
									, (double)(spline_y.size() - 1)};
						spline_x2 = {spline_x[0]
									, spline_x[1]
									, spline_x[spline_x.size()-1]};
						spline_y2 = {spline_y[0]
									, spline_y[1]
									, spline_y[spline_y.size()-1]};

						sx.set_points(spline_t, spline_x2);
						sy.set_points(spline_t, spline_y2);
						for (int i = 0; i < path_size; i++) {
							double x = sx(i); //s(x);
							double y = sy(i); //s(x);

									previous_path[i].x = x;
									previous_path[i].y = y;
									next_x_vals[i] = x;
									next_y_vals[i] = y;
						}
						for (int i = 1; i <= horizon - path_size; i++) {
							traj[i].x = sx(path_size - 1 + i); //s(x);
							traj[i].y = sy(path_size - 1 + i); //s(x);
									previous_path.push_back(traj[i]);
									next_x_vals.push_back(traj[i].x);
									next_y_vals.push_back(traj[i].y);
						}
						path_size = next_x_vals.size();
						if (path_size == 0)
							cout << "NO QUEUE......................................................................" << endl;



						call_count++;
						cout << endl;

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
