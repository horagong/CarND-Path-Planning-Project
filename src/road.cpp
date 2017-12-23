#include <iostream>
#include "road.h"
#include "vehicle.h"
#include "constants.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>

using namespace std;
using json = nlohmann::json;
/**
 * Initializes Road
 */
Road::Road() {
}

Road::~Road() {
}

void Road::set_map(vector<double> map_waypoints_x, vector<double> map_waypoints_y
		, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) {
	this->map_waypoints_x = map_waypoints_x;
	this->map_waypoints_y = map_waypoints_y;
	this->map_waypoints_s = map_waypoints_s;
	this->map_waypoints_dx = map_waypoints_dx;
	this->map_waypoints_dy = map_waypoints_dy;
}


void Road::populate_traffic(json fusion) {
	
  map<int, Vehicle>::iterator it;

  for (auto& element : fusion) {
		// [ id, x, y, vx, vy, s, d]
    int vehicle_id = element[0].get<int>();
    double x = element[1].get<double>();
    double y = element[2].get<double>();
    double vx = element[3].get<double>();
    double vy = element[4].get<double>();
    double s = element[5].get<double>();
    double d = element[6].get<double>();

    int lane = d2lane(d);

    if (lane >=0 && lane <=2) {
      it = vehicles.find(vehicle_id);
      if (it != vehicles.end()) {
        Vehicle vehicle = it->second;
        vehicle.update(s, d, vx, vy);
        it->second = vehicle;
      } else {
        Vehicle vehicle = Vehicle(s, d, vx, vy);
        vehicles.insert(std::pair<int, Vehicle>(vehicle_id, vehicle));
      }
    }
  }

	vehicles_added = vehicles.size();
}

bool Road::is_colliding(int lane, int path_size, double predicted_car_s, Vehicle & vehicle) {
  bool collision = false;

  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while (it != this->vehicles.end() && !collision) {

    int v_id = it->first;
    Vehicle v = it->second;

    if (v.lane == lane) {

      double vehicle_s = v.s + ((double)path_size * Timestep * v.v);

      if (((vehicle_s > predicted_car_s) && ((vehicle_s - predicted_car_s) < 30.0)) 
						|| fabs(vehicle_s - predicted_car_s) < 15.0) {
				vehicle = v;
				vehicle.s = vehicle_s;
        collision = true;
        break;
      }
    }
    it++;
  }

  return collision;
}

