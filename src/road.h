#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "json.hpp"
#include "vehicle.h"
#include "helpers.h"

using namespace std;
using json = nlohmann::json;

class Road {
public:

    map<int, Vehicle> vehicles;
    int vehicles_added = 0;
		Vehicle ego;

		vector<double> map_waypoints_x;
		vector<double> map_waypoints_y;
		vector<double> map_waypoints_s;
		vector<double> map_waypoints_dx;
		vector<double> map_waypoints_dy;

    /**
  	* Constructor
  	*/
		Road();

  	/**
  	* Destructor
  	*/
  	virtual ~Road();


		void set_map(vector<double> map_waypoints_x, vector<double> map_waypoints_y
				, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
  	void populate_traffic(json fusion);


		bool is_colliding(int lane, int path_size, double end_path_s, Vehicle & vehicle);
  	vector<Vehicle> advance(int from, int to);
};

#endif
