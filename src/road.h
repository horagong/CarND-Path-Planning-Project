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

		// These affect the visualization
		int FRAMES_PER_SECOND = 4;
		int AMOUNT_OF_ROAD_VISIBLE = 40;
		int update_width = 70;
  	string ego_rep = " *** ";


  	int num_lanes;
		//all traffic in lane (besides ego) follow these speeds
    vector<double> lane_speeds;
    double speed_limit;
    map<int, Vehicle> vehicles;
    int vehicles_added = 0;

  	int ego_key = -1;
		Vehicle ego;

    /**
  	* Constructor
  	*/
  	Road(double speed_limit, vector<double> lane_speeds);

  	/**
  	* Destructor
  	*/
  	virtual ~Road();

  	Vehicle get_ego();

  	void populate_traffic(json fusion);

  	vector<Vehicle> advance(int from, int to);

  	void display(int timestep);

  	//void add_ego(int lane_num, int s, vector<int> config_data);

  	void cull();

};

