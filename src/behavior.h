#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "road.h"
#include "vehicle.h"
#include <vector>


class Behavior {

public:
	Road road;

	Behavior();
	virtual ~Behavior();

  void plan_path(int path_size, double end_path_s);
	vector<bool> get_available_next_lane(vector<Vehicle> & vehicles, int & next_lane);

  int lane = 1;
  int target_lane = 1;
	double end_path_s;
	double path_size;
  double target_speed = 0.5; //0.22;
	bool no_available_lanes;
	string state = "KL";
};


#endif

