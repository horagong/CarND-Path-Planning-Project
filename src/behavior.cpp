#include "behavior.h"


Behavior::Behavior() {}

Behavior::~Behavior() {}


void Behavior::plan_path(int path_size, double end_path_s) {

	this->path_size = path_size;
	this->end_path_s = end_path_s;
  this->no_available_lanes = false;


	vector<Vehicle> collision_vehicles(3);
	int available_next_lane;
	vector<bool> collision_list = get_available_next_lane(collision_vehicles, available_next_lane);

	cout << "lane=" << lane 
				<< ", target_lane=" << target_lane 
				<< ", target_speed=" << target_speed 
				<< ", available_next_lane=" << available_next_lane << endl;



	double next_v = this->target_speed;
	if (collision_list[lane]) {
		cout << "reducing speed=" << road.ego.v 
					<< "... at ego lane according to v=" << collision_vehicles[lane].v << endl;
		double buffer = collision_vehicles[lane].s - end_path_s;
		if (collision_vehicles[lane].v < next_v 
				|| (buffer > 0 && buffer < 15)) {

				if (fabs(buffer) < 5) { 
						cout << "COLLISION at v=" << road.ego.v << ", buffer=" 
									<< buffer 
									<< ", lane=" << lane << endl;
				}
				next_v -= 0.1;
				if (next_v < 1)
					next_v = 1;
		}
	}
	else {
		next_v += 0.1;
		if (next_v > Speed_Limit)
			next_v = Speed_Limit;
	}
	this->target_speed = next_v;


	string next_state;
	if (state.compare("KL") == 0) {
		target_lane = available_next_lane;

		if (target_lane < lane)
				next_state = "PLCL";
		else if (target_lane > lane)
				next_state = "PLCR";
		else
				next_state = "KL";

	} else if (state.compare("PLCL") == 0) {	
		if (collision_list[target_lane]) {
				next_state = "PLCL";
		} else {
				next_state = "LCL";
		}
	} else if (state.compare("PLCR") == 0) {	
		if (collision_list[target_lane]) {
				next_state = "PLCR";
		} else {
				next_state = "LCR";
		}
	} else if (state.compare("LCL") == 0) {	
			lane = target_lane;
			next_state = "KL";
	} else if (state.compare("LCR") == 0) {	
			lane = target_lane;
			next_state = "KL";
	}
	

	cout << "state=" << state << " -> " << next_state 
			<< ", lane=" << road.ego.lane << " -> " << this->lane << ", target_speed=" << this->target_speed 
			<< endl << endl;

  state = next_state;

  return;
}


vector<bool> Behavior::get_available_next_lane(vector<Vehicle> & vehicles, int & next_lane) {
	// XXX rear check
	vector<bool> collision_list = {false, false, false};

  bool collision = false;
	double max_v = 0;
 	int available_next_lane = -1; 
	int lane_with_highest_v = -1;

	for (int l = 0; l < Num_Lanes; l++) {
		collision = road.is_colliding(l, path_size, end_path_s, vehicles[l]);
		if (collision) {
			cout << "collision at lane=" << l << endl;
			collision_list[l] = true;

			if (fabs(l - road.ego.lane) <= 1) {
				if (vehicles[l].v > max_v) {
					max_v = vehicles[l].v;
					lane_with_highest_v = l;
				}
			}
		} else {
			if (fabs(l - road.ego.lane) <= 1) {
				available_next_lane = l;
			}
		}
	}

	if (available_next_lane == -1) {
		available_next_lane = lane_with_highest_v;
	}
	next_lane = available_next_lane;

	return collision_list;
}
