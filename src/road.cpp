#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>

using namespace std;
using json = nlohmann::json;
/**
 * Initializes Road
 */
Road::Road(double speed_limit, vector<double> lane_speeds) {

		cout << "Road: ego max_acceleration=" << this->ego.max_acceleration << endl;

    this->num_lanes = lane_speeds.size();
    this->lane_speeds = lane_speeds;
    this->speed_limit = speed_limit;

		cout << " num_lanes=" << num_lanes;
		cout << " speed_limit=" << speed_limit << endl;
}

Road::~Road() {
}

Vehicle Road::get_ego() {
	
	return this->ego;
}

void Road::populate_traffic(json fusion) {
	
  map<int, Vehicle>::iterator it;

	cout << "F: ";
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
/*
				if (this->ego.lane == lane)
					cout << "[" << vehicle_id << ": " << vehicle.lane << ", " << vehicle.s << "-" << this->ego.s << ", " << vehicle.v;
*/
      } else {
        Vehicle vehicle = Vehicle(s, d, vx, vy);
        vehicles.insert(std::pair<int, Vehicle>(vehicle_id, vehicle));
/*
				if (this->ego.lane == lane)
					cout << "[" << vehicle_id << "+ " << vehicle.lane << ", " << vehicle.s << "-" << this->ego.s << ", " << vehicle.v;
*/
      }
			if (this->ego.lane == lane)
				cout << "], ";
    }
  }
	cout << endl;

	vehicles_added = vehicles.size();
	//cout << " number of vehicles =" << vehicles_added << endl;
}

vector<Vehicle> Road::advance(int from, int to) {
	
	map<int ,vector<Vehicle> > predictions;

	map<int, Vehicle>::iterator it = this->vehicles.begin();
	while(it != this->vehicles.end())
	{
			int v_id = it->first;
			vector<Vehicle> preds = it->second.generate_predictions(v_id, this->ego, from, to);
			predictions[v_id] = preds;

			it++;
	}

	vector<Vehicle> trajectory = this->ego.choose_next_state(predictions, to - from);
	return trajectory;
}

