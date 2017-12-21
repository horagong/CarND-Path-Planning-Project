#include <algorithm>
#include <iostream>
#include <sstream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "constants.h"
#include "helpers.h"
#include "jmt.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}


Vehicle::Vehicle(double s, double v, double a, double d, double psi, string state) {
    this->lane = d2lane(d);
    this->intended_lane = d2lane(d);
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;

    this->d = d;
		this->psi = psi;
}

/* From fusion data */
Vehicle::Vehicle(double s, double d, double vx, double vy) {

    this->lane = d2lane(d);
    this->intended_lane = d2lane(d);
    this->s = s;
		this->v = sqrt(vx*vx + vy*vy);
		this->a = 0;
    this->state = "KL";

    this->d = d;
		this->theta = atan2(vy, vx);
}


Vehicle::~Vehicle() {}

/* From fusion data */
void Vehicle::update(double s, double d, double vx, double vy) {
  this->lane = d2lane(d);
	this->intended_lane = d2lane(d);
	this->s = s;
	this->v = sqrt(vx*vx + vy*vy);
	this->a = 0;

	this->d = d;
	this->theta = atan2(vy, vx);
}


vector<Vehicle> Vehicle::generate_predictions(int v_id, Vehicle ego, int from, int to) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
		vector<Vehicle> predictions;
		// from -1 
    for(int i = from - 1; i < to; i++) {
      auto sva = state_in(Timestep*i);

      Vehicle vehicle = Vehicle(sva[0], sva[1], sva[2], this->d, pi()/2, "KL");
			//if (i == from + 1)
/*
			if (this->lane == ego.lane) {
				auto ego_timestep = i - from;
				auto ego_state = ego.state_in(Timestep*ego_timestep);
				cout << "GP:" << i << " [" << v_id << ": " << vehicle.lane << ", " 
							<< (vehicle.s - ego_state[0]) << "(" << vehicle.s << "-" << ego_state[0] << "), " 
							<< (vehicle.v - ego_state[1]) << "(" << vehicle.v << "-" << ego_state[1] << ")" << endl;
			}
*/
      predictions.push_back(vehicle);
  	}
    return predictions;
}

vector<double> Vehicle::state_in(double t) const {
	double s = this->s + this->v*t + 1/2*this->a*t*t;
	double v = this->v + this->a*t;
	double a = this->a;
	return {s, v, a};
}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions, int horizon) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */

    vector<string> states = successor_states();
    double cost;
    vector<double> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions, horizon);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
						Vehicle ego = trajectory[1];
            final_trajectories.push_back(trajectory);
        }
    }

    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
		cout << "C:BEST " << final_trajectories[best_idx][1].state << ", cost=" << costs[best_idx] << endl;
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.

		lane number starts from 0 at the most left lane.
    */
    vector<string> states;
    string state = this->state;
    if(state.compare("KL") == 0) {

			if (this->lane == lanes_available - 1) {
				states.push_back("PLCL");
				states.push_back("KL");
			} else if (this->lane == 0) {
				states.push_back("PLCR");
				states.push_back("KL");
			} else{
				states.push_back("PLCL");
				states.push_back("PLCR");
				states.push_back("KL");
			}


    } else if (state.compare("PLCL") == 0) {
			if (this->lane != 0) {
					states.push_back("PLCL");
					states.push_back("LCL");
			}
			states.push_back("KL");
    } else if (state.compare("PLCR") == 0) {
			if (this->lane != lanes_available - 1) {
					states.push_back("PLCR");
					states.push_back("LCR");
			}
			states.push_back("KL");

    } else if (state.compare("LCR") == 0) {

			if (this->lane == lanes_available - 1)
				states.push_back("KL");
			else {
				
				if (this->intended_lane != this->lane)
					states.push_back("LCR");
				else {
					states.push_back("KL");
				}
			}


		} else if (state.compare("LCL") == 0) {

			if (this->lane == 0)
				states.push_back("KL");
			else {
				if (this->intended_lane != this->lane)
					states.push_back("LCL");
				else {
					states.push_back("KL");
				}
			}

		}

    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions, int horizon) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions, horizon);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions, horizon);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions, horizon);
    }
    return trajectory;
}

int Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, int horizon) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -1;

		int vehicle_id = -1;
		vector<double> s_v_a = state_in(Timestep*horizon);

    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[horizon];
        if (temp_vehicle.lane == lane && temp_vehicle.s < s_v_a[0] && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
						vehicle_id = it->first;

						if (horizon == 1)
						cout << "GV:B" << horizon << " [" << vehicle_id << ": " << temp_vehicle.lane << ", " 
									<< temp_vehicle.s - s_v_a[0] << " (" << temp_vehicle.s << " - " << s_v_a[0] << "), " 
									<< temp_vehicle.v - s_v_a[1] << " (" << temp_vehicle.v << " - " << s_v_a[1] << ")" << endl;
        }
    }
    return vehicle_id;
}

int Vehicle::get_vehicle_ahead (map<int, vector<Vehicle>> predictions, int lane, int horizon) const {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */

		int vehicle_id = -1;
		vector<double> s_v_a = state_in(Timestep*horizon);
    int min_s = Follow_Ahead_Range + s_v_a[0];

    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[horizon];

        if (temp_vehicle.lane == lane && temp_vehicle.s > s_v_a[0] && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
						vehicle_id = it->first;

						if (horizon == 1)
						cout << "GV:A" << horizon << " [" << vehicle_id << ": " << temp_vehicle.lane << ", "
									<< temp_vehicle.s - s_v_a[0] << " (" << temp_vehicle.s << " - " << s_v_a[0] << "), " 
									<< temp_vehicle.v - s_v_a[1] << " (" << temp_vehicle.v << " - " << s_v_a[1] << ")" << endl;
        }
    }
    return vehicle_id;
}


int Vehicle::get_vehicle_in_range(map<int, vector<Vehicle>> predictions, int lane, int horizon) {
		Vehicle vehicle;
		int vehicle_id = -1;
		vector<double> s_v_a = state_in(Timestep*horizon);

		for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
				vehicle = it->second[horizon];
				if (vehicle.lane == lane
						&& vehicle.s <= (s_v_a[0] + Lane_Change_Buffer)
						&& vehicle.s >= (s_v_a[0] - Lane_Change_Buffer)) {
						vehicle_id = it->first;
						cout << "L:" << this->state << horizon << " OCCUPIED=[" << vehicle_id << ": " << vehicle.lane << ", " 
								<< vehicle.s - s_v_a[0] << "] ego.s=" << s_v_a[0] << ", l=" << lane << endl;
				}
/*
						cout << "L:" << this->state << horizon << " NOT OCCUPIED=[" << vehicle_id << ": " << vehicle.lane << ", " 
								<< vehicle.s - s_v_a[0] << "] ego.s=" << s_v_a[0] << ", l=" << lane << endl;
*/
		}
		return vehicle_id;
}

vector<JMT> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane, int horizon, int & v_ahead_id, int & v_behind_id) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */


    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

		vector<vector<double>> sva_dva;
		double max_velocity_in_front;

		this->target_speed = Speed_Limit * sin(this->psi);

		int h = 1;
		for (h = 1; h <= horizon; h++) {
			v_ahead_id = get_vehicle_ahead(predictions, lane, h);

			if (v_ahead_id != -1) {
					auto it = predictions.find(v_ahead_id);
					vehicle_ahead = it->second[h];

					v_behind_id = get_vehicle_behind(predictions, lane, h);
					if (v_behind_id != -1) {
							//must travel at the speed of traffic, regardless of preferred buffer
						auto it = predictions.find(v_behind_id);
						vehicle_behind = it->second[h];

						this->target_speed = min(vehicle_ahead.v * 8/10, Speed_Limit);
						break;

					} else {
							//choose max velocity and acceleration that leaves desired buffer in front
							//Equation: front buffer <= (vcl.s - ego.s) + (vcl.v - ego.v)*t + 0.5 * (vcl.a - ego.a)*t*t
							//double max_velocity_in_front = ((vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v*t - 0.5*(this->a)*t*t)/t;

						/* XXX*/
						/*
						double t = Timestep * h;
						double max_velocity_in_front = ((vehicle_ahead.s - s_v_a[0] - this->preferred_buffer) 
								+ vehicle_ahead.v*t - 1/2*s_v_a[2]*t*t - 1/6*s_v_a[3]*t*t*t)/t;
						*/
						this->target_speed = min(vehicle_ahead.v * 8/10, Speed_Limit);
						break;
					}
			}
		}
		if (h > horizon)
			h = horizon;
		
		double Ts = Timestep*horizon;
		double s0 = this->s;
		double v0_s = this->v*sin(this->psi);
		double a0_s = this->a*sin(this->psi);

		double v_s = this->target_speed*sin(this->psi);
		double s = s0 + v0_s * Ts;
		double a_s = a0_s; //0;

		bool slow_start = false;
		if (v0_s <= 15 && v_s > 15) {
			v_s = v0_s + 0.005;
			s = s0 + v_s * Ts;
			slow_start = true;
		}
		JMT jmt_s = JMT({s0, v0_s, a0_s}
									, {s, v_s, a_s}
									, Ts);


		double d0 = this->d;
		double v0_d = this->v*cos(this->psi);
		double a0_d = this->a*cos(this->psi);

		double d;
		if (fabs(this->lane - this->intended_lane) != 0) {
			d = (lane2d(this->intended_lane) - lane2d(this->lane)) / 6 + d0;
		} else
			d = lane2d(this->intended_lane);
		double v_d = v0_d; //this->target_speed*cos(this->psi);
		double a_d = a0_d; //this->a*cos(this->psi);
		double Td = Timestep;//*horizon;
		if (slow_start) {
			d = d0;
		}
		JMT jmt_d = JMT({d0, v0_d, a0_d}
									, {d, v_d, a_d}
									, Td);

		sva_dva = {jmt_s.predict(Timestep), jmt_d.predict(Timestep)};

		if (v_ahead_id != -1 && v_behind_id != -1) {
			cout << "GK:" << h << " A[" << v_ahead_id << ": " << lane << ", " 
									<< vehicle_ahead.s - sva_dva[0][0] << " (" << vehicle_ahead.s << " - " << sva_dva[0][0] << "), " 
									<< vehicle_ahead.v - sva_dva[0][1] << " (" << vehicle_ahead.v << " - " << sva_dva[0][1] << ")], B[" 
									<< v_behind_id << ": " << lane << ", " 
									<< vehicle_behind.s - sva_dva[0][0] << " (" << vehicle_behind.s << " - " << sva_dva[0][0] << "), "
									<< vehicle_behind.v - sva_dva[0][1] << " (" << vehicle_behind.v << " - " << sva_dva[0][1] << ")]" << endl;

		} else if (v_ahead_id != -1) {
			cout << "GK:" << h << " A[" << v_ahead_id << ": " << lane << ", " 
								<< vehicle_ahead.s - sva_dva[0][0] << " (" << vehicle_ahead.s << " - " << sva_dva[0][0] << "), "
								<< vehicle_ahead.v - sva_dva[0][1] << " (" << vehicle_ahead.v << " - " << sva_dva[0][1] << ")]" << endl;
								//<< ")], (v_in_front=" << max_velocity_in_front << ")" << endl;
	/*
		} else if (behind_id != -1) {
			cout << "V:" << h << " B[" << v_behind_id << ": " << lane << ", " 
								<< vehicle_behind.s - sva_dva[0][0] << " (" << vehicle_behind.s << " - " << sva_dva[0][0] << "), "
								<< vehicle_behind.v - sva_dva[0][1] << " (" << vehicle_behind.v << " - " << sva_dva[0][1] << ")]" << endl;
	*/
		}
		return {jmt_s, jmt_d};
}




vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions, int horizon) {
    /*
    Generate a keep lane trajectory.
    */
		Vehicle v = Vehicle(this->s, this->v, this->a, this->d, this->psi, this->state);
		v.intended_lane = v.lane;
    vector<Vehicle> trajectory = {v};

		//cout << "KL:0" << v.display() << ", lane=" << v.lane << ", i_lane=" << v.intended_lane << endl;
		int v_ahead_id;
		int v_behind_id;
		auto jmt_sd = v.get_kinematics(predictions, this->lane, horizon, v_ahead_id, v_behind_id);
		cout << "GK: " << this->state << " -> KL"
					<< " (psi=" << this->psi
					<< " i_d=" << this->intended_lane
					<< ") -> (t_vs=" << v.target_speed*sin(v.psi) << " psi=" << v.psi
					<< " i_d=" << v.intended_lane
					<< "), s:[" << jmt_sd[0].start[0] << ", " << jmt_sd[0].start[1] << ", " << jmt_sd[0].start[2]
					<< "] -> [" << jmt_sd[0].goal[0] << ", " << jmt_sd[0].goal[1] << ", " << jmt_sd[0].goal[2] 
					<< "] in " << jmt_sd[0].T 
					<< ", d:[" << jmt_sd[1].start[0] << ", " << jmt_sd[1].start[1] << ", " << jmt_sd[1].start[2]
					<< "] -> [" << jmt_sd[1].goal[0] << ", " << jmt_sd[1].goal[1] << ", " << jmt_sd[1].goal[2] 
					<< "] in " << jmt_sd[1].T << endl;


		for (int i = 1; i <= horizon; i++) {
			vector<vector<double>> sva_dva = {jmt_sd[0].predict(Timestep*i), jmt_sd[1].predict(Timestep*i)};

			double new_s = sva_dva[0][0];
			double new_v_s = sva_dva[0][1];
			double new_a_s = sva_dva[0][2];
			double new_j_s = sva_dva[0][3];

			double new_d = sva_dva[1][0];
			double new_v_d = sva_dva[1][1];
			double new_a_d = sva_dva[1][2];
			double new_j_d = sva_dva[1][3];

			double new_v = sqrt(new_v_s*new_v_s + new_v_d*new_v_d);
			double new_a = sqrt(new_a_s*new_a_s + new_a_d*new_a_d);
			double new_j = sqrt(new_j_s*new_j_s + new_j_d*new_j_d);

			v = Vehicle(new_s, new_v, new_a, new_d, this->psi, "KL");
			v.intended_lane = this->lane;
			trajectory.push_back(v);

				cout << "GK:" << i << " " << this->state << " -> " << v.state
					<< " l=" << v.lane << "<" << v.intended_lane << ">"
					<< " s:[" << new_s << ", " << new_v_s << ", " << new_a_s << ", " << new_j_s 
					<< "], d:[" << new_d << ", " << new_v_d << ", " << new_a_d << ", " << new_j_d 
					<< "] v=" << new_v << ", a=" << new_a << ", j=" << new_j << endl;
		}
    return trajectory;
}


vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions, int horizon) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];

		Vehicle v = Vehicle(this->s, this->v, this->a, this->d, this->psi, this->state);
		v.intended_lane = v.lane;
		Vehicle v2 = v;
		v2.intended_lane = v2.lane;
    vector<Vehicle> trajectory = {v};
		int v_behind_id;
		int v_ahead_id;
		int v_behind_id2;
		int v_ahead_id2;

		auto curr_lane_jmt = v.get_kinematics(predictions, this->lane, horizon, v_ahead_id, v_behind_id);
		cout << "GK: " << this->state << " -> PLC-c"
					<< " (psi=" << this->psi
					<< " i_d=" << this->intended_lane
					<< ") -> (t_vs=" << v.target_speed*sin(v.psi) << " psi=" << v.psi
					<< " i_d=" << v.intended_lane
					<< "), s:[" << curr_lane_jmt[0].start[0] << ", " << curr_lane_jmt[0].start[1] << ", " << curr_lane_jmt[0].start[2]
					<< "] -> [" << curr_lane_jmt[0].goal[0] << ", " << curr_lane_jmt[0].goal[1] << ", " << curr_lane_jmt[0].goal[2] 
					<< "] in " << curr_lane_jmt[0].T 
					<< ", d:[" << curr_lane_jmt[1].start[0] << ", " << curr_lane_jmt[1].start[1] << ", " << curr_lane_jmt[1].start[2]
					<< "] -> [" << curr_lane_jmt[1].goal[0] << ", " << curr_lane_jmt[1].goal[1] << ", " << curr_lane_jmt[1].goal[2] 
					<< "] in " << curr_lane_jmt[1].T << endl;

		auto next_lane_jmt = v2.get_kinematics(predictions, new_lane, horizon, v_ahead_id2, v_behind_id2);
		cout << "GK: " << this->state << " -> PLC-n"
					<< " (psi=" << this->psi
					<< " i_d=" << this->intended_lane
					<< ") -> (t_vs=" << v2.target_speed*sin(v2.psi) << " psi=" << v2.psi
					<< " i_d=" << v2.intended_lane
					<< "), s:[" << next_lane_jmt[0].start[0] << ", " << next_lane_jmt[0].start[1] << ", " << next_lane_jmt[0].start[2]
					<< "] -> [" << next_lane_jmt[0].goal[0] << ", " << next_lane_jmt[0].goal[1] << ", " << next_lane_jmt[0].goal[2] 
					<< "] in " << next_lane_jmt[0].T 
					<< ", d:[" << next_lane_jmt[1].start[0] << ", " << next_lane_jmt[1].start[1] << ", " << next_lane_jmt[1].start[2]
					<< "] -> [" << next_lane_jmt[1].goal[0] << ", " << next_lane_jmt[1].goal[1] << ", " << next_lane_jmt[1].goal[2] 
					<< "] in " << next_lane_jmt[1].T << endl;

		vector<JMT> best_lane_jmt;

		vector<vector<double>> curr_lane_new_kinematics = {curr_lane_jmt[0].predict(Timestep)
																											, curr_lane_jmt[1].predict(Timestep)};
		vector<vector<double>> next_lane_new_kinematics = {next_lane_jmt[0].predict(Timestep)
																											, next_lane_jmt[1].predict(Timestep)};

		if (v_behind_id != -1) {
				//Keep speed of current lane so as not to collide with car behind.
				best_lane_jmt = curr_lane_jmt;			
		} else {
				//Choose kinematics with lowest velocity.
				if (next_lane_new_kinematics[0][1] < curr_lane_new_kinematics[0][1]) {
						best_lane_jmt = next_lane_jmt;
				} else {
						best_lane_jmt = curr_lane_jmt;
				}
		}

    for(int i = 1; i <= horizon; i++) {
				vector<vector<double>> best_lane_new_kinematics = {best_lane_jmt[0].predict(Timestep*i)
																				, best_lane_jmt[1].predict(Timestep*i)};
				auto new_s = best_lane_new_kinematics[0][0];
				auto new_v_s = best_lane_new_kinematics[0][1];
				auto new_a_s = best_lane_new_kinematics[0][2];
				auto new_j_s = best_lane_new_kinematics[0][3];

				auto new_d = best_lane_new_kinematics[1][0];
				auto new_v_d = best_lane_new_kinematics[1][1];
				auto new_a_d = best_lane_new_kinematics[1][2];
				auto new_j_d = best_lane_new_kinematics[1][3];

				auto new_v = sqrt(new_v_s*new_v_s + new_v_d*new_v_d);
				auto new_a = sqrt(new_a_s*new_a_s + new_a_d*new_a_d);
				auto new_j = sqrt(new_j_s*new_j_s + new_j_d*new_j_d);

				v = Vehicle(new_s, new_v, new_a, new_d, this->psi, state);
				v.intended_lane = v.lane;
				trajectory.push_back(v);
				/*
				if (i == horizon)
				cout << "TR:" << state << ":" << i 
				*/
				cout << "GK:" << i << " " << this->state << " -> " << v.state
					<< " l=" << v.lane << "<" << v.intended_lane << ">"
					<< " s:[" << new_s << ", " << new_v_s << ", " << new_a_s << ", " << new_j_s 
					<< "], d:[" << new_d << ", " << new_v_d << ", " << new_a_d << ", " << new_j_d 
					<< "] v=" << new_v << ", a=" << new_a << ", j=" << new_j << endl;
		}
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions, int horizon) {
    /*
    Generate a lane change trajectory.
    */
    int intended_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;

		// from ego car's current state
		Vehicle v = Vehicle(this->s, this->v, this->a, this->d, this->psi, this->state);
		v.intended_lane = intended_lane;
		trajectory.push_back(v);

		//this->intended_lane = intended_lane;
		int v_ahead_id;
		int v_behind_id;
		auto jmt_sd = v.get_kinematics(predictions, intended_lane, horizon, v_ahead_id, v_behind_id);
		cout << "GK: " << this->state << " -> LC"
					<< " (psi=" << this->psi
					<< " i_d=" << this->intended_lane
					<< ") -> (t_vs=" << v.target_speed*sin(v.psi) << " psi=" << v.psi
					<< " i_d=" << v.intended_lane
					<< "), s:[" << jmt_sd[0].start[0] << ", " << jmt_sd[0].start[1] << ", " << jmt_sd[0].start[2]
					<< "] -> [" << jmt_sd[0].goal[0] << ", " << jmt_sd[0].goal[1] << ", " << jmt_sd[0].goal[2] 
					<< "] in " << jmt_sd[0].T
					<< "), d:[" << jmt_sd[1].start[0] << ", " << jmt_sd[1].start[1] << ", " << jmt_sd[1].start[2]
					<< "] -> [" << jmt_sd[1].goal[0] << ", " << jmt_sd[1].goal[1] << ", " << jmt_sd[1].goal[2] 
					<< "] in " << jmt_sd[1].T << endl;

    for(int i = 1; i <= horizon; i++) {

			//Check if a lane change is possible (check if another vehicle occupies that spot).
			int vehicle_id = v.get_vehicle_in_range(predictions, intended_lane, i);
			if (vehicle_id != -1) {
				cout << "TR:" << state << " returns KL" << endl;
				return keep_lane_trajectory(predictions, horizon);
			}

			vector<vector<double>> sva_dva = {jmt_sd[0].predict(Timestep*i), jmt_sd[1].predict(Timestep*i)};
			double new_s = sva_dva[0][0];
			double new_v_s = sva_dva[0][1];
			double new_a_s = sva_dva[0][2];
			double new_j_s = sva_dva[0][3];

			double new_d = sva_dva[1][0];
			double new_v_d = sva_dva[1][1];
			double new_a_d = sva_dva[1][2];
			double new_j_d = sva_dva[1][3];

			double new_v = sqrt(new_v_s*new_v_s + new_v_d*new_v_d);
			double new_a = sqrt(new_a_s*new_a_s + new_a_d*new_a_d);
			double new_j = sqrt(new_j_s*new_j_s + new_j_d*new_j_d);

			v = Vehicle(new_s, new_v, new_a, new_d, this->psi, state);
			v.intended_lane = intended_lane;
			trajectory.push_back(v);

			cout << "GK:" << i << " " << this->state << " -> " << v.state
				<< " l=" << v.lane << "<" << v.intended_lane << ">"
				<< " s:[" << new_s << ", " << new_v_s << ", " << new_a_s << ", " << new_j_s 
				<< "], d:[" << new_d << ", " << new_v_d << ", " << new_a_d << ", " << new_j_d 
				<< "] v=" << new_v << ", a=" << new_a << ", j=" << new_j << endl;
		}
    return trajectory;
}

void Vehicle::configure(vector<double> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}

