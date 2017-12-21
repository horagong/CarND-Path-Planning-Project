#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include "constants.h"

const double REACH_GOAL = pow(10, 1); //5
const double EFFICIENCY = pow(10, 13); //6
const double BUFFER = pow(10, 1); //10

double logistic(double x) {
    /*
    A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
		*/   
    return 2.0 / (1 + exp(-x)) - 1.0;
}


double max_accel_cost(const Vehicle & ego, const vector<Vehicle> & trajectory
			, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
	double max_a = 0;	
	for (vector<Vehicle>::const_iterator it = trajectory.begin(); it != trajectory.end(); ++it) {
		Vehicle v = *it;
		if (fabs(v.a) > fabs(max_a)) {
			max_a = v.a;
		}
	}
	if (fabs(max_a) >= fabs(Max_Acceleration))
		return 1;
	else
		return 0;
}
/*
double max_jerk_cost(const Vehicle & ego, const vector<Vehicle> & trajectory
			, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
	double max_j = 0;	
	for (vector<Vehicle>::const_iterator it = trajectory.begin(); it != trajectory.end(); ++it) {
		Vehicle v = *it;
		if (fabs(v.j) > fabs(max_j)) {
			max_j = v.j;
		}
	}
	if (fabs(max_j) >= fabs(Max_Jerk))
		return 1;
	else
		return 0;
}
*/

double total_accel_cost(const Vehicle & ego, const vector<Vehicle> & trajectory
			, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
	double total_a = 0;	
	double t = 0;
	for (vector<Vehicle>::const_iterator it = trajectory.begin(); it != trajectory.end(); ++it) {
		Vehicle v = *it;
		total_a += fabs(v.a * t);
		t += Timestep;
	}
	double accel_per_sec = total_a/t;
	return logistic(accel_per_sec / Expected_Acceleration_Per_Sec);
}
/*
double total_jerk_cost(const Vehicle & ego, const vector<Vehicle> & trajectory
			, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
	double total_j = 0;	
	double t = 0;
	for (vector<Vehicle>::const_iterator it = trajectory.begin(); it != trajectory.end(); ++it) {
		Vehicle v = *it;
		total_j += fabs(v.j * t);
		t += Timestep;
	}
	double jerk_per_sec = total_j/t;
	return logistic(jerk_per_sec / Expected_Jerk_Per_Sec);
}
*/
double nearest_approach_to_any_vehicle(int lane, const vector<Vehicle> trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Calculates the closest distance to any vehicle during a trajectory.
    */

    double closest = 999999;
		int vehicle_id = -1;
		double dist = 999999;
		Vehicle vehicle;

    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        vehicle = it->second[1];

				double ego_pred_s = trajectory[1].s;
				double ego_pred_v = trajectory[1].v;
				double ego_pred_a = trajectory[1].a;
				double ego_pred_d = trajectory[1].d;
				double ego_pred_lane = trajectory[1].lane;

				//cout << "{" << it->first << ": " << vehicle.lane << "=" << vehicle.s - ego_pred_s << "} ";
				if (lane == vehicle.lane) {
					//double dist = sqrt(pow(ego_pred_s - vehicle.s, 2) + pow(ego_pred_d - vehicle.d, 2));
					dist = vehicle.s - ego_pred_s;
					if (dist < 0)
						continue;
					if (dist < closest) {
						closest = dist;
						vehicle_id = it->first;
					}
				}
		}
		cout << vehicle_id << ":" << lane << "=" << closest << ", ";
    return closest;
}


double buffer_cost(const Vehicle & ego, const vector<Vehicle> & trajectory
			, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {

	map<string, double> trajectory_data = get_helper_data(ego, trajectory, predictions);
	int intended_lane = trajectory_data["intended_lane"];
	int final_lane = trajectory_data["final_lane"];

	cout << " [";
	double nearest = nearest_approach_to_any_vehicle(intended_lane, trajectory, predictions)
									+ nearest_approach_to_any_vehicle(final_lane, trajectory, predictions);
	cout << " :" << logistic(2*Vehicle_Radius/nearest) << "]";
	return logistic(2*Vehicle_Radius / nearest);
}

double goal_distance_cost(const Vehicle & ego, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    */
    double cost;
    double distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - 2*exp(-(abs(2.0*ego.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
    } else {
        cost = 1;
    }
		cout << " [" << cost << ": " << distance << ", " << ego.goal_lane << ", " << data["intended_lane"] << ", " << data["final_lane"] << "]";
    return cost;
}

double inefficiency_cost(const Vehicle & ego, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have slower traffic. 
    */

    map<string, double> trajectory_data = get_helper_data(ego, trajectory, predictions);
		int intended_lane = trajectory_data["intended_lane"];
		int final_lane = trajectory_data["final_lane"];

    double proposed_speed_intended = lane_speed(ego, predictions, intended_lane);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = Speed_Limit;
    }

    double proposed_speed_final;
		if (final_lane == intended_lane) {
				proposed_speed_final = proposed_speed_intended;
		} else {
    		proposed_speed_final = lane_speed(ego, predictions, final_lane);
				if (proposed_speed_final < 0) {
						proposed_speed_final = Speed_Limit;
				}
		}

    double cost = (2.0*ego.target_speed - proposed_speed_intended - proposed_speed_final)/ego.target_speed;
		cout <<  " [" << cost << ": " << intended_lane << "=" << proposed_speed_intended 
													<< ", " << final_lane << "=" << proposed_speed_final << "]";

    return cost;
}

double lane_speed(Vehicle ego, const map<int, vector<Vehicle>> & predictions, int lane) {

/*
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.lane == lane && vehicle.s <= Lane_Change_Search_Range + s) {
            return vehicle.v;
        }
    }
*/
		int horizon = predictions.begin()->second.size() - 1;
		int v_id;
		for (int h = 1; h <= horizon; h++) {
			v_id = ego.get_vehicle_ahead(predictions, lane, horizon);
			if (v_id != -1) {
				auto it = predictions.find(v_id);
				Vehicle v = it->second[h];
				return v.v;
			}
		}
    //Found no vehicle in the lane
    return -1.0;
}

double calculate_cost(const Vehicle & ego, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, double> trajectory_data = get_helper_data(ego, trajectory, predictions);
    double cost = 0.0;

    //Add additional cost functions here.
    vector< function<double(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, double> &)>> 
					cf_list = {inefficiency_cost}; //goal_distance_cost, buffer_cost};//, lane_change_cost};//, inefficiency_cost, buffer_cost};

    vector<double> weight_list = {EFFICIENCY}; //REACH_GOAL, BUFFER};//, LANE_CHANGE};//, EFFICIENCY, BUFFER};
    
		Vehicle ego_trajectory = trajectory[1];
		cout << "C:" << ego_trajectory.state;
    for (int i = 0; i < cf_list.size(); i++) {
        double f_cost = cf_list[i](ego, trajectory, predictions, trajectory_data);
        double new_cost = weight_list[i] * f_cost;
				//cout << ", [" << f_cost << ", lane=" << ego_trajectory.lane << "]";
        cost += new_cost;
    }
		cout << " " << cost << endl;

    return cost;

}

map<string, double> get_helper_data(const Vehicle & ego, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
    final_lane: the lane of the vehicle at the end of the trajectory.
    distance_to_goal: the distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
//		cout << "get_helper" << endl;
    map<string, double> trajectory_data;
//		cout << "get_helper1" << endl;
    Vehicle trajectory_last = trajectory.back();
    double intended_lane;
    double final_lane;
//		cout << "get_helper11" << endl;

/* XXX */
    if (trajectory_last.state.compare("PLCL") == 0) {
			intended_lane = trajectory_last.lane - 1;
    	final_lane = trajectory_last.lane;
    } else if (trajectory_last.state.compare("PLCR") == 0) {
			intended_lane = trajectory_last.lane + 1;
    	final_lane = trajectory_last.lane;
    } else if (trajectory_last.state.compare("LCL") == 0 || trajectory_last.state.compare("LCR") == 0) {
			intended_lane = trajectory_last.intended_lane;
			final_lane = trajectory_last.intended_lane;
    } else {
			intended_lane = trajectory_last.lane;
    	final_lane = trajectory_last.lane;
    }
/*
		if (trajectory_last.s > Max_S) 
			trajectory_last.s -= Max_S;
*/
//		cout << "get_helper111" << endl;
    double distance_to_goal = ego.goal_s - trajectory_last.s;
		//cout << "goal_to_dist=" << ego.goal_s << "-" << trajectory_last.s << endl;
//		cout << "get_helper111" << endl;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
//		cout << "get_helper111" << endl;
    return trajectory_data;
}


