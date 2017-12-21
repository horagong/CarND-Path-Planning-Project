#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

double logistic(double x);

double max_accel_cost(int lane, const vector<Vehicle> trajectory, const map<int, vector<Vehicle>> & predictions);
double max_jerk_cost(int lane, const vector<Vehicle> trajectory, const map<int, vector<Vehicle>> & predictions);
double total_accel_cost(int lane, const vector<Vehicle> trajectory, const map<int, vector<Vehicle>> & predictions);
double total_jerk_cost(int lane, const vector<Vehicle> trajectory, const map<int, vector<Vehicle>> & predictions);

double nearest_approach_to_any_vehicle(int lane, const vector<Vehicle> trajectory, const map<int, vector<Vehicle>> & predictions);

double buffer_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory);

double goal_distance_cost(const Vehicle & vehicle,  const vector<Vehicle> & trajectory,  const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double lane_speed(Vehicle v, const map<int, vector<Vehicle>> & predictions, int lane);

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);

#endif
