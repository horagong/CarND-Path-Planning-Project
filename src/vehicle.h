#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "constants.h"
#include "jmt.h"

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.


  double s;
  double d;
  double v;
  double a;
  string state;
  int lane;
	int intended_lane;
  double target_speed = Speed_Limit;


  int goal_lane;
  double goal_s;
  double x;
  double y;
	double theta;
	double psi;
	double delta;
  double vx;
  double vy;


  int lanes_available = Num_Lanes;
  double max_acceleration = Max_Acceleration;
  double max_jerk = Max_Jerk;


  /**
  * Constructor
  */
  Vehicle();
	// for the other cars from fusion data
  Vehicle(double s, double d, double vx, double vy);

  Vehicle(double s, double v, double a, double d, double psi, string state);
  Vehicle(double s, double v, double a, double d, string state);

  /**
  * Destructor
  */
  virtual ~Vehicle();


	// for the other cars from fusion data
	void update(double s, double d, double vx, double vy);
	// for the ego car
	void update(double s, double v, double a, double d, double theta, double psi, string state);

  string display();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions, int horizon);

  vector<string> successor_states();


	double position_at(double t);

  vector<Vehicle> generate_predictions(int v_id, Vehicle ego, int from, int to);

	vector<double> state_in(double t) const;
	vector<vector<double>> ego_predictions(int horizon);

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions, int horizon);

  vector<JMT> get_kinematics(map<int, vector<Vehicle>> predictions, int lane, int horizon, int & v_ahead_id, int & v_behind_id);

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions, int horizon);
  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions, int horizon);
  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions, int horizon);

	int get_vehicle_in_range(map<int, vector<Vehicle>> predictions, int lane, int horizon);
  int get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, int horizon);
  int get_vehicle_ahead (map<int, vector<Vehicle>> predictions, int lane, int horizon) const;

	void configure(vector<double> road_data);


};

#endif
