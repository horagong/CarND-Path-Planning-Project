#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "constants.h"

using namespace std;

class Vehicle {
public:

  double x;
  double y;
  double s;
  double d;
  double v;
//  string state = "KL";

  int lane;
//  double target_speed = Speed_Limit;

	double theta;

  /**
  * Constructor
  */
  Vehicle();
	// for the other cars from fusion data
  Vehicle(double s, double d, double vx, double vy);

  Vehicle(double x, double y, double s, double d, double theta, double v);

  /**
  * Destructor
  */
  virtual ~Vehicle();


	// for the other cars from fusion data
	void update(double s, double d, double vx, double vy);
	// for the ego car
	void update(double x, double y, double s, double d, double theta, double v);
};

#endif
