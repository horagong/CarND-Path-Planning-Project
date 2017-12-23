#include <algorithm>
#include <iostream>
#include <sstream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "constants.h"
#include "helpers.h"

/**
 * Initializes Vehicle
 */

using namespace std;

Vehicle::Vehicle(){}


Vehicle::Vehicle(double x, double y, double s, double d, double theta, double v) {

    this->lane = d2lane(d);
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
		this->theta = theta;
    this->v = v;
}

/* From fusion data */
Vehicle::Vehicle(double s, double d, double vx, double vy) {
    this->lane = d2lane(d);
    this->s = s;
    this->d = d;
		this->v = sqrt(vx*vx + vy*vy);
		this->theta = atan2(vy, vx);
}


Vehicle::~Vehicle() {}

/* From fusion data */
void Vehicle::update(double s, double d, double vx, double vy) {
  this->lane = d2lane(d);
	this->s = s;
	this->d = d;
	this->v = sqrt(vx*vx + vy*vy);
	this->theta = atan2(vy, vx);
}

void Vehicle::update(double x, double y, double s, double d, double theta, double v) {

    this->lane = d2lane(d);
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
		this->theta = theta;
    this->v = v;
}

