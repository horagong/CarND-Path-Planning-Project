#ifndef HELPERS_H
#define HELPERS_H

#include <vector>
#include "constants.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

//constexpr double pi();
constexpr double pi() { return M_PI; };
double deg2rad(double x);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);
double to_equation(float t, vector<double> coefficients);
double mph2mps(double mph);

double lane2d(int lane);
int d2lane(double d);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
#endif
