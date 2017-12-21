#ifndef JMT_H
#define JMT_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class JMT {
public:
	vector<double> coeffs;
	vector<double> start;
	vector<double> goal;

	double T;

	JMT(vector<double> start, vector<double> goal, double T);
	virtual ~JMT();

	vector<double> predict(double t);

};

#endif
