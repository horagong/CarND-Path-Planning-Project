#include <iostream>
#include "jmt.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


JMT::JMT(vector<double> start, vector<double> goal, double T) {
		/**
		Calculates Jerk Minimizing Trajectory for start, goal and T.
		*/

		MatrixXd A = MatrixXd(3, 3);
		A << T*T*T, T*T*T*T, T*T*T*T*T,
						3*T*T, 4*T*T*T,5*T*T*T*T,
						6*T, 12*T*T, 20*T*T*T;

		MatrixXd B = MatrixXd(3,1);
		B << goal[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
						goal[1]-(start[1]+start[2]*T),
						goal[2]-start[2];

		MatrixXd Ai = A.inverse();

		MatrixXd C = Ai*B;

		vector <double> result = {start[0], start[1], start[2]/2};
		for(int i = 0; i < C.size(); i++)
		{
				result.push_back(C.data()[i]);
		}

		this->start = start;
		this->goal = goal;
		this->T = T;
		this->coeffs = result;

		//cout << "P:JMT: [" << start[0] << ", " << start[1] << ", " << start[2] << "] -> ["
		//									<< goal[0] << ", " << goal[1] << ", " << goal[2] << "] in " << T << endl;
}

JMT::~JMT() {}

vector<double> JMT::predict(double t) {
	double s, v, a, j, n, c;
	if (t <= T) {
		s = coeffs[0] + coeffs[1]*t + coeffs[2]*t*t + coeffs[3]*t*t*t + coeffs[4]*t*t*t*t + coeffs[5]*t*t*t*t*t;
		v = coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t*t + 4*coeffs[4]*t*t*t + 5*coeffs[5]*t*t*t*t;
		a = 2*coeffs[2] + 3*2*coeffs[3]*t + 4*3*coeffs[4]*t*t + 5*4*coeffs[5]*t*t*t;

		j = 3*2*coeffs[3] + 4*3*2*coeffs[4]*t + 5*4*3*coeffs[5]*t*t;
		n = 4*3*2*coeffs[4] + 5*4*3*2*coeffs[5]*t;
		c = 5*4*3*2*coeffs[5];
	} else {
		s = coeffs[0] + coeffs[1]*t + coeffs[2]*t*t/2;
		v = coeffs[0] + coeffs[1]*t;
		a = coeffs[0];
		j = 0;
		n = 0;
		c = 0;
	}

	return {s, v, a, j, n, c};
}

