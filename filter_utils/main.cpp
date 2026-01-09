#include <iostream>
#include "Eigen/Dense"
#include "kf.h"


void main() {
	FB::KF ekf;

	Eigen::VectorXd state(FB::STATE_SIZE);
	state << 0, 0, 0, FB::PI, 0, 0, // x, y, z, roll, pitch, yaw
		0, 0, 0, 0, -FB::PI, 0, // vx, vy, vz, gx, gy, gz
		0, 0, 0; // ax, ay, az

	ekf.setState(state);
	ekf.predict();
	ekf.correct();
}