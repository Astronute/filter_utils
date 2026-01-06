#include <iostream>
#include "Eigen/Dense"


void main() {
	Eigen::MatrixXd transfer_function_(2, 2);
	transfer_function_ << 1, 2, 3, 4;
	std::cout << transfer_function_.setIdentity() << std::endl;
}