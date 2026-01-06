#pragma once

#include "Eigen/Dense"

enum StateMembers {
	StateMemberX = 0,
	StateMemberY,
	StateMemberZ,
	StateMemberRoll,
	StateMemberPitch,
	StateMemberYaw,
	StateMemberVx,
	StateMemberVy,
	StateMemberVz,
	StateMemberGx,
	StateMemberGy,
	StateMemberGz,
	StateMemberAx,
	StateMemberAy,
	StateMemberAz
};

class FilterBase {
public:
	FilterBase() {
		transfer_func_.setIdentity();
	}
	~FilterBase() {}

	void setState(const Eigen::VectorXd& state);

protected:
	Eigen::VectorXd state_;
	Eigen::MatrixXd transfer_func_;
};