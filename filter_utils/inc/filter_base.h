#pragma once

#include "Eigen/Dense"

namespace FB{

const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

enum StateMembers {
	// state terms
	StateMemberX = 0,
	StateMemberY,
	StateMemberZ,
	StateMemberRoll,
	StateMemberPitch,
	StateMemberYaw,
	StateMemberVx,
	StateMemberVy,
	StateMemberVz,
	// control terms
	StateMemberGx,
	StateMemberGy,
	StateMemberGz,
	StateMemberAx,
	StateMemberAy,
	StateMemberAz
};

enum ControlMembers {
	ControlMemberAx,
	ControlMemberAy,
	ControlMemberAz,
	ControlMemberGAroll,
	ControlMemberGApitch,
	ControlMemberGAyaw
};

class FilterBase {
public:
	FilterBase() {
		transfer_func_.setIdentity();
	}
	~FilterBase() {}

	void setState(const Eigen::VectorXd& state);

	double normalize_angle(double angle);

protected:
	Eigen::VectorXd state_;
	Eigen::MatrixXd control_acceleration_;
	Eigen::MatrixXd transfer_func_;
};

}


