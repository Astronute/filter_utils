#pragma once

#include "Eigen/Dense"

namespace FB{

	const double PI = 3.141592653589793;
	const double TAU = 6.283185307179587;
	const int STATE_SIZE = 15;

	// control vector
	const int TWIST_SIZE = 6;

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
		FilterBase() : state_(STATE_SIZE), 
			transfer_func_(STATE_SIZE, STATE_SIZE),
			transfer_func_jacobian_(STATE_SIZE, STATE_SIZE),
			control_acceleration_(TWIST_SIZE),
			estimate_error_covariance_(STATE_SIZE, STATE_SIZE),
			process_noise_covariance_(STATE_SIZE, STATE_SIZE){
			reset();
		}
		~FilterBase() {}

		void reset();

		void setState(const Eigen::VectorXd& state);

		void set_estimate_error_covariance(const Eigen::MatrixXd & estimate_error_covariance);

		void set_process_noise_covariance(const Eigen::MatrixXd& process_noise_covariance);

		double normalize_angle(double angle);

	protected:
		Eigen::VectorXd state_;
		Eigen::VectorXd control_acceleration_;
		Eigen::MatrixXd transfer_func_;
		Eigen::MatrixXd transfer_func_jacobian_;
		Eigen::MatrixXd estimate_error_covariance_; // 先验估计协方差矩阵
		Eigen::MatrixXd process_noise_covariance_;
	};

}


