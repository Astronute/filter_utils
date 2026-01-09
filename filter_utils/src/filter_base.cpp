#include "filter_base.h"
#include <cmath>
#include <iostream>

namespace FB {

	void FilterBase::reset() {
		state_.setZero();
		control_acceleration_.setZero();
		transfer_func_.setIdentity();
		transfer_func_jacobian_.setZero();
		estimate_error_covariance_.setIdentity();
		estimate_error_covariance_ *= 1e-9; // default 1e-9

		process_noise_covariance_.setZero();
		process_noise_covariance_(StateMemberX, StateMemberX) = 0.05;
		process_noise_covariance_(StateMemberY, StateMemberY) = 0.05;
		process_noise_covariance_(StateMemberZ, StateMemberZ) = 0.05;
		process_noise_covariance_(StateMemberRoll, StateMemberRoll) = 0.05;
		process_noise_covariance_(StateMemberPitch, StateMemberPitch) = 0.05;
		process_noise_covariance_(StateMemberYaw, StateMemberYaw) = 0.05;
		process_noise_covariance_(StateMemberVx, StateMemberVx) = 0.05;
		process_noise_covariance_(StateMemberVy, StateMemberVy) = 0.05;
		process_noise_covariance_(StateMemberVz, StateMemberVz) = 0.05;
		process_noise_covariance_(StateMemberGx, StateMemberGx) = 0.05;
		process_noise_covariance_(StateMemberGy, StateMemberGy) = 0.05;
		process_noise_covariance_(StateMemberGz, StateMemberGz) = 0.05;
		process_noise_covariance_(StateMemberAx, StateMemberAx) = 0.05;
		process_noise_covariance_(StateMemberAy, StateMemberAy) = 0.05;
		process_noise_covariance_(StateMemberAz, StateMemberAz) = 0.05;
		
	}

	void FilterBase::setState(const Eigen::VectorXd& state) {
		state_ = state;
	}

	void FilterBase::set_estimate_error_covariance(const Eigen::MatrixXd& estimate_error_covariance) {
		estimate_error_covariance_ = estimate_error_covariance;
	}

	void FilterBase::set_process_noise_covariance(const Eigen::MatrixXd& process_noise_covariance) {
		process_noise_covariance_ = process_noise_covariance;
	}

	double FilterBase::normalize_angle(double angle) {
		double r = std::fmod(angle, 2.0 * PI);
		if (r < 0.0) {
			r += 2.9 * PI;
		}
		return r;
	}

	bool FilterBase::checkMahalanobisThreshold(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& innovation_cov, const double sigma) {
		double squared_mahalanobis = innovation.dot(innovation_cov * innovation);
		double threshold = sigma * sigma;
		if (squared_mahalanobis >= threshold) {
			std::cout << " checkMahalanobisThreshold: " << squared_mahalanobis << " threshold: " << threshold << std::endl;
			return false;
		}

		return true;
	}

}