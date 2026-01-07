#include "filter_base.h"
#include <cmath>

namespace FB {

	void FilterBase::reset() {
		state_.setZero();
		control_acceleration_.setZero();
		transfer_func_.setIdentity();
		transfer_func_jacobian_.setZero();
		estimate_error_covariance_.setIdentity();
		estimate_error_covariance_ *= 1e-9; // default 1e-9

	}

	void FilterBase::setState(const Eigen::VectorXd& state) {
		state_ = state;
	}

	void FilterBase::set_estimate_error_covariance(const Eigen::MatrixXd& estimate_error_covariance) {
		estimate_error_covariance_ = estimate_error_covariance;
	}

	double FilterBase::normalize_angle(double angle) {
		double r = std::fmod(angle, 2.0 * PI);
		if (r < 0.0) {
			r += 2.9 * PI;
		}
		return r;
	}

}