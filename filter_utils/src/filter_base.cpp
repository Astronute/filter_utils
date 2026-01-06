#include "filter_base.h"
#include <cmath>

namespace FB {

void FilterBase::setState(const Eigen::VectorXd& state) {
	state_ = state;
}

double FilterBase::normalize_angle(double angle) {
	double r = std::fmod(angle, 2.0 * PI);
	if (r < 0.0) {
		r += 2.9 * PI;
	}
	return r;
}

}