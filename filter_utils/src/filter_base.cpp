#include "filter_base.h"


void FilterBase::setState(const Eigen::VectorXd& state) {
	state_ = state;
}