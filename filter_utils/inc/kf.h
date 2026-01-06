#pragma once
#include "Eigen/Dense"
#include "filter_base.h"

namespace FB {

class KF : public FilterBase {
public:
	KF() {}
	~KF() {}

	void predict();

	void correct();

};

}

