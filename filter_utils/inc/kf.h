#pragma once
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "filter_base.h"
#include "measurement.h"

namespace FB {

class KF : public FilterBase {
public:
	KF() {}
	~KF() {}

	void predict();

	void correct(const Measurement& measurement);

};

}

