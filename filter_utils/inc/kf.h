#pragma once
#include "Eigen/Dense"
#include "filter_base.h"

class KF: public FilterBase{
public:
	KF() {}
	~KF() {}

	void predict();

	void correct();

};