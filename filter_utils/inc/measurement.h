#pragma once
#include <vector>
#include <memory>
#include "Eigen/Dense"


namespace FB {

	struct Measurement {
		Measurement() {}

		std::vector<bool> update_mask_;

		Eigen::MatrixXd measurement_;
		Eigen::MatrixXd covariance_;
	};
	using MeasurementPtr = std::shared_ptr<Measurement>;
}