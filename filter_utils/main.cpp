#include <iostream>
#include "Eigen/Dense"
#include "kf.h"
#include <queue>
#include <memory>

struct Measurement {
	Measurement() {}

	Measurement(int val) {
		val_ = val;
	}

	int val_;

	bool operator()(const std::shared_ptr<Measurement>& a, const std::shared_ptr<Measurement>& b) {
		return (*this)(*(a.get()), *(b.get()));
	}

	bool operator()(const Measurement& a, const Measurement& b) {
		return a.val_ > b.val_;
	}
};

void main() {
	std::priority_queue<Measurement, std::vector<Measurement>, Measurement> iq;
	iq.push(Measurement(3));
	iq.push(Measurement(1));
	iq.push(Measurement(6));
	iq.push(Measurement(2));

	while (!iq.empty()) {
		std::cout << iq.top().val_ << std::endl;
		iq.pop();
	}
}