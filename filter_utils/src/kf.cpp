#include "kf.h"


namespace FB {

	void KF::predict() {
		double delta_t = 0.01;

		double roll = state_(StateMemberRoll);
		double pitch = state_(StateMemberPitch);
		double yaw = state_(StateMemberYaw);
		double x_vel = state_(StateMemberVx);
		double y_vel = state_(StateMemberVy);
		double z_vel = state_(StateMemberVz);
		double x_acc = state_(StateMemberAx);
		double y_acc = state_(StateMemberAy);
		double z_acc = state_(StateMemberAz);
		double gyro_x = state_(StateMemberGx);
		double gyro_y = state_(StateMemberGy);
		double gyro_z = state_(StateMemberGz);

		// 姿态表示旋转顺序yaw->pitch->roll z->y->x
		double cos_roll = std::cos(roll);
		double cos_pitch = std::cos(pitch);
		double cos_yaw = std::cos(yaw);
		double sin_roll = std::sin(roll);
		double sin_pitch = std::sin(pitch);
		double sin_yaw = std::sin(yaw);
		double sec_pitch = 1 / cos_pitch;
		double tan_pitch = sin_pitch * sec_pitch;

		// state predict
		//transfer_func_.setIdentity();
		transfer_func_(StateMemberX, StateMemberVx) = cos_yaw * cos_pitch * delta_t; // cos_yaw * cos_pitch * delta_t * Vx = 世界坐标系下x轴的线速度对世界坐标系下x轴坐标的影响
		transfer_func_(StateMemberX, StateMemberVy) = (-1.0 * sin_yaw * cos_roll + cos_yaw * sin_pitch * sin_roll) * delta_t;
		transfer_func_(StateMemberX, StateMemberVz) = (sin_yaw * sin_roll + cos_yaw * sin_pitch * cos_roll) * delta_t;
		transfer_func_(StateMemberX, StateMemberAx) = 0.5 * transfer_func_(StateMemberX, StateMemberVx) * delta_t; // 0.5*Ax*t^2
		transfer_func_(StateMemberX, StateMemberAy) = 0.5 * transfer_func_(StateMemberX, StateMemberVy) * delta_t;
		transfer_func_(StateMemberX, StateMemberAz) = 0.5 * transfer_func_(StateMemberX, StateMemberVz) * delta_t;
		transfer_func_(StateMemberY, StateMemberVx) = sin_yaw * cos_pitch * delta_t;
		transfer_func_(StateMemberY, StateMemberVy) = (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) * delta_t;
		transfer_func_(StateMemberY, StateMemberVz) = (-1.0 * cos_yaw * sin_roll + sin_yaw * sin_pitch * cos_roll) * delta_t;
		transfer_func_(StateMemberY, StateMemberAx) = 0.5 * transfer_func_(StateMemberY, StateMemberVx) * delta_t;
		transfer_func_(StateMemberY, StateMemberAy) = 0.5 * transfer_func_(StateMemberY, StateMemberVy) * delta_t;
		transfer_func_(StateMemberY, StateMemberAz) = 0.5 * transfer_func_(StateMemberY, StateMemberVz) * delta_t;
		transfer_func_(StateMemberZ, StateMemberVx) = (-1.0 * sin_pitch) * delta_t;
		transfer_func_(StateMemberZ, StateMemberVy) = (cos_pitch * sin_roll) * delta_t;
		transfer_func_(StateMemberZ, StateMemberVz) = (cos_pitch * cos_roll) * delta_t;
		transfer_func_(StateMemberZ, StateMemberAx) = 0.5 * transfer_func_(StateMemberZ, StateMemberVx) * delta_t;
		transfer_func_(StateMemberZ, StateMemberAy) = 0.5 * transfer_func_(StateMemberZ, StateMemberVy) * delta_t;
		transfer_func_(StateMemberZ, StateMemberAz) = 0.5 * transfer_func_(StateMemberZ, StateMemberVz) * delta_t;

		transfer_func_(StateMemberRoll, StateMemberGx) = delta_t;
		transfer_func_(StateMemberRoll, StateMemberGy) = sin_roll * tan_pitch * delta_t;
		transfer_func_(StateMemberRoll, StateMemberGz) = cos_roll * tan_pitch * delta_t;
		transfer_func_(StateMemberPitch, StateMemberGx) = 0;
		transfer_func_(StateMemberPitch, StateMemberGy) = cos_roll * delta_t;
		transfer_func_(StateMemberPitch, StateMemberGz) = -1.0 * sin_roll * delta_t;
		transfer_func_(StateMemberYaw, StateMemberGx) = 0;
		transfer_func_(StateMemberYaw, StateMemberGy) = sin_roll * sec_pitch * delta_t;
		transfer_func_(StateMemberYaw, StateMemberGz) = cos_roll * sec_pitch * delta_t;

		transfer_func_(StateMemberVx, StateMemberAx) = delta_t; // 状态矩阵中的加速度和速度都在体坐标系下所以不用旋转
		transfer_func_(StateMemberVy, StateMemberAy) = delta_t;
		transfer_func_(StateMemberVz, StateMemberAz) = delta_t;

		// Linearilze
		double x_coeff = 0.0;
		double y_coeff = 0.0;
		double z_coeff = 0.0;
		double one_half_at_squared = 0.5 * delta_t * delta_t;

		x_coeff = 0.0;
		y_coeff = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
		z_coeff = sin_yaw * cos_roll - cos_yaw * sin_pitch * sin_roll;
		double dFx_dR = (y_coeff * y_vel + z_coeff * z_vel) * delta_t + (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
		x_coeff = -1.0 * cos_yaw * sin_pitch;
		y_coeff = cos_yaw * cos_pitch * sin_roll;
		z_coeff = cos_yaw * cos_pitch * cos_roll;
		double dFx_dP = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_t + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
		x_coeff = -1.0 * sin_yaw * cos_pitch;
		y_coeff = -1.0 * sin_yaw * sin_pitch * sin_roll - cos_yaw * cos_roll;
		z_coeff = -1.0 * sin_yaw * sin_pitch * cos_roll + cos_yaw * sin_roll;
		double dFx_dY = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_t + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;

		x_coeff = 0.0;
		y_coeff = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
		z_coeff = -1.0 * sin_yaw * sin_pitch * sin_roll - cos_yaw * cos_roll;
		double dFy_dR = (y_coeff * y_vel + z_coeff * z_vel) * delta_t + (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
		x_coeff = -1.0 * sin_yaw * sin_pitch;
		y_coeff = sin_yaw * cos_pitch * sin_roll;
		z_coeff = sin_yaw * cos_pitch * cos_roll;
		double dFy_dP = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_t + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
		x_coeff = cos_yaw * cos_pitch;
		y_coeff = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
		z_coeff = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
		double dFy_dY = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_t + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;

		x_coeff = 0.0;
		y_coeff = cos_pitch * cos_roll;
		z_coeff = -1.0 * cos_pitch * sin_roll;
		double dFz_dR = (y_coeff * y_vel + z_coeff * z_vel) * delta_t + (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
		x_coeff = -1.0 * cos_pitch;
		y_coeff = -1.0 * sin_pitch * sin_roll;
		z_coeff = -1.0 * sin_pitch * cos_roll;
		double dFz_dP = (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_t + (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
		//double dFz_dY = 0.0;

		double dFR_dR = 1.0 + (cos_roll * tan_pitch * gyro_y - sin_roll * tan_pitch * gyro_z) * delta_t;
		double dFR_dP = (sin_roll * sec_pitch * sec_pitch * gyro_y + cos_roll * sec_pitch * sec_pitch * gyro_z) * delta_t;
		double dFR_dY = 0.0;
		double dFP_dR = (-1.0 * sin_roll * gyro_y - cos_roll * gyro_z) * delta_t;
		double dFP_dP = 1.0;
		double dFP_dY = 0.0;
		double dFY_dR = (cos_roll * sec_pitch * gyro_y - sin_roll * sec_pitch * gyro_z) * delta_t;
		double dFY_dP = (sin_roll * tan_pitch * sec_pitch * gyro_y + cos_roll * tan_pitch * sec_pitch * gyro_z) * delta_t;
		double dFY_dY = 1.0;


		transfer_func_jacobian_ = transfer_func_;
		transfer_func_jacobian_(StateMemberX, StateMemberRoll) = dFx_dR;
		transfer_func_jacobian_(StateMemberX, StateMemberPitch) = dFx_dP;
		transfer_func_jacobian_(StateMemberX, StateMemberYaw) = dFx_dY;
		transfer_func_jacobian_(StateMemberY, StateMemberRoll) = dFy_dR;
		transfer_func_jacobian_(StateMemberY, StateMemberPitch) = dFy_dP;
		transfer_func_jacobian_(StateMemberY, StateMemberYaw) = dFy_dY;
		transfer_func_jacobian_(StateMemberZ, StateMemberRoll) = dFz_dR;
		transfer_func_jacobian_(StateMemberZ, StateMemberPitch) = dFz_dP;
		transfer_func_jacobian_(StateMemberRoll, StateMemberRoll) = dFR_dR;
		transfer_func_jacobian_(StateMemberRoll, StateMemberPitch) = dFR_dP;
		transfer_func_jacobian_(StateMemberPitch, StateMemberRoll) = dFP_dR;
		transfer_func_jacobian_(StateMemberYaw, StateMemberRoll) = dFY_dR;
		transfer_func_jacobian_(StateMemberYaw, StateMemberPitch) = dFY_dP;

		// control terms
		// 角加速度指令永远存在，有时可以不输出加速度（设计上角速度环是内环需要持续输出(高优先级)，线速度环是外环可以休眠(低优先级)）
		state_(StateMemberGx) += control_acceleration_(ControlMemberGAroll) * delta_t;
		state_(StateMemberGy) += control_acceleration_(ControlMemberGApitch) * delta_t;
		state_(StateMemberGz) += control_acceleration_(ControlMemberGAyaw) * delta_t;
		state_(StateMemberAx) = control_update_mask_[ControlMemberAx] ? control_acceleration_(ControlMemberAx) : state_(StateMemberAx); // wait update
		state_(StateMemberAy) = control_update_mask_[ControlMemberAy] ? control_acceleration_(ControlMemberAy) : state_(StateMemberAy);
		state_(StateMemberAz) = control_update_mask_[ControlMemberAz] ? control_acceleration_(ControlMemberAz) : state_(StateMemberAz);

		// x_{k} = A*x_{k-1} + B*u_{k-1}
		state_ = transfer_func_ * state_;
		state_(StateMemberRoll) = normalize_angle(state_(StateMemberRoll));
		state_(StateMemberPitch) = normalize_angle(state_(StateMemberPitch));
		state_(StateMemberYaw) = normalize_angle(state_(StateMemberYaw));
		std::cout << "estimate state " << std::endl;
		std::cout << state_.transpose() << std::endl;
		std::cout << " process noise cov" << std::endl;
		std::cout << process_noise_covariance_ << std::endl;

		// P'_{k} = AP_{k-1}A + Q
		estimate_error_covariance_ = transfer_func_jacobian_ * estimate_error_covariance_ * transfer_func_jacobian_.transpose();
		estimate_error_covariance_.noalias() += delta_t * process_noise_covariance_;
	}


	void KF::correct(const Measurement& measurement) {

		std::vector<size_t> update_indices;
		for (int i = 0; i < measurement.update_mask_.size(); ++i) {
			if (measurement.update_mask_[i]) {
				if (std::isnan(measurement.measurement_(i))) {
					std::cout << " vector nan " << std::endl;
				}
				else if (std::isinf(measurement.measurement_(i))) {
					std::cout << " vector inf " << std::endl;
				}
				else {
					update_indices.push_back(i);
				}
			}
		}
		
		size_t update_size = update_indices.size();
		std::cout << " update indices size: " << update_size << std::endl;

		Eigen::VectorXd state_subset(update_size); // x_{k-1}
		Eigen::VectorXd measurement_subset(update_size); // z_k
		Eigen::MatrixXd measurement_error_covariance(update_size, update_size); // R
		Eigen::MatrixXd state_to_measurement_subset(update_size, state_.rows()); // H
		Eigen::MatrixXd kalman_gain_subset(state_.rows(), update_size); // K
		Eigen::VectorXd innovation_subset(update_size); // z - Hx

		state_subset.setZero();
		measurement_subset.setZero();
		measurement_error_covariance.setZero();
		state_to_measurement_subset.setZero();
		kalman_gain_subset.setZero();
		innovation_subset.setZero();

		for (size_t i = 0; i < update_size; ++i) {
			state_subset(i) = state_(update_indices[i]);
			measurement_subset(i) = measurement.measurement_(update_indices[i]);

			for (size_t j = 0; j < update_size; ++j) {
				measurement_error_covariance(i, j) = measurement.covariance_(update_indices[i], update_indices[j]);
			}
			if (measurement_error_covariance(i, i) < 0) {
				measurement_error_covariance(i, i) = std::fabs(measurement_error_covariance(i, i));
				std::cout << " R negative cov " << std::endl;
			}
			else if (measurement_error_covariance(i, i) < 1e-9) {
				measurement_error_covariance(i, i) = 1e-9;
				std::cout << " R small cov " << std::endl;
			}
		}
		for (size_t i = 0; i < update_size; ++i) {
			state_to_measurement_subset(i, update_indices[i]) = 1;
		}

		// calcualate kalman gain: K = (PH'/(HPH'+R))
		// P:先验误差协方差 R:测量误差协方差
		Eigen::MatrixXd pht = estimate_error_covariance_ * state_to_measurement_subset.transpose();
		Eigen::MatrixXd hphr_inverse = (state_to_measurement_subset * pht + measurement_error_covariance).inverse();
		kalman_gain_subset.noalias() = pht * hphr_inverse;
		// 测量值 - 先验估计
		innovation_subset = measurement_subset - state_subset;

		for (size_t i = 0; i < update_size; ++i) {
			if (update_indices[i] == StateMemberRoll || update_indices[i] == StateMemberPitch || update_indices[i] == StateMemberYaw) {
				innovation_subset(i) = normalize_angle(innovation_subset(i));
			}
		}

		if (checkMahalanobisThreshold(innovation_subset, hphr_inverse, measurement.mahalanobis_thresh_)) {
			// 后验估计
			state_.noalias() += kalman_gain_subset * innovation_subset;

			// I-KH
			Eigen::MatrixXd ikh = Eigen::MatrixXd::Identity(state_.rows(), state_.rows());
			ikh.noalias() -= kalman_gain_subset * state_to_measurement_subset;
			// P_{K} = (I-KH)P'_{k}(I-KH)' + KRK 根据先验误差协方差和测量误差协方差更新后验误差协方差
			estimate_error_covariance_ = ikh * estimate_error_covariance_ * ikh.transpose();
			estimate_error_covariance_.noalias() += kalman_gain_subset * measurement_error_covariance * kalman_gain_subset.transpose();

			state_(StateMemberRoll) = normalize_angle(state_(StateMemberRoll));
			state_(StateMemberPitch) = normalize_angle(state_(StateMemberPitch));
			state_(StateMemberYaw) = normalize_angle(state_(StateMemberYaw));

			std::cout << " correct state: " << state_.transpose() << std::endl;
		}

	}


}

