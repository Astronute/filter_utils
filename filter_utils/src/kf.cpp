#include "kf.h"


namespace FB {

void KF::predict() {
	double delta_t = 1.0;

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

	// ×ËÌ¬±íÊ¾Ðý×ªË³Ðòyaw->pitch->roll z->y->x
	double cos_roll = std::cos(roll);
	double cos_pitch = std::cos(pitch);
	double cos_yaw = std::cos(yaw);
	double sin_roll = std::sin(roll);
	double sin_pitch = std::sin(pitch);
	double sin_yaw = std::sin(yaw);
	double sec_pitch = 1 / cos_pitch;
	double tan_pitch = sin_pitch * sec_pitch;

	// state predict
	transfer_func_(StateMemberX, StateMemberVx) = cos_yaw * cos_pitch * delta_t;
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

	transfer_func_(StateMemberVx, StateMemberAx) = delta_t;
	transfer_func_(StateMemberVy, StateMemberAy) = delta_t;
	transfer_func_(StateMemberVz, StateMemberAz) = delta_t;
	// control terms
	state_(StateMemberGx) += control_acceleration_(ControlMemberGAroll) * delta_t;
	state_(StateMemberGy) += control_acceleration_(ControlMemberGApitch) * delta_t;
	state_(StateMemberGz) += control_acceleration_(ControlMemberGAyaw) * delta_t;

	// wait update error
	state_(StateMemberAx) = control_acceleration_(ControlMemberAx);
	state_(StateMemberAy) = control_acceleration_(ControlMemberAy);
	state_(StateMemberAz) = control_acceleration_(ControlMemberAz);

	state_ = transfer_func_ * state_;
	state_(StateMemberRoll) = normalize_angle(state_(StateMemberRoll));
	state_(StateMemberPitch) = normalize_angle(state_(StateMemberPitch));
	state_(StateMemberYaw) = normalize_angle(state_(StateMemberYaw));



}

void KF::correct() {

}


}

