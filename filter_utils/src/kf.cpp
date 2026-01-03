#include "kf.h"

void KF::predict() {
	
}

void KF::correct() {

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
	transfer_func_(StateMemberRoll, StateMemberGy) = 
}