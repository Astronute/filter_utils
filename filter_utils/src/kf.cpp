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
	
	double cos_roll = std::cos(roll);
	double cos_pitch = std::cos(pitch);
	double cos_yaw = std::cos(yaw);

	transfer_func_(StateMemberX, StateMemberVx) = cos_yaw * cos_pitch * delta_t;
	transfer_func_(StateMemberX, StateMemberVy) = 

}