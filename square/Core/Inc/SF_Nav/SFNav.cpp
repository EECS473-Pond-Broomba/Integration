/*
 * SFNav.cpp
 *
 *  Created on: Oct 26, 2020
 *      Author: rishgoel
 */

#include <SF_Nav/SFNav.h>
#include <cmath>
#define WNOISE 0.1
#define VNOISE 0.1



SF_Nav::SF_Nav() {
	// TODO Auto-generated constructor stub
	gps.has_data = false;
}

SF_Nav::~SF_Nav() {
	// TODO Auto-generated destructor stub
}

void SF_Nav::init(UART_HandleTypeDef* uh, I2C_HandleTypeDef* ih, float refresh_time)
{
	//Initialize IMU and GPS
	gps.init(uh);
	imu.initializeIMU(ih);
	//Initialize all matrices and set refresh time
	t = refresh_time;


//	h << Eigen::Matrix4f::Identity();


	// EKF
	I.setIdentity();	// 6x6 Identity
	// f is a function of previous state and action
	f << 1, 0, 0, t, 0, 0, 0.5*t*t, 0,
		 0, 1, 0, 0, t, 0, 0, 		0.5*t*t,
		 0, 0, 1, 0, 0, t, 0, 		0,
		 0, 0, 0, 1, 0, 0, t, 		0,
		 0, 0, 0, 0, 1, 0, 0, 		t,
		 0, 0, 0, 0, 0, 1, 0, 		0;
	// F is a jacobian of f*states vector w.r.t each state (acceleration not a state)
	F << 1, 0, 0, t, 0, 0,
		 0, 1, 0, 0, t, 0,
		 0, 0, 1, 0, 0, t,
		 0, 0, 0, 1, 0, 0,
		 0, 0, 0, 0, 1, 0,
		 0, 0, 0, 0, 0, 1;
	P_n = I;
	P_pred = I;
	//TODO: Get an estimate for w, Q and v, R
	w << WNOISE,
		WNOISE,
		WNOISE,
		WNOISE*10.0,
		WNOISE,
		WNOISE;
	// Jacobian of w w.r.t states
	W << WNOISE, 0, 0, 0, 0, 0,
		 0, WNOISE, 0, 0, 0, 0,
		 0, 0, WNOISE, 0, 0, 0,
		 0, 0, 0, WNOISE*10.0, 0, 0,
		 0, 0, 0, 0, WNOISE, 0,
		 0, 0, 0, 0, 0, WNOISE;
//	Q << Eigen::Matrix6f::Identity();
	Q = I;
	v << VNOISE,
		VNOISE,
		VNOISE,
		VNOISE*10.0,
		VNOISE*1.0,
		VNOISE;
	// Jacobian of w w.r.t states
	V << VNOISE, 0, 0, 0, 0, 0,
		 0, VNOISE, 0, 0, 0, 0,
		 0, 0, VNOISE, 0, 0, 0,
		 0, 0, 0, VNOISE*10.0, 0, 0,
		 0, 0, 0, 0, VNOISE*1.0, 0,
		 0, 0, 0, 0, 0, VNOISE;
//	R << Eigen::Matrix6f::Identity();
//	h << Eigen::Matrix6f::Identity();
//	H << Eigen::Matrix6f::Identity();
	R = I;
	h = I;
	H = I;

	x_n <<  0,
			0,
			imu.getOrientation(IMU::Axes::z),
			0,
			0,
			0;
	prev_location.latitude = 0.0;
	prev_location.longitude = 0.0;
}

void SF_Nav::update()
{
	double dist, bearing, gpsBearing;
//	imu.calculateLinearVelocity();
	bearing = imu.getOrientation(IMU::Axes::z);
	u_n <<  imu.getLinearAcceleration(IMU::Axes::x)*cosd(bearing)+imu.getLinearAcceleration(IMU::Axes::y)*sind(bearing),
			imu.getLinearAcceleration(IMU::Axes::x)*sind(bearing)+imu.getLinearAcceleration(IMU::Axes::y)*cosd(bearing);
//	u_n <<  imu.getLinearAcceleration(IMU::Axes::x),
//			imu.getLinearAcceleration(IMU::Axes::y);
	// Step 1: Predicted mean
	muu << x_n, u_n;	// Concatenate state at n-1 and actions
	x_pred = f*muu;		// Get next predicted state

	// Step 2: Predicted covariance
	P_pred = F*P_pred*F.transpose()+W*Q*W.transpose();
//	bearing = imu.getOrientation(IMU::Axes::z);
//	vTaskDelay(500);
	//Get inputs u_n and z_n
	if(gps.update()) {
		prev_location = curr_location;
		curr_location = gps.getPosition();
		// If prev_location is not set yet, set it to curr_location so our dist wont be 2000 miles
		if(prev_location.latitude < 0.1 && prev_location.latitude > -0.1) {
			prev_location = curr_location;
		}
//		imu.calculateLinearVelocity();
		curr_vel = gps.getVelocity();
		lwgps_distance_bearing(prev_location.latitude, prev_location.longitude, curr_location.latitude, curr_location.longitude, &dist, &gpsBearing);
//		bearing = imu.getOrientation(IMU::Axes::z);
		//Now convert the distance and bearing to and x and y
//		state.x = state.x + sind(bearing)* dist;
//		state.y = state.y + cosd(bearing)* dist;
//		state.b = bearing;
//		state.vX = sind(bearing) * curr_vel.speed;
//		state.vY = cosd(bearing) * curr_vel.speed;
//		state.vB = imu.getAngVel(IMU::Axes::z);

		// Set u_n and z_n, turn IMU frame into world frame
//		u_n <<  imu.getLinearAcceleration(IMU::Axes::x)*cosd(bearing)+imu.getLinearAcceleration(IMU::Axes::y)*sind(bearing),
//				imu.getLinearAcceleration(IMU::Axes::x)*sind(bearing)+imu.getLinearAcceleration(IMU::Axes::y)*cosd(bearing);
		z_n <<  x_n(0) + sind(bearing)* dist,
				x_n(1) + cosd(bearing)* dist,
				bearing,
				sind(bearing) * curr_vel.speed,
				cosd(bearing) * curr_vel.speed,
				imu.getAngVel(IMU::Axes::z);

		// Step 1: Predicted mean
//		muu << x_n, u_n;	// Concatenate state at n-1 and actions
//		x_pred = f*muu;		// Get next predicted state

		// Step 2: Predicted covariance
//		P_pred = F*P_pred*F.transpose()+W*Q*W.transpose();

		// Step 3: Innovation
		y = z_n-x_pred;

		// Step 4: Innovation covariance
		S = H*P_pred*H.transpose()+V*R*V.transpose();

		// Step 5: Filter gain
		K_n = P_pred*H.transpose()*S.inverse();

		// Step 6: Corrected mean
		x_n = x_pred+K_n*y;

		// Step 7: Corrected covariance
		P_n = (I-K_n*H)*P_pred;

		if(posCtr < LOGLENGTH) {
			xPosLog[posCtr] = x_n(0);
			yPosLog[posCtr] = x_n(1);
			posCtr++;
		}
	}
}

state_var SF_Nav::get_state() {
	state_var current_state = { .x = x_n(0),
								.y = x_n(1),
								.b = x_n(2),
								.vX = x_n(3),
								.vY = x_n(4),
								.vB = x_n(5)};
	return current_state;
}

double SF_Nav::get_bearing() {
	return imu.getOrientation(IMU::Axes::z);
}
