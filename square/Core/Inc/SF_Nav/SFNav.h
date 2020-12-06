/*
 * SFNav.h
 *
 *  Created on: Oct 26, 2020
 *      Author: rishgoel
 */

#ifndef INC_SF_NAV_SFNAV_H_
#define INC_SF_NAV_SFNAV_H_
#define LOGLENGTH 200
#include "IMU/IMU.h"
#include "GPS/GPS.h"
#include "Eigen/Dense"
#include "main.h"
#include <stdio.h>"

struct state_var
{
	double x;
	double y;
	double b;
	double vX;
	double vY;
	double vB;
};


class SF_Nav {
public:
	SF_Nav();
	virtual ~SF_Nav();

	void init(UART_HandleTypeDef* uh, I2C_HandleTypeDef* ih, float refresh_time);

	void update();
	state_var get_state();
	GPS gps;
	double get_bearing();
	bool get_valid();
	void setCalibration(bool startCalibrate) {
		calibrate = startCalibrate;
	}

private:
	IMU imu;
//	Eigen::Matrix4f A, B, Q, H, R, P_pred, P, S, K;
//	Eigen::RowVector4f x_pred, x, u_n, y, z_n;
	// EKF
	Eigen::Matrix<float, 6, 8> f;
	Eigen::Matrix<float, 6, 6> F, P_pred, P_n, W, V, Q, R, h, H, S, K_n, I;
	Eigen::Matrix<float, 6, 1> x_pred, x_n, y, z_n, w, v;
	Eigen::Matrix<float, 8, 1> muu;	// Concatenated mu and u for step 1
	Eigen::Matrix<float, 2, 1> u_n;	// Action, contains x and y acceleration
	float t;
	location curr_location, prev_location;
	velocity curr_vel, prev_vel;
	state_var state;
	double xPosLog[LOGLENGTH];
	double yPosLog[LOGLENGTH];
	int posCtr = 0;
	bool validState = false;
	bool calibrate = false;

	inline double sind(double x)
	{
	    return sin(x * M_PI / 180);
	}

	inline double cosd(double x)
	{
	    return cos(x * M_PI / 180);
	}

};

#endif /* INC_SF_NAV_SFNAV_H_ */
