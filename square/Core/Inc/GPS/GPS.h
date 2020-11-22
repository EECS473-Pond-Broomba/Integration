/*
 * GPS.h
 *
 *  Created on: Oct 15, 2020
 *      Author: rishgoel
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "usart.h"
#include "GPS/lwgps.h"
#include "cmath"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t gps_sem;

#define GPS_MSG_SIZE 144
#define GPS_INACCURACY 1.5

struct location{
	double latitude;
	double longitude;

};

struct velocity{
	double speed;
	double bearing;
};

struct geofence{
	double latitude;
	double longitude;
	double radius;
	bool active;
};

enum geofenceStatus{
	IN_GF,
	OUT_GF,
	EDGE_GF,
	INACTIVE,
};

class GPS {
public:
	GPS();
	virtual ~GPS();

	void init(UART_HandleTypeDef* handle);

	bool update();

	bool addGeoFence(double latitude, double longitude, double radius)
	{
		if(radius <= GPS_INACCURACY)
		{
			return false;
		}
		gf_status.latitude = latitude;
		gf_status.longitude = longitude;
		gf_status.radius = radius;
		gf_status.active = true;
		return true;
	}

	//Returns 1 if in geofence, returns 0 if outside geofence,
	geofenceStatus checkGeoFence()
	{
		if(!gf_status.active)
		{
			return INACTIVE;
		}

		double dist, bear;
		lwgps_distance_bearing(curr_position.latitude, curr_position.longitude, gf_status.latitude, gf_status.longitude, &dist, &bear);

		if(dist > (gf_status.radius + GPS_INACCURACY))
		{
			return OUT_GF;
		}
		else if(dist < (gf_status.radius - GPS_INACCURACY))
		{
			return IN_GF;
		}
		else
		{
			return EDGE_GF;
		}
	}

	location getPosition()
	{
		return curr_position;
	}
	velocity getVelocity()
	{
		return curr_velocity;
	}

	char data[GPS_MSG_SIZE];
	bool has_data;

private:
	UART_HandleTypeDef* huart;
	lwgps_t lwgps_handle;

	location curr_position;
	velocity curr_velocity;

	geofence gf_status;

};

#endif /* INC_GPS_H_ */
