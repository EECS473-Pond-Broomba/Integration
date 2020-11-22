/*
 * GPS.cpp
 *
 *  Created on: Oct 19, 2020
 *      Author: rishgoel
 */

#include <GPS/GPS.h>
#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"

GPS::GPS() {
	// TODO Auto-generated constructor stub
	has_data = false;
}

GPS::~GPS() {
	// TODO Auto-generated destructor stub
}

void GPS::init(UART_HandleTypeDef* handle)
{
	huart = handle;

	lwgps_init(&lwgps_handle);

	HAL_UART_Receive_IT(huart, (uint8_t*)data, sizeof(data));
}

bool GPS::update()
{
	if(has_data)
	{
		//Store buffer here
		char temp[GPS_MSG_SIZE];
		strcpy(temp, data);
		has_data = false;
		lwgps_process(&lwgps_handle, temp, sizeof(temp));

		if(lwgps_handle.is_valid)
		{
			curr_position.latitude = lwgps_handle.latitude;
			curr_position.longitude = lwgps_handle.longitude;

			curr_velocity.speed = lwgps_to_speed(lwgps_handle.speed, lwgps_speed_mps);
			curr_velocity.bearing = lwgps_handle.course;

			//HAL_UART_Receive_IT(huart, (uint8_t*)data, GPS_MSG_SIZE);

			return true;
		}
		else
		{
			//HAL_UART_Receive_IT(huart, (uint8_t*)data, GPS_MSG_SIZE);

			return false;
		}


	}
	else
	{
		HAL_UART_Receive_IT(huart, (uint8_t*)data, GPS_MSG_SIZE);
		return false;
	}

}
