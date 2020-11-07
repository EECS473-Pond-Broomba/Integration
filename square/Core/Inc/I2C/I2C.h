/*
 * I2C.h
 *
 *  Created on: Oct 18, 2020
 *      Author: rishgoel
 */

#ifndef INC_I2C_I2C_H_
#define INC_I2C_I2C_H_

#include "stm32f4xx_hal.h"
#define I2C_TIMEOUT 2

class I2C {
public:
	I2C();
	virtual ~I2C();

	void init(I2C_HandleTypeDef* handle, uint8_t address)
	{
		hi2c = handle;
		addr = address;
	}

	HAL_StatusTypeDef write8(uint8_t reg, uint8_t val);

	int8_t read8(uint8_t reg);

	int16_t read16(uint8_t reg);

private:
	I2C_HandleTypeDef* hi2c;
	uint8_t addr;
};

#endif /* INC_I2C_I2C_H_ */
