/*
 * I2C.cpp
 *
 *  Created on: Oct 18, 2020
 *      Author: rishgoel
 */

#include <I2C/I2C.h>

I2C::I2C() {
	// TODO Auto-generated constructor stub

}

I2C::~I2C() {
	// TODO Auto-generated destructor stub
}

HAL_StatusTypeDef I2C::write8(uint8_t reg, uint8_t value) {
	HAL_StatusTypeDef ret;
	// Combine reg and value into a buffer
	uint8_t buffer[2];
	buffer[0] = reg;
	buffer[1] = value;
	// Send buffer over
	ret = HAL_I2C_Master_Transmit(hi2c, addr << 1, buffer, 2, 2);
	return ret;
}

int8_t I2C::read8(uint8_t reg) {
	HAL_StatusTypeDef ret;
	uint8_t value = 0;
	// Tell sensor that we want to read from reg
	ret = HAL_I2C_Master_Transmit(hi2c, addr << 1, &reg, 1, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFF;
	}
	// Read 1 byte from reg
	ret = HAL_I2C_Master_Receive(hi2c, addr << 1, &value, 1, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFF;
	}
	return (int8_t)value;
}

int8_t I2C::read8()
{
	HAL_StatusTypeDef ret;
	uint8_t value = 0;
	// Read 1 byte from reg
	ret = HAL_I2C_Master_Receive(hi2c, addr << 1, &value, 1, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFF;
	}
	return (int8_t)value;
}

int16_t I2C::read16(uint8_t reg) {
	HAL_StatusTypeDef ret;
	uint8_t buffer[2];
	// Tell sensor that we want to read from reg
	ret = HAL_I2C_Master_Transmit(hi2c, addr << 1, &reg, 1, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFFFF;
	}
	// Read 1 byte from reg
	ret = HAL_I2C_Master_Receive(hi2c, addr << 1, buffer, 2, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFFFF;
	}
	// The LSB is always at the lower register address, so cast buffer[0] into 16 bits and shift it left by 8
	// And then OR with MSB to combine into 2 bytes
	int16_t value = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
	return value;
}

int16_t I2C::read16()
{
	HAL_StatusTypeDef ret;
	uint8_t buffer[2];

	// Read 1 byte from reg
	ret = HAL_I2C_Master_Receive(hi2c, addr << 1, buffer, 2, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFFFF;
	}
	// The LSB is always at the lower register address, so cast buffer[0] into 16 bits and shift it left by 8
	// And then OR with MSB to combine into 2 bytes
	int16_t value = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
	return value;
}
