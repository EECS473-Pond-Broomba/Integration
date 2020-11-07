/*
 * IMU.cpp
 *
 *  Created on: Oct 3, 2020
 *      Author: cy
 */

#include <IMU/IMU.h>

IMU::IMU() {
	// Auto-generated constructor stub
}

IMU::~IMU() {
	// Auto-generated destructor stub
}

void IMU::initializeIMU(I2C_HandleTypeDef* handle) {
	i2c.init(handle, IMU_I2C_ADDR);
	// Set mode to NDOF
	setMode(IMU_Mode::OPR_MODE_NDOF);
	// Set acceleration samples to 0 and time step to 0
	accelTimeSteps = 0;
	accelerationSamples[0][0] = 0.0;
	accelerationSamples[0][1] = 0.0;
	accelerationSamples[1][0] = 0.0;
	accelerationSamples[1][1] = 0.0;
	accelerationSamples[2][0] = 0.0;
	accelerationSamples[2][1] = 0.0;
	// Set Euler Angles units to degrees
	eulerAngleUnits = false;
	// Set Angular Rate units to Dps
	gyroscopeUnits = false;
	// Set Acceleration units to m/s^2
	totalAccelerationUnits = false;
	// Set Temperature units to C
	tempUnits = false;
	// Set data output format to Windows format
	i2c.write8(Registers::BNO055_UNIT_SEL_ADDR, 0x00);
	vTaskDelay(20);
}

double IMU::getOrientation(Axes axis) {
	// Set register we need to read depending on the axis passed in
	uint8_t registerToRead = 0;
	switch(axis) {
	case Axes::x:
		registerToRead = Registers::BNO055_EULER_P_LSB_ADDR;
		break;
	case Axes::y:
		registerToRead = Registers::BNO055_EULER_R_LSB_ADDR;
		break;
	case Axes::z:
		registerToRead = Registers::BNO055_EULER_H_LSB_ADDR;
		break;
	default:
		break;
	}

	// Read the data registers
	int16_t data = i2c.read16(registerToRead);
	// Section 3.6.5.4 of datasheet for conversion from LSBs to deg/rad
	return eulerAngleUnits ? (double)data / 900.0 : (double)data / 16.0;
}

double IMU::getAngVel(Axes axis) {
	// Set register we need to read depending on the axis passed in
	uint8_t registerToRead = 0;
	switch(axis) {
	case Axes::x:
		registerToRead = Registers::BNO055_GYRO_DATA_X_LSB_ADDR;
		break;
	case Axes::y:
		registerToRead = Registers::BNO055_GYRO_DATA_Y_LSB_ADDR;
		break;
	case Axes::z:
		registerToRead = Registers::BNO055_GYRO_DATA_Z_LSB_ADDR;
		break;
	default:
		break;
	}

	// Read the data registers
	int16_t data = i2c.read16(registerToRead);
	// Table 3-22 of datasheet for conversion from LSBs to Dps/Rps
	return gyroscopeUnits ? (double)data / 900.0 : (double)data / 16.0;
}

double IMU::getTotalAcceleration(Axes axis) {
	// Set register we need to read depending on the axis passed in
	uint8_t registerToRead = 0;
	switch(axis) {
	case Axes::x:
		registerToRead = Registers::BNO055_ACCEL_DATA_X_LSB_ADDR;
		break;
	case Axes::y:
		registerToRead = Registers::BNO055_ACCEL_DATA_Y_LSB_ADDR;
		break;
	case Axes::z:
		registerToRead = Registers::BNO055_ACCEL_DATA_Z_LSB_ADDR;
		break;
	default:
		break;
	}

	// Read the data registers
	int16_t data = i2c.read16(registerToRead);
	// Section 3.6.4.1 of datasheet for conversion from LSBs to m/s^2
	return totalAccelerationUnits ? (double)data : (double)data / 100.0;
}

// Shifts elements in column 1 into column 0 and new readings into column 1
void IMU::storeTotalAcceleration() {
	accelerationSamples[0][0] = accelerationSamples[0][1];
	accelerationSamples[0][1] = getTotalAcceleration(Axes::x);
	accelerationSamples[1][0] = accelerationSamples[1][1];
	accelerationSamples[1][1] = getTotalAcceleration(Axes::y);
	accelerationSamples[2][0] = accelerationSamples[2][1];
	accelerationSamples[2][1] = getTotalAcceleration(Axes::z);
	accelTimeSteps++;
}

double IMU::getLinearAcceleration(Axes axis) {
	// Set register we need to read depending on the axis passed in
	uint8_t registerToRead = 0;
	switch(axis) {
	case Axes::x:
		registerToRead = Registers::BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
		break;
	case Axes::y:
		registerToRead = Registers::BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR;
		break;
	case Axes::z:
		registerToRead = Registers::BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR;
		break;
	default:
		break;
	}

	// Read the data registers
	int16_t data = i2c.read16(registerToRead);
	// Section 3.6.5.6 of datasheet for conversion from LSBs to m/s^2
	return (double)data / 100.0;
}

// Shifts elements in column 1 into column 0 and new readings into column 1
void IMU::storeLinearAcceleration() {
	accelerationSamples[0][0] = accelerationSamples[0][1];
	accelerationSamples[0][1] = getLinearAcceleration(Axes::x);
	accelerationSamples[1][0] = accelerationSamples[1][1];
	accelerationSamples[1][1] = getLinearAcceleration(Axes::y);
	accelerationSamples[2][0] = accelerationSamples[2][1];
	accelerationSamples[2][1] = getLinearAcceleration(Axes::z);
	accelTimeSteps++;
}

void IMU::calculateLinearVelocity() {
	storeLinearAcceleration();
	velocity[0] += (accelerationSamples[0][0] + accelerationSamples[0][1]) / 2.0 * (ACCELERATION_TIME_STEP / 1000.0);
	velocity[1] += (accelerationSamples[1][0] + accelerationSamples[1][1]) / 2.0 * (ACCELERATION_TIME_STEP / 1000.0);
	velocity[2] += (accelerationSamples[2][0] + accelerationSamples[2][1]) / 2.0 * (ACCELERATION_TIME_STEP / 1000.0);
}

double IMU::getLinearVelocity(Axes axis) {
	switch(axis) {
	case Axes::x:
		return velocity[0];
		break;
	case Axes::y:
		return velocity[1];
		break;
	case Axes::z:
		return velocity[2];
		break;
	case Axes::xy:
		return sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
		break;
	default:
		break;
	}
	return 0.0;
}

void IMU::setMode(IMU_Mode mode) {
	i2c.write8(Registers::BNO055_OPR_MODE_ADDR, mode);
	currentMode = mode;
	// Time required to switch between operating modes (Datasheet Table 3-6)
	mode == IMU_Mode::OPR_MODE_CONFIGMODE ? vTaskDelay(pdMS_TO_TICKS(19)) : vTaskDelay(pdMS_TO_TICKS(7));
	return;
}

uint8_t IMU::getCalibStatus() {
	return (uint8_t)i2c.read8(Registers::BNO055_CALIB_STAT_ADDR);
}

uint8_t IMU::getSysStatus() {
	return (uint8_t)i2c.read8(Registers::BNO055_SYS_STAT_ADDR);
}

uint8_t IMU::getSysError() {
	return (uint8_t)i2c.read8(Registers::BNO055_SYS_ERR_ADDR);
}

