/*
 * MCP3221.cpp
 *
 *  Created on: Nov 16, 2020
 *      Author: rishgoel
 */

#include <MCP3221/MCP3221.h>
#include <cmath>

MCP3221::MCP3221() {


}

MCP3221::~MCP3221() {
	// TODO Auto-generated destructor stub
}

void MCP3221::init(I2C_HandleTypeDef* handle, uint8_t addr, uint8_t num)
{
	i2c.init(handle, addr);

	//Initialize samples array to 0
	memset(samples, 0, sizeof(samples));
	address = addr;
	numSamples = num;
}

unsigned int MCP3221::getRawData()
{
	return i2c.read16();
}

unsigned int MCP3221::getRawVoltage()
{
	return round(vRef * getRawData()/(0xFFFF));
}

float MCP3221::getCurrent()
{
	return (getRawVoltage()-OFFSET_VOLTS)/sensitivity;
}


void MCP3221::setNumSamples(unsigned int count)
{
	numSamples = count;
}
