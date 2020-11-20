/*
 * MCP3221.cpp
 *
 *  Created on: Nov 16, 2020
 *      Author: rishgoel
 */

#include <MCP3221/MCP3221.h>
#include <cmath>
#include <stdio.h>

MCP3221::MCP3221() {


}

MCP3221::~MCP3221() {
	// TODO Auto-generated destructor stub
}

void MCP3221::init(I2C_HandleTypeDef* handle, uint8_t addr, uint8_t num)
{
	i2c.init(handle, addr);

	//Initialize samples array to 0
	address = addr;
	numSamples = num;
}

unsigned int MCP3221::getRawData()
{
	return i2c.read16();
}

unsigned int MCP3221::getRawVoltage()
{
	unsigned int temp = getRawData();
	if(temp > 4000) {
		return OFFSET_VOLTS;
	}
	return round(vRef * getRawData()/(float)(0xFFF));
}

float MCP3221::getCurrent()
{
	return ((float)getRawVoltage()-OFFSET_VOLTS)/sensitivity;
}

bool MCP3221::checkCurrent()
{
	return (getCurrent() < CURRENT_LIMIT);
}


void MCP3221::setNumSamples(unsigned int count)
{
	numSamples = count;
}
