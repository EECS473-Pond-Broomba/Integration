/*
 * MCP3221.h
 *
 *  Created on: Nov 16, 2020
 *      Author: rishgoel
 */

#ifndef INC_MCP3221_MCP3221_H_
#define INC_MCP3221_MCP3221_H_

#include "I2C/I2C.h"

const uint8_t MIN_NUM_SAMPLES = 1;
const uint8_t MAX_NUM_SAMPLES = 20;
const uint8_t DEFAULT_NUM_SAMPLES = 10;

const int OFFSET_VOLTS = 2500;
const int sensitivity = 66;

class MCP3221 {
public:
	MCP3221();
	virtual ~MCP3221();

	void init(I2C_HandleTypeDef* handle, uint8_t addr, uint8_t num);

	unsigned int getData();
	unsigned int getRawVoltage();
	float getCurrent();

	void setVref(unsigned int v);
	void setNumSamples(unsigned int count);


private:
	I2C i2c;
	uint8_t address, numSamples;
	unsigned int samples[MAX_NUM_SAMPLES];

	unsigned int getRawData();


};

#endif /* INC_MCP3221_MCP3221_H_ */
