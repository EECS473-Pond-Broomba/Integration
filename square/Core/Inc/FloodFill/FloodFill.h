/*
 * FloodFill.h
 *
 *  Created on: Nov 21, 2020
 *      Author: rishgoel
 */

#ifndef INC_FLOODFILL_FLOODFILL_H_
#define INC_FLOODFILL_FLOODFILL_H_

#include <set>
#include "GPS/GPS.h"
#include "Controller/controller.h"
#include "utility"

#define STEP_SIZE 1

class FloodFill {
public:
	FloodFill(GPS* gpsIn, controller* controllerIn);
	virtual ~FloodFill();

	void update(int x_curr, int y_curr, int bear);

	void setStart(double latitude, double longitude);

private:
	std::set<std::pair<int, int> > visisted_set;
	GPS* gps;
	controller* robot;
	std::pair<double, double> startCoordinate;

	void set_adj_pos(std::pair<int, int> *curr, std::pair<int, int> *adj)
	{
		adj[0] = std::make_pair(curr->first, curr->second - STEP_SIZE);
		adj[1] = std::make_pair(curr->first + STEP_SIZE, curr->second);
		adj[2] = std::make_pair(curr->first, curr->second + STEP_SIZE);
		adj[3] = std::make_pair(curr->first - STEP_SIZE, curr->second);
	}

	int calculateWeight(std::pair<int, int> pos);
	int num_visited_adj_block(std::pair<int, int> pos);
};

#endif /* INC_FLOODFILL_FLOODFILL_H_ */
