/*
 * FloodFill.cpp
 *
 *  Created on: Nov 21, 2020
 *      Author: rishgoel
 */

#include <FloodFill/FloodFill.h>

FloodFill::FloodFill() {
	// TODO Auto-generated constructor stub

}

FloodFill::~FloodFill() {
	// TODO Auto-generated destructor stub
}

void FloodFill::update(int x_curr, int y_curr, int bear)
{
	std::pair<int, int> adjacent_pos[4];
	std::pair<int, int> curr_pos = std::make_pair(x_curr, y_curr);
	int geofence_state;
	// geofence_status = gps->checkGeofence();

	//Check if we are currently in the geofence
	//TODO: Update this check
	if(geofence_state != 0)
	{
		set_adj_pos(&curr_pos, adjacent_pos);

		int maxWeight = -1, max_idx = 0;
		for(int i = 0; i < 4; ++i) {

			int weight = calculateWeight(adjacent_pos[i]);
			if(weight > maxWeight)
			{
			  maxWeight = weight;
			  max_idx = i;
			}
		}

		if(maxWeight > 0)
		{
			std::pair<int, int> target_pos = adjacent_pos[max_idx];

			robot->setTarget(target_pos.first, target_pos.second);

		}
		else
		{
			//TODO:Change this to the geofence center position
			robot->setTarget(x_curr, y_curr);
		}
	}
	//We are not in the geofence so move to the center
	else
	{
		//TODO:Change this to the geofence center position
		robot->setTarget(x_curr, y_curr);
	}

	robot->update(x_curr, y_curr, bear);
}

int FloodFill::calculateWeight(std::pair<int, int> pos)
{
	int score = 0;
	int gfStatus = 0;// TODO: CHange this to geofence

	bool visited = (visisted_set.find(pos) != visisted_set.end());
	if(visited|| gfStatus == 0)
	{
		return -1;
	}
	if(gfStatus == 1)
	{
		score++;
	}

	float dist_edge = 0;//TODO: change this to distance from edge

	if(dist_edge <= 10)
	{
		score += 5;
	}

	score += num_visited_adj_block(pos);
	return score;
}

int FloodFill::num_visited_adj_block(std::pair<int, int> pos)
{
	int val = 0;
	std::pair<int, int> temp = std::make_pair(pos.first - STEP_SIZE, pos.second);
	if(visisted_set.find(temp) != visisted_set.end())
	{
		val++;
	}

	temp = std::make_pair(pos.first + STEP_SIZE, pos.second);
	if(visisted_set.find(temp) != visisted_set.end())
	{
		val++;
	}

	temp = std::make_pair(pos.first, pos.second - STEP_SIZE);
	if(visisted_set.find(temp) != visisted_set.end())
	{
		val++;
	}

	temp = std::make_pair(pos.first, pos.second + STEP_SIZE);
	if(visisted_set.find(temp) != visisted_set.end())
	{
		val++;
	}

	return val;
}

