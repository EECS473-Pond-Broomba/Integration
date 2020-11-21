#include "controller.h"
#include "cmath"
#include "Motor/motor.h"

controller::controller()
{
}

controller::~controller()
{
}

void controller::init()
{
	motorInit(&htim2, &htim3);
}

void controller::setTarget(int16_t x, int16_t y) 
{
    targetX = x;
    targetY = y;
}

void controller::updateLinearPosition(int16_t x_curr, int16_t y_curr, uint16_t bear)
{
    uint16_t bearingError = calculate_bearing(x_curr, y_curr);
    if(bearingError > WIDE_ANG) {
        setDirection(false, true);
        setSpeed(MIN_DUTY_CYCLE, MIN_DUTY_CYCLE);
    }
    else if(bearingError < -WIDE_ANG) {
        setDirection(true, false);
        setSpeed(MIN_DUTY_CYCLE, MIN_DUTY_CYCLE);
    }
    // No point turn needed
    else {
        setDirection(true, true);
        int16_t curr_dist = calculate_dist(x_curr, y_curr);
        int l_pwm, r_pwm;

        if(curr_dist > FAR_DIST)
        {
            r_pwm = l_pwm = MAX_DUTY_CYCLE - ROT_DUTY_CYCLE;
        }
        else if(curr_dist < CLOSE_DIST)
        {
            r_pwm = l_pwm = MIN_DUTY_CYCLE + ROT_DUTY_CYCLE;
        }
        else
        {
            r_pwm = l_pwm = (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE - 2*ROT_DUTY_CYCLE) / (FAR_DIST - CLOSE_DIST) * curr_dist + (MIN_DUTY_CYCLE + ROT_DUTY_CYCLE - CLOSE_DIST*((MAX_DUTY_CYCLE - MIN_DUTY_CYCLE - 2*ROT_DUTY_CYCLE) / (FAR_DIST - CLOSE_DIST)));
        }
        
        uint16_t b_diff = (bearingError + 540)%360 -180;

        unsigned int rotPWM = abs(bearingError) * ROT_DUTY_CYCLE/WIDE_ANG;

        if(b_diff > 0)
        {
            //Clockwise rotation
            l_pwm += rotPWM;
            r_pwm -= rotPWM;
        }
        else
        {
            l_pwm -= rotPWM;
            r_pwm += rotPWM;
        }
        
        setSpeed(l_pwm, r_pwm);
    }
}

void controller::updatePidPosition(int16_t x_curr, int16_t y_curr, uint16_t bear)
{
	setSpeed(0, 0);
	//If the bearing error is very large then we do a fixed turn
	uint16_t bearingError = calculate_bearing(x_curr, y_curr);
	if(bearingError > WIDE_ANG) {
		setDirection(false, true);
		setSpeed(MIN_DUTY_CYCLE, MIN_DUTY_CYCLE);
	}
	else if(bearingError < -WIDE_ANG) {
		setDirection(true, false);
		setSpeed(MIN_DUTY_CYCLE, MIN_DUTY_CYCLE);
	}
	//Use PID to go to target
	else {
		setDirection(true, true);
		//First calculate distance
		int16_t dist_error = calculate_dist(x_curr, y_curr);

		int l_pwm, r_pwm;

		//Get the Propotional value
		pid_p_dist = KP_Dist * dist_error;

		//Get the integral value but only if we are close to the target
		if(dist_error < 5)
		{
			pid_i_dist += KI_Dist*dist_error;
		}

		pid_d_dist = KD_Dist*((dist_error - old_distance)/PID_UPDATE_TIME);

		old_distance = dist_error;

		int pid_dist_out = pid_p_dist + pid_i_dist + pid_d_dist;

		if(pid_dist_out > MAX_DUTY_CYCLE - ROT_DUTY_CYCLE)
		{
			pid_dist_out = MAX_DUTY_CYCLE - ROT_DUTY_CYCLE;
		}
		else if(pid_dist_out < MIN_DUTY_CYCLE + ROT_DUTY_CYCLE)
		{
			pid_dist_out = MIN_DUTY_CYCLE + ROT_DUTY_CYCLE;
		}

		//Get the Propotional value
		pid_p_bear = KP_Bear * bearingError;

		//Get the integral value but only if we are close to the target
		if(dist_error < 5)
		{
			pid_i_bear += KI_Bear*bearingError;
		}

		pid_d_bear = KD_Bear*((bearingError - old_bearing)/PID_UPDATE_TIME);

		int pid_bear_out = pid_p_bear + pid_i_bear + pid_d_bear;

		if(pid_dist_out > ROT_DUTY_CYCLE)
		{
			pid_dist_out = ROT_DUTY_CYCLE;
		}
		else if(pid_dist_out < -ROT_DUTY_CYCLE)
		{
			pid_dist_out =  -ROT_DUTY_CYCLE;
		}

		l_pwm = pid_dist_out + pid_bear_out;
		r_pwm = pid_dist_out - pid_bear_out;

		setSpeed(l_pwm, r_pwm);

	}
	old_bearing = bearingError;


}

int16_t controller::calculate_dist(int16_t x, int16_t y)
{
    return sqrt(pow(x - targetX, 2) + pow(y - targetY, 2));
}

uint16_t controller::calculate_bearing(int16_t x, int16_t y)
{
     return atan2(y - targetY, x - targetX) * (180.0/3.141592653589793238463);
}
