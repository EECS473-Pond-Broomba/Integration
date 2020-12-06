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
	if(!manual) {
		int16_t curr_dist = calculate_dist(x_curr, y_curr);
		char targetXStr[16];
		char targetYStr[16];
		sprintf(targetXStr, "targetX: %i", targetX);
		sprintf(targetYStr, "targetY: %i", targetY);
		lora_tx(targetXStr, 16);
		lora_tx(targetYStr, 16);
		if(curr_dist > CLOSE_ENOUGH) {
			int16_t tempBearing = calculate_bearing(x_curr, y_curr) % 360;
			while(tempBearing < 0) {
				tempBearing += 360;
			}
			int16_t bearingError = tempBearing - bear;
			while(bearingError < -180) {
				bearingError += 360;
			}
			while(bearingError > 180) {
				bearingError -= 360;
			}
			if(bearingError > WIDE_ANG) {
				setDirection(true, false);
				setSpeed(MIN_DUTY_CYCLE, MIN_DUTY_CYCLE);
				char l_pwmStr[11];
				char r_pwmStr[11];
				sprintf(l_pwmStr, "l_pwm: %i", MIN_DUTY_CYCLE);
				sprintf(r_pwmStr, "r_pwm: %i", MIN_DUTY_CYCLE);
				lora_tx(l_pwmStr, 11);
				lora_tx(r_pwmStr, 11);
			}
			else if(bearingError < -WIDE_ANG) {
				setDirection(false, true);
				setSpeed(MIN_DUTY_CYCLE, MIN_DUTY_CYCLE);
				char l_pwmStr[11];
				char r_pwmStr[11];
				sprintf(l_pwmStr, "l_pwm: %i", MIN_DUTY_CYCLE);
				sprintf(r_pwmStr, "r_pwm: %i", MIN_DUTY_CYCLE);
				lora_tx(l_pwmStr, 11);
				lora_tx(r_pwmStr, 11);
			}
			// No point turn needed
			else {
				setDirection(true, true);

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

				int16_t b_diff = (bearingError + 540)%360 -180;

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
				char l_pwmStr[10];
				char r_pwmStr[10];
				sprintf(l_pwmStr, "l_pwm: %i", l_pwm);
				sprintf(r_pwmStr, "r_pwm: %i", r_pwm);
				lora_tx(l_pwmStr, 10);
				lora_tx(r_pwmStr, 10);
			}
		}
		// If we are close enough, don't move
		else {
			setSpeed(0, 0);
		}
	}
}

void controller::updatePidPosition(int16_t x_curr, int16_t y_curr, uint16_t bear)
{
	if(!manual) {
		//First calculate distance
		int16_t dist_error = calculate_dist(x_curr, y_curr);
		if(dist_error > CLOSE_ENOUGH) {
			char targetXStr[11];
			char targetYStr[11];
			sprintf(targetXStr, "targetX: %i", targetX);
			sprintf(targetYStr, "targetY: %i", targetY);
			lora_tx(targetXStr, 11);
			lora_tx(targetYStr, 11);
			setSpeed(0, 0);
			//If the bearing error is very large then we do a fixed turn
			int16_t tempBearing = calculate_bearing(x_curr, y_curr) % 360;
			while(tempBearing < 0) {
				tempBearing += 360;
			}
			int16_t bearingError = tempBearing - bear;
			while(bearingError < -180) {
				bearingError += 360;
			}
			while(bearingError > 180) {
				bearingError -= 360;
			}
			if(bearingError > WIDE_ANG) {
				setDirection(true, false);
				setSpeed(MIN_DUTY_CYCLE, MIN_DUTY_CYCLE);
				char l_pwmStr[11];
				char r_pwmStr[11];
				sprintf(l_pwmStr, "l_pwm: %i", MIN_DUTY_CYCLE);
				sprintf(r_pwmStr, "r_pwm: %i", MIN_DUTY_CYCLE);
				lora_tx(l_pwmStr, 11);
				lora_tx(r_pwmStr, 11);
			}
			else if(bearingError < -WIDE_ANG) {
				setDirection(false, true);
				setSpeed(MIN_DUTY_CYCLE, MIN_DUTY_CYCLE);
				char l_pwmStr[11];
				char r_pwmStr[11];
				sprintf(l_pwmStr, "l_pwm: %i", MIN_DUTY_CYCLE);
				sprintf(r_pwmStr, "r_pwm: %i", MIN_DUTY_CYCLE);
				lora_tx(l_pwmStr, 11);
				lora_tx(r_pwmStr, 11);
			}
			//Use PID to go to target
			else {
				setDirection(true, true);


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
				char l_pwmStr[11];
				char r_pwmStr[11];
				sprintf(l_pwmStr, "l_pwm: %i", l_pwm);
				sprintf(r_pwmStr, "r_pwm: %i", r_pwm);
				lora_tx(l_pwmStr, 11);
				lora_tx(r_pwmStr, 11);
			}
			old_bearing = bearingError;
		}
		else {
			setSpeed(0, 0);
			char l_pwmStr[11];
			char r_pwmStr[11];
			sprintf(l_pwmStr, "l_pwm: %i", 0);
			sprintf(r_pwmStr, "r_pwm: %i", 0);
			lora_tx(l_pwmStr, 11);
			lora_tx(r_pwmStr, 11);
		}
	}
}

int16_t controller::calculate_dist(int16_t x, int16_t y)
{
    return sqrt(pow(x - targetX, 2) + pow(y - targetY, 2));
}

int16_t controller::calculate_bearing(int16_t x, int16_t y)
{
	double tempAtan = atan2(targetY - y, targetX - x);
    return 90 - tempAtan * (180.0/3.141592653589793238463);
}

void controller::setMotorSpeed(int pwm1, int pwm2) {
	setSpeed(pwm1, pwm2);
}

void controller::setMotorDirection(bool left_forward, bool right_forward) {
	setDirection(left_forward, right_forward);
}
