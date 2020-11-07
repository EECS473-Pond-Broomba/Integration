#ifndef INC_MOTOR_MOTOR_H_
#define INC_MOTOR_MOTOR_H_

#include "stm32f4xx_hal.h"

#define MIN 20
#define MEDIUM 85
#define MAX 150

//TODO :::::::: Figure out speed ratios for turning

//pwm output timers for each of the pwm for each motor
TIM_HandleTypeDef * htiml1;
TIM_HandleTypeDef * htiml2;
TIM_HandleTypeDef * htimr1;
TIM_HandleTypeDef * htimr2;
int pwml1;
int pwml2;
int pwmr1;
int pwmr2;

/*
 * This function sets the timers to the initialized timers from the .ioc
 */
void motorInit(TIM_HandleTypeDef * htim1, TIM_HandleTypeDef * htim2,
		TIM_HandleTypeDef * htim3, TIM_HandleTypeDef * htim4);

/*
* Set the motor to a new speed
* takes in the pwm values for left and right pwm for each motor
*/
void setSpeed(int pwm1, int pwm2, int pwm3, int pwm4);

/*
* Changes the direction the motors are driving in
* direction: A value that distinguishes which exact direction the motors should go in:
  left forward(0), left back(1), right forward(4), right back (5), forward(2), backwards(3)
* degreeTurn: The amount of degrees to turn the robot
*/
void changeDirection(int direction, int degreeTurn);

/*
Causes the motors to move forward or backward based on current GPIO Pin State
*/
void oppositeDirection();

/*
Turns motors off by setting the pwm equal to 0
*/
void stopMotors();

#endif /* INC_MOTOR_MOTOR_H_ */
