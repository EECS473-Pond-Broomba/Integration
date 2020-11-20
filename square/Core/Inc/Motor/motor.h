#ifndef INC_MOTOR_MOTOR_H_
#define INC_MOTOR_MOTOR_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define MIN 78
#define MEDIUM 85
#define MAX 150

//TODO :::::::: Figure out speed ratios for turning

//pwm output timers for each of the pwm for each motor
TIM_HandleTypeDef * htiml1;
//TIM_HandleTypeDef * htiml2;
TIM_HandleTypeDef * htimr1;
//TIM_HandleTypeDef * htimr2;
int pwml1;
//int pwml2;
int pwmr1;
//int pwmr2;

void motorInit(TIM_HandleTypeDef * htim1, TIM_HandleTypeDef * htim2){
	htiml1 = htim1;
	htimr1 = htim2;
}

//void motorInit(TIM_HandleTypeDef * htim1, TIM_HandleTypeDef * htim2,
//		TIM_HandleTypeDef * htim3, TIM_HandleTypeDef * htim4){
//	htiml1 = htim1;
//	htiml2 = htim2;
//	htimr1 = htim3;
//	htimr2 = htim4;
//}

/*
* Set the motor to a new speed
* takes in the pwm values for left and right pwm for each motor
*/
void setSpeed(int pwm1, int pwm2){
	__HAL_TIM_SetCompare(htiml1, TIM_CHANNEL_3, pwm1);
	pwml1 = pwm1;
	__HAL_TIM_SetCompare(htimr1, TIM_CHANNEL_2, pwm2);
	pwmr1 = pwm2;
//	HAL_Delay(100);
}

/*
* Set the motor to a new speed
* takes in the pwm values for left and right pwm for each motor
*/
//void setSpeed(int pwm1, int pwm2, int pwm3, int pwm4){
//	__HAL_TIM_SetCompare(htiml1, TIM_CHANNEL_1, pwm1);
//	pwml1 = pwm1;
//	__HAL_TIM_SetCompare(htiml2, TIM_CHANNEL_3, pwm2);
//	pwml2 = pwm2;
//	__HAL_TIM_SetCompare(htimr1, TIM_CHANNEL_3, pwm3);
//	pwmr1 = pwm3;
//	__HAL_TIM_SetCompare(htimr2, TIM_CHANNEL_3, pwm4);
//	pwmr2 = pwm4;
//	HAL_Delay(100);
//}

/*
* Changes the direction the motors are driving in
* direction: A value that distinguishes which exact direction the motors should go in:
  left forward(0), left back(1), right forward(4), right back (5), forward(2), backwards(3)
* degreeTurn: The amount of degrees to turn the robot
*/
void changeDirection(int degreeTurn){
	return;
}

/*
Causes the motors to move forward or backward based on current GPIO Pin State
*/
void setDirection(bool left_forward, bool right_forward){
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, (GPIO_PinState)!left_forward);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, (GPIO_PinState)right_forward);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, (GPIO_PinState)left_forward);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, (GPIO_PinState)!right_forward);
//	if(forward) {
//		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
//	}
//	else {
//		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
//	}
}

/*
Turns motors off by setting the pwm equal to 0
*/
void stopMotors(){
	__HAL_TIM_SetCompare(htiml1, TIM_CHANNEL_3, 0);
//	__HAL_TIM_SetCompare(htiml2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(htimr1, TIM_CHANNEL_2, 0);
//	__HAL_TIM_SetCompare(htimr2, TIM_CHANNEL_3, 0);
	pwml1 = 0;
//	pwml2 = 0;
	pwmr1 = 0;
//	pwmr2 = 0;
//	HAL_Delay(100);
}

#endif /* INC_MOTOR_MOTOR_H_ */
