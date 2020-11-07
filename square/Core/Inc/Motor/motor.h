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

void motorInit(TIM_HandleTypeDef * htim1, TIM_HandleTypeDef * htim2,
		TIM_HandleTypeDef * htim3, TIM_HandleTypeDef * htim4){
	htiml1 = htim1;
	htiml2 = htim2;
	htimr1 = htim3;
	htimr2 = htim4;
}

void setSpeed(int pwm1, int pwm2, int pwm3, int pwm4){
	__HAL_TIM_SetCompare(htiml1, TIM_CHANNEL_3, pwm1);
	pwml1 = pwm1;
	__HAL_TIM_SetCompare(htiml2, TIM_CHANNEL_1, pwm2);
	pwml2 = pwm2;
	__HAL_TIM_SetCompare(htimr1, TIM_CHANNEL_1, pwm3);
	pwmr1 = pwm3;
	__HAL_TIM_SetCompare(htimr2, TIM_CHANNEL_1, pwm4);
	pwmr2 = pwm4;
	HAL_Delay(100);
}

void changeDirection(int direction, int degreeTurn){
	return;
}

void oppositeDirection(){
	int temp1;
	int temp2;

	temp1 = pwmr1;
	temp2 = pwml1;

	pwmr1 = pwmr2;
	pwml1 = pwml2;
	pwmr2 = temp1;
	pwml2 = temp2;
}

void stopMotors(){
	__HAL_TIM_SetCompare(htiml1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(htiml2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(htimr1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(htimr2, TIM_CHANNEL_1, 0);
	pwml1 = 0;
	pwml2 = 0;
	pwmr1 = 0;
	pwmr2 = 0;
	HAL_Delay(100);
}

#endif /* INC_MOTOR_MOTOR_H_ */
