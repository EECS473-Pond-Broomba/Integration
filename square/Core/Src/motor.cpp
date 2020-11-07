#include "Motor/motor.h"

void motorInit(TIM_HandleTypeDef * htim1, TIM_HandleTypeDef * htim2,
		TIM_HandleTypeDef * htim3, TIM_HandleTypeDef * htim4){
	htiml1 = htim1;
	htiml2 = htim2;
	htimr1 = htim3;
	htimr2 = htim4;
}

void setSpeed(int pwm1, int pwm2, int pwm3, int pwm4){
	__HAL_TIM_SetCompare(htiml1, TIM_CHANNEL_1, pwm1);
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
	__HAL_TIM_SetCompare(htiml1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(htiml2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(htimr1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(htimr2, TIM_CHANNEL_1, 0);
	pwml1 = 0;
	pwml2 = 0;
	pwmr1 = 0;
	pwmr2 = 0;
	HAL_Delay(100);
}
