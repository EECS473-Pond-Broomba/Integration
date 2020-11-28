/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Controller/controller.h"
#include "main.h"
#include "spi.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "GPS/GPS.h"
#include "IMU/IMU.h"
#include "SF_Nav/SFNav.h"
#include "LoRa.h"
//#include "uart_printf.h"
#include "semphr.h"
#include "FreeRTOS.h"
#include <vector>
#include "MCP3221/MCP3221.h"
#include "tim.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim5;
//TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

xSemaphoreHandle gps_sem;
MCP3221 bat_curr;
SF_Nav kf;
controller cont;
state_var boatState;		// Stores current state of the boat
std::vector<state_var> targetStates;	// States that we want boat to be at
std::vector<state_var> testStates;
int targetCounter = 0;
int stateCounter = 0;
int pwm1 = 0;
int pwm2 = 0;

int pwm = 0;
long input_capture1 = 0;
long input_capture2 = 0;
long Difference = 0;
uint8_t Is_First_Captured = 0;
uint8_t buf[20];
uint32_t value[2];
loraClass radio;
byte getstr[21];


//#define PIDPERIOD 500
#define MOTORMAXDIFF 20
#define MOTORMIN 150 + MOTORMAXDIFF
#define MOTORMAX 250 - MOTORMAXDIFF
//#define DISTDEADZONE 0.3	// If robot is within this distance of target, motors dont move
//#define DISTSATURATE 5		// If robot is further than this distance of target, motors move at maximum speed
#define TESTDELAY 0

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_I2C1_Init(void);
void MX_I2C3_Init(void);
void MX_ADC1_Init(void);
//void MX_TIM5_Init(void);
//void MX_TIM9_Init(void);
void MX_USART6_UART_Init(void);
void MX_FREERTOS_Init(void);
double distanceBetweenStates(state_var &state1, state_var &state2) {
	return sqrt(pow(state1.x - state2.x, 2) + pow(state1.y - state2.y, 2));
}
double bearingBetweenStates(state_var &state1, state_var &state2) {
	return atan2(state1.y - state2.y, state1.x - state2.x) * (180.0/3.141592653589793238463);
}

// ---------------------------- Our Tasks --------------------------------------
float timedifference_msec(struct timeval t0, struct timeval t1){
  return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
}

//void ServoOpen() {
//	for(pwm=4;pwm<23;pwm++) {
//	  __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, pwm);
//	  sprintf((char*)buf, "%u pwm\r\n", (unsigned int)pwm);
//	  HAL_UART_Transmit(&huart6, buf, 20, HAL_MAX_DELAY);
//	  HAL_Delay(100);
//	}
//	HAL_Delay(200);
//}
//
//void ServoClose() {
//  for(pwm=22;pwm>=4;pwm--) {
//	  __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, pwm);
//	  sprintf((char*)buf, "%u pwm\r\n", (unsigned int)pwm);
//	  HAL_UART_Transmit(&huart6, buf, 20, HAL_MAX_DELAY);
//	  HAL_Delay(100);
//  }
//  HAL_Delay(200);
//}

void Sensors(void* arg) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(10000);
	xLastWakeTime = xTaskGetTickCount();
	uint32_t raw;
	int tdsValue;
	char bufNew[20];

  while(1) {
  		vTaskDelayUntil(&xLastWakeTime, xPeriod);
  		 ///////////////TDS/////////////////////////////////
  		  // Test: Set GPIO pin high
  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  		  // Get ADC value
  		  HAL_ADC_Start(&hadc1);
  		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  		  raw = HAL_ADC_GetValue(&hadc1);
  		  tdsValue = (raw * 3.3) / 4096.0;



  		  //float compensationCoefficient=1.0;//+0.02*(temperature-25.0);
  		  float compensationVoltage= tdsValue / 1; //compensationCoefficient;
  		  tdsValue = (133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;

  		  // Test: Set GPIO pin low
  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  		  // Pretend we have to do something else for a while
//  		  HAL_Delay(100);
  		  vTaskDelay(pdMS_TO_TICKS(100));
//  		  ServoOpen();
//  		  ServoClose();

  }
}

// Moves to a target state using PID like a noob
void MovePID(void* arg) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(PID_UPDATE_TIME);
	xLastWakeTime = xTaskGetTickCount();
	cont.init();
	kf.init(&huart6, &hi2c1, KALMAN_REFRESH_TIME);
	cont.setTarget(0, 5);
	while(1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
//		kf.update();
//		boatState = kf.get_state();
//		// Only start controllers and motors when GPS data is valid
//		if(kf.get_valid()) {
//			cont.updateLinearPosition(boatState.x, boatState.y, boatState.b);
//		}

		if(stateCounter < TESTDELAY) {
			stateCounter++;
			cont.setMotorSpeed(0, 0);
		}
		// Drive out a bit to the pond before doing its ting
		else if(stateCounter >= TESTDELAY && stateCounter < (TESTDELAY+10)) {
			stateCounter++;
			cont.setMotorDirection(true, true);
			cont.setMotorSpeed(500, 530);
		}
		else if(stateCounter >= (TESTDELAY+10) && stateCounter < (TESTDELAY+20)) {
			boatState = testStates[stateCounter - TESTDELAY - 10];
			stateCounter++;
			cont.updatePidPosition(boatState.x, boatState.y, boatState.b);
		}
		else {
			cont.setMotorSpeed(0, 0);
		}

	}
}

void MoveLinear(void* arg) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(PID_UPDATE_TIME);
	xLastWakeTime = xTaskGetTickCount();
	cont.init();
	kf.init(&huart6, &hi2c1, KALMAN_REFRESH_TIME);
//	vTaskDelay(pdMS_TO_TICKS(10000));
	cont.setTarget(0, 0);
	while(1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
//		kf.update();
//		boatState = kf.get_state();
		// Only start controllers and motors when GPS data is valid
//		if(kf.get_valid()) {
//			cont.updateLinearPosition(boatState.x, boatState.y, boatState.b);
//		}

		if(stateCounter < TESTDELAY) {
			stateCounter++;
			cont.setMotorSpeed(0, 0);
		}
		// Drive out a bit to the pond before doing its ting
		else if(stateCounter >= TESTDELAY && stateCounter < (TESTDELAY+10)) {
			stateCounter++;
			cont.setMotorDirection(true, true);
			cont.setMotorSpeed(500, 560);
		}
		else if(stateCounter >= (TESTDELAY+10) && stateCounter < (TESTDELAY+20)) {
			boatState = testStates[stateCounter - TESTDELAY - 10];
			stateCounter++;
			cont.updateLinearPosition(boatState.x, boatState.y, boatState.b);
		}
		else {
			cont.setMotorSpeed(0, 0);
		}

	}
}

// Calls updates on the Kalman Filter and initializes the GPS and IMU
void UpdateKF(void* arg) {
	gps_sem = xSemaphoreCreateBinary();
//	kf.gps.init(&huart1);
//	gps.init(&huart1);
	kf.init(&huart6, &hi2c1, KALMAN_REFRESH_TIME);
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(KALMAN_REFRESH_TIME * 1000);
	xLastWakeTime = xTaskGetTickCount();
	vTaskDelay(1000);
	while(1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
//		if(kf.gps.update()) {
//			location loc = kf.gps.getPosition();
//			uart_printf("Latitude %f\r\nLongitude %f\r\n", loc.latitude, loc.longitude);
//		}
		kf.update();
		boatState = kf.get_state();
		int dummy = 0;
//		uart_printf("some\r\n");
//		vTaskDelay(1000);
	}
}

//THIS SHOULD BE HIGHEST PRIORITY TASK
void checkBattery(void*)
{
	bat_curr.init(&hi2c3, 0x4f, 1);
	HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
	//Wait while bat_curr is less than 0.1 A
//	while(bat_curr.getCurrent() < 0.1);

	//Now turn on the relay pin
	HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(5));

	while(1)
	{
//		float dummyV = bat_curr.getRawVoltage();
//		float dummyC = bat_curr.getCurrent();
		//If current is ever greater than the limit than switch relay off
		if(!bat_curr.checkCurrent())
		{
			if(!bat_curr.checkCurrent()) {
				//Disconnect relay and wait forever
				HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
				//Just wait forever here
				while(1);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void TestMotors(void* arg) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();
	cont.init();
	cont.setMotorDirection(true, true);	// Go forward
	cont.setMotorSpeed(400, 400);
	while(1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

// Task that sends heartbeat
void Heartbeat(void* arg) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	radio.Modulation = LORA;
	radio.COB            = RFM95;
	radio.Frequency      = 434000;
	radio.OutputPower    = 17;             //17dBm OutputPower
	radio.PreambleLength = 16;             //16Byte preamble
	radio.FixedPktLength = false;          //explicit header mode for LoRa
	radio.PayloadLength  = 21;
	radio.CrcDisable     = true;	// True for TX and False for RX

	radio.SFSel          = SF9;
	radio.BWSel          = BW125K;
	radio.CRSel          = CR4_5;

	radio.vInitialize();
	radio.vGoStandby();
	byte heartbeat[7] = {'b','r', 'o', 'o', 'm', 'b', 'a'};
	while(1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		radio.bSendMessage(heartbeat, 7);
	}
}

// ---------------------------- Our Tasks --------------------------------------

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
//  MX_TIM5_Init();
//  MX_TIM9_Init();
//  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  //NEEDS TO CHANGE//
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
//  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);
  //motorInit(&htim2, &htim3);

  // -------------------- Artificial states for moving straight -----------------
  /*
  // Straight South - Correct
  for(int i = 0; i < 10; ++i) {
  	  state_var temp = { 	.x = 0,
  	    					.y = 5-0.5*(double)i,
  	    					.b = 180,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
*/
/*
  // Straight West -- Did a right point turn instead
  for(int i = 0; i < 10; ++i) {
  	  state_var temp = { 	.x = 5-0.5*(double)i,
  	    					.y = 0,
  	    					.b = 270,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
*/
/*
  // Straight North -- Curved right instead
  for(int i = 0; i < 10; ++i) {
  	  state_var temp = { 	.x = 0,
  	    					.y = -5+0.5*(double)i,
  	    					.b = 0,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
*/
/*
  // Straight East - Correct
  for(int i = 0; i < 10; ++i) {
  	  state_var temp = { 	.x = -5+0.5*(double)i,
  	    					.y = 0,
  	    					.b = 90,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
*/

  // -------------------- Artificial states for left point turn and then straight -----------------
/*
  // Target to the North -- Crrect util end where it did a short right point turn
  for(int i = 0; i < 3; ++i) {
  	  state_var temp = { 	.x = 0,
  							.y = -5,
  							.b = 90-(double)i*31,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
  for(int i = 0; i < 7; ++i) {
	  state_var temp = { 	.x = 0,
							.y = -5+(double)i*0.7,
							.b = 0,
							.vX = 0,
							.vY = 0,
							.vB = 0};
	  testStates.push_back(temp);
  }
*/
/*
  // Target to the East -- Correct
  for(int i = 0; i < 3; ++i) {
  	  state_var temp = { 	.x = -5,
  							.y = 0,
  							.b = 180-(double)i*31,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
  for(int i = 0; i < 7; ++i) {
	  state_var temp = { 	.x = -5+(double)i*0.7,
							.y = 0,
							.b = 90,
							.vX = 0,
							.vY = 0,
							.vB = 0};
	  testStates.push_back(temp);
  }
*/
/*
  // Target to the South -- Small left point turn at the end
  for(int i = 0; i < 3; ++i) {
  	  state_var temp = { 	.x = 0,
  							.y = 5,
  							.b = 270-(double)i*31,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
  for(int i = 0; i < 7; ++i) {
	  state_var temp = { 	.x = 0,
							.y = 5-(double)i*0.7,
							.b = 180,
							.vX = 0,
							.vY = 0,
							.vB = 0};
	  testStates.push_back(temp);
  }
*/
/*
  // Target to the West - Left point turn the entire 10 seconds
  for(int i = 0; i < 3; ++i) {
  	  state_var temp = { 	.x = 5,
  							.y = 0,
  							.b = 359-(double)i*31,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
  for(int i = 0; i < 7; ++i) {
	  state_var temp = { 	.x = 5-(double)i*0.7,
							.y = 0,
							.b = 270,
							.vX = 0,
							.vY = 0,
							.vB = 0};
	  testStates.push_back(temp);
  }
*/

  // -------------------- Artificial states for right point turn and then straight -----------------
  /*
  // Target to the North
  for(int i = 0; i < 3; ++i) {
  	  state_var temp = { 	.x = 0,
  							.y = -5,
  							.b = 270+(double)i*29,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
  for(int i = 0; i < 7; ++i) {
	  state_var temp = { 	.x = 0,
							.y = -5+(double)i*0.7,
							.b = 0,
							.vX = 0,
							.vY = 0,
							.vB = 0};
	  testStates.push_back(temp);
  }
  */
/*
  // Target to the East
  for(int i = 0; i < 3; ++i) {
  	  state_var temp = { 	.x = -5,
  							.y = 0,
  							.b = 0+(double)i*31,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
  for(int i = 0; i < 7; ++i) {
	  state_var temp = { 	.x = -5+(double)i*0.7,
							.y = 0,
							.b = 90,
							.vX = 0,
							.vY = 0,
							.vB = 0};
	  testStates.push_back(temp);
  }
*/
/*
  // Target to the South
  for(int i = 0; i < 3; ++i) {
  	  state_var temp = { 	.x = 0,
  							.y = 5,
  							.b = 90+(double)i*31,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
  for(int i = 0; i < 7; ++i) {
	  state_var temp = { 	.x = 0,
							.y = 5-(double)i*0.7,
							.b = 180,
							.vX = 0,
							.vY = 0,
							.vB = 0};
	  testStates.push_back(temp);
  }
*/
/*
  // Target to the West
  for(int i = 0; i < 3; ++i) {
  	  state_var temp = { 	.x = 5,
  							.y = 0,
  							.b = 180+(double)i*31,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
  }
  for(int i = 0; i < 7; ++i) {
	  state_var temp = { 	.x = 5-(double)i*0.7,
							.y = 0,
							.b = 270,
							.vX = 0,
							.vY = 0,
							.vB = 0};
	  testStates.push_back(temp);
  }
*/


/*
  // ------------- Artificial boat states for moving in slight right curve towards south --------------------
  for(int i = 0; i < 10; ++i) {
  	  state_var temp = { 	.x = 0.5*(double)i,
  	    					.y = 1-0.1*(double)i,
  	    					.b = 90+i,
  							.vX = 0,
  							.vY = 0,
  							.vB = 0};
  	  testStates.push_back(temp);
   }
   */
/*
  // ------------- Artificial boat states for moving in slight left curve towards north --------------------
  for(int i = 0; i < 10; ++i) {
	  state_var temp = { 	.x = 0.5*(double)i,
    	    				.y = -1 + 0.1*(double)i,
    	    				.b = 90-i,
    						.vX = 0,
    						.vY = 0,
    						.vB = 0};
	  testStates.push_back(temp);
   }
   */
//  xTaskCreate(UpdateKF, "kalman", 2048, NULL, 1, NULL);
//  xTaskCreate(MovePID, "noob", 1024, NULL, 1, NULL);
//  xTaskCreate(MoveLinear, "chad", 2048, NULL, 1, NULL);
//  xTaskCreate(TestMotors, "testMotors", 1024, NULL, 1, NULL);
//  xTaskCreate(Sensors, "sensors", 128, NULL, 1, NULL);
  xTaskCreate(Heartbeat, "heartbeat", 128, NULL, 0, NULL);
  xTaskCreate(checkBattery, "currentSensor", 256, NULL, 3, NULL);	// MUST BE HIGHEST PRIORITY
  vTaskStartScheduler();
//
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */

void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
//void MX_TIM5_Init(void)
//{
//
//  /* USER CODE BEGIN TIM5_Init 0 */
//
//  /* USER CODE END TIM5_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM5_Init 1 */
//
//  /* USER CODE END TIM5_Init 1 */
//  htim5.Instance = TIM5;
//  htim5.Init.Prescaler = 4;
//  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim5.Init.Period = 690;
//  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM5_Init 2 */
//
//  /* USER CODE END TIM5_Init 2 */
//  HAL_TIM_MspPostInit(&htim5);
//
//}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
//void MX_TIM9_Init(void)
//{
//
//  /* USER CODE BEGIN TIM9_Init 0 */
//
//  /* USER CODE END TIM9_Init 0 */
//
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM9_Init 1 */
//
//  /* USER CODE END TIM9_Init 1 */
//  htim9.Instance = TIM9;
//  htim9.Init.Prescaler = 9333;
//  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim9.Init.Period = 180;
//  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM9_Init 2 */
//
//  /* USER CODE END TIM9_Init 2 */
//  HAL_TIM_MspPostInit(&htim9);
//
//}

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */

//void MX_USART6_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART6_Init 0 */
//
//  /* USER CODE END USART6_Init 0 */
//
//  /* USER CODE BEGIN USART6_Init 1 */
//
//  /* USER CODE END USART6_Init 1 */
//  huart6.Instance = USART6;
//  huart6.Init.BaudRate = 115200;
//  huart6.Init.WordLength = UART_WORDLENGTH_8B;
//  huart6.Init.StopBits = UART_STOPBITS_1;
//  huart6.Init.Parity = UART_PARITY_NONE;
//  huart6.Init.Mode = UART_MODE_TX_RX;
//  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart6) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART6_Init 2 */
//
//  /* USER CODE END USART6_Init 2 */
//
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
