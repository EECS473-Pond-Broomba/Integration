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
#include "Motor/motor.h"
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "GPS/GPS.h"
#include "IMU/IMU.h"
#include "SF_Nav/SFNav.h"
#include "uart_printf.h"
#include "semphr.h"
#include "FreeRTOS.h"
#include <vector>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

xSemaphoreHandle gps_sem;
SF_Nav kf;
state_var boatState;		// Stores current state of the boat
std::vector<state_var> targetStates;	// States that we want boat to be at
int targetCounter = 0;
int pwm1 = 0;
int pwm2 = 0;

#define PIDPERIOD 500
#define MOTORMAXDIFF 20
#define MOTORMIN 150 + MOTORMAXDIFF
#define MOTORMAX 250 - MOTORMAXDIFF
#define DISTDEADZONE 0.3	// If robot is within this distance of target, motors dont move
#define DISTSATURATE 5		// If robot is further than this distance of target, motors move at maximum speed

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void MX_I2C1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_USART1_UART_Init(void);
void MX_FREERTOS_Init(void);
double distanceBetweenStates(state_var &state1, state_var &state2) {
	return sqrt(pow(state1.x - state2.x, 2) + pow(state1.y - state2.y, 2));
}
double bearingBetweenStates(state_var &state1, state_var &state2) {
	return atan2(state1.y - state2.y, state1.x - state2.x) * (180.0/3.141592653589793238463);
}

// ---------------------------- Our Tasks --------------------------------------
/*
// Moves to a target state using PID like a noob
void MovePID(void* arg) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(PIDPERIOD);
	xLastWakeTime = xTaskGetTickCount();
	double dError = 0;
	double bError = 0;
	double dIntegral = 0;
	double bIntegral = 0;
	int dKp = 1;
	int dKi = 1;
	int dKd = 1;
	int bKp = 1;
	int bKi = 1;
	int bKd = 1;
	while(1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		// Calculate error between current and set point
		double newDError = distanceBetweenStates(boatState, targetStates[targetCounter]);
		double newBError = bearingBetweenStates(boatState, targetStates[targetCounter]);
		// Riemann sum basically
		double newDIntegral = dIntegral + newDError * PIDPERIOD;
		double newBIntegral = bIntegral + newBError * PIDPERIOD;
		// Basic slope calculation
		double newDDerivative = (newDError - dError) / PIDPERIOD;
		double newBDerivative = (newBError - bError) / PIDPERIOD;

		// TODO: Set pwm duty cycles
		// I need a range of duty cycles that we are working with
		double dOutput = dKp*newDError + dKi*newDIntegral + dKd*newDDerivative;
		double bOutput = bKp*newBError + bKi*newBIntegral + bKd*newBDerivative;
		// Calculate one for distance and one for bearing, then add/superimpose them
		// onto each other to get final pwm duty cycles

		// Update values
		dError = newDError;
		bError = newBError;
		dIntegral = newDIntegral;
		bIntegral = newBIntegral;
	}
}
*/
// Moves to a target using basic error correction function
void MoveToPoint(void* arg) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();
	while(1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		kf.update();
		boatState = kf.get_state();

		// Calculate errors
		double distanceError = distanceBetweenStates(boatState, targetStates[targetCounter]);
		double bearingError = bearingBetweenStates(boatState, targetStates[targetCounter]);

		int pwml = 0;
		int pwmr = 0;
		// NOTE: Might have to negative bearingError
		int bearingAdjustment = pow((0.1 * bearingError), 3);
		// Clamp bearing adjustment to +/-20
		bearingAdjustment = bearingAdjustment > 20 ? 20 :
							bearingAdjustment < -20 ? -20 :
							bearingAdjustment;
		// Only spin motors if robot is further than this amount from target
		if(distanceError > DISTDEADZONE && distanceError < DISTSATURATE) {
			int temp = pow(0.9 * distanceError, 3) + MOTORMIN;	// If distanceError is just under 5, set DC to around 260
			pwml = temp;
			pwmr = temp;
			// Adjust speeds based on bearing error
		}
		else if(distanceError >= DISTSATURATE) {
			pwml = MOTORMAX;
			pwmr = MOTORMAX;
		}

		// NOTE: Might have to add bearingAdjustment to pwmr instead of pwml
		setSpeed(0, pwml + bearingAdjustment, 0, pwmr);
	}
}

// Calls updates on the Kalman Filter and initializes the GPS and IMU
void UpdateKF(void* arg) {
	gps_sem = xSemaphoreCreateBinary();
//	kf.gps.init(&huart1);
//	gps.init(&huart1);
	kf.init(&huart1, &hi2c1, KALMAN_REFRESH_TIME);
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
//		uart_printf("some\r\n");
//		vTaskDelay(1000);
	}
}

void TestMotors(void* arg) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();
	setSpeed(0, 150, 0, 150);
	while(1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

void TurnBoat(void* arg) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();
	while(1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		changeDirection(15);
		vTaskDelay(500);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
  motorInit(&htim1, &htim2, &htim3, &htim4);

  // Set up path
  state_var temp1 = { .x = 0,
  					.y = 1,
  					.b = 0,
					.vX = 0,
					.vY = 0,
					.vB = 0};
  targetStates.push_back(temp1);
  state_var temp2 = { .x = 1,
    					.y = 1,
    					.b = 0,
  					.vX = 0,
  					.vY = 0,
  					.vB = 0};
  targetStates.push_back(temp2);
  state_var temp3 = { .x = 1,
      					.y = 0,
      					.b = 0,
    					.vX = 0,
    					.vY = 0,
    					.vB = 0};
    targetStates.push_back(temp3);
    state_var temp4 = { .x = 0,
        					.y = 0,
        					.b = 0,
      					.vX = 0,
      					.vY = 0,
      					.vB = 0};
      targetStates.push_back(temp4);

  //xTaskCreate(UpdateKF, "kalman", 2048, NULL, 0, NULL);
//  xTaskCreate(MovePID, "pid", 256, NULL, 1, NULL);
  //xTaskCreate(MoveToPoint, "move", 128, NULL, 1, NULL);
  xTaskCreate(TestMotors, "testMotors", 128, NULL, 1, NULL);
  //xTaskCreate(TurnBoat, "turn", 128, NULL, 1, NULL);
  vTaskStartScheduler();

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 690;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 690;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 690;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 690;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

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
