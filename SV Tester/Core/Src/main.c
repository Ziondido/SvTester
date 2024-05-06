/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 RADGA9 GAF AHAT.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SVFunc.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

/* USER CODE BEGIN PV */

SVT_Mode ModeFlag = SLEEP_MODE;
SVT_TestStatus TestStatus = TEST_FAILED;
SVT_SamplesValues SamplesValues;

uint8_t	SwitchArray[6] = {SW1_Pin,SW2_Pin,SW3_Pin,SW4_Pin,SW5_Pin,SW6_Pin};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SELF_TEST_EXTI6_Pin)
	{
		ModeFlag = SELF_TEST;
	}
	else if(GPIO_Pin == STRAY_VOLTAGE_TEST_EXTI7_Pin)
	{
		ModeFlag = SV_TEST;
	}
}









/* USER CODE END 0 */

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
  MX_DMA_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  SamplesValues.RMSValue = 0;
  SamplesValues.SamplesAvg = 0;
  SamplesValues.SamplesSum = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*This is necessary after exiting stop mode*/
		SystemClock_Config();					//System Clock Configuration
		HAL_ResumeTick();						//Resumes the Tick increment
		HAL_ADC_Start(&hadc);					//Enable ADC, start conversion of regular group

		/*Default PINs states*/
		HAL_GPIO_WritePin(GPIOA, SW1_Pin, SET);
		HAL_GPIO_WritePin(GPIOA, SW2_Pin, SET);
		HAL_GPIO_WritePin(GPIOA, SW3_Pin, SET);
		HAL_GPIO_WritePin(GPIOA, SW4_Pin, SET);
		HAL_GPIO_WritePin(GPIOA, SW5_Pin, SET);
		HAL_GPIO_WritePin(GPIOA, SW6_Pin, SET);
		HAL_GPIO_WritePin(GPIOA, SW7_Pin, SET);
		HAL_GPIO_WritePin(GPIOB, AUX_EN_Pin, SET);


		/* The delay needs to make a sure of changing of voltage level in GPIOs*/
		HAL_Delay(50);

		/* Calculation Offset before tests*/
		SAMPLES_DC_Offset_Calibration(hadc);

		// in self test pass, blink both leds a couple times and then turn the respective led on until button released.
		// if failed, keep red led on until a self test passed, or for...10 sec?.
		if(ModeFlag == SELF_TEST)
		{
			HAL_GPIO_WritePin(GPIOB, AUX_EN_Pin, RESET);
			HAL_Delay(50);

			SAMPLES_Get(hadc,&SamplesValues);
			SAMPLES_To_Voltage(&SamplesValues);
			SAMPLES_Gain_Error_Calibration(&SamplesValues);
			TestStatus = DC_SV_Test(SamplesValues,SELF_TEST_VREF*0.95 , SELF_TEST_VREF*1.05);
			if(TestStatus == TEST_SUCCESS)
			{
				LEDs_Blink(1,200);
			}
			HAL_GPIO_WritePin(GPIOA, SW7_Pin, RESET);

			for(int i=0;i<6;i++)
			{

				HAL_GPIO_WritePin(GPIOA, SwitchArray[i], RESET);
				HAL_Delay(50);
				SAMPLES_Get(hadc,&SamplesValues);
				HAL_GPIO_WritePin(GPIOA, SwitchArray[i], SET);
				SAMPLES_Sum_Buffer(&SamplesValues);
				SamplesValues.SamplesAvg = SamplesValues.SamplesSum / SAMPLES_TIMES;
				SAMPLES_To_Voltage(&SamplesValues);
				SAMPLES_Gain_Error_Calibration(&SamplesValues);

				TestStatus = DC_SV_Test(SamplesValues, SELF_TEST_ADC_VAL*0.95, SELF_TEST_ADC_VAL*1.05);
				if(TestStatus == TEST_SUCCESS)
				{
					TestStatus = AC_RMS_Test(SamplesValues);
				}
				if(TestStatus == TEST_FAILED)
				{
					break;
				}
			}
			if(TestStatus == TEST_SUCCESS)
			{
				HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin, SET);
				HAL_Delay(200);
				HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin, RESET);
			}
			else if(TestStatus == TEST_FAILED)
			{
				HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, SET);
				HAL_Delay(200);
				HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, RESET);
			}

			HAL_GPIO_WritePin(GPIOB, AUX_EN_Pin, SET);
			ModeFlag = SLEEP_MODE;
		}

		// in stray voltage test, turn the green led until if passed until button released.
		// if failed, keep red led on until a self test passed, or for...10 sec?.
		//add test for all 6 pins
		else if(ModeFlag == SV_TEST)
		{
			HAL_GPIO_WritePin(GPIOB, AUX_EN_Pin, RESET);
			for(int i = 0 ; i < 6 ; i++)
			{
				HAL_GPIO_WritePin(GPIOA, SwitchArray[i], RESET);
				HAL_Delay(50);
				SAMPLES_Get(hadc,&SamplesValues);
				// SamplesValues.SamplesBuffer[i] = SamplesValues.SamplesBuffer[i];
				HAL_GPIO_WritePin(GPIOA, SwitchArray[i], SET);
				SAMPLES_To_Voltage(&SamplesValues);
				SAMPLES_Gain_Error_Calibration(&SamplesValues);
 				TestStatus = DC_SV_Test(SamplesValues, SELF_TEST_VREF-SV_DC_THRESHOLD, SELF_TEST_VREF+SV_DC_THRESHOLD);
				if(TestStatus == TEST_SUCCESS)
				{
					TestStatus = AC_RMS_Test(SamplesValues);
				}
				if(TestStatus == TEST_FAILED)
				{
					break;
				}

			}
			if(TestStatus == TEST_SUCCESS)
			{
				HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin, SET);
				HAL_Delay(250);
				HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin, RESET);
			}
			if(TestStatus == TEST_FAILED)
			{
				HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, SET);
				HAL_Delay(250);
				HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, RESET);
			}



			HAL_GPIO_WritePin(GPIOB, AUX_EN_Pin, SET);
 			ModeFlag = SLEEP_MODE;
		}

		/*Entering to sleep mode if the flag is SLEEP_MODE*/
		if(ModeFlag == SLEEP_MODE)
		{
			HAL_ADC_Stop(&hadc);													//Stop the ADC
			HAL_SuspendTick();														//Suspend the tick count
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);		//Enter to stop-mode with interrupt wake-up
		}
	}
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin
                          |SW5_Pin|SW6_Pin|SW7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AUX_EN_GPIO_Port, AUX_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin
                           SW5_Pin SW6_Pin SW7_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin
                          |SW5_Pin|SW6_Pin|SW7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AUX_EN_Pin */
  GPIO_InitStruct.Pin = AUX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AUX_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SELF_TEST_EXTI6_Pin STRAY_VOLTAGE_TEST_EXTI7_Pin */
  GPIO_InitStruct.Pin = SELF_TEST_EXTI6_Pin|STRAY_VOLTAGE_TEST_EXTI7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
