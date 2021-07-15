/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
//save state of Button Matrix
uint16_t BMS = 0;

//Button TimeStamp
uint32_t BMT = 0;
uint32_t Timestamp = 0;

GPIO_TypeDef *Data[11] = { 0b0 };
int data = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
//scan and update data of Button Matrix
void BMU();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		BMU();
		if (HAL_GetTick() - Timestamp >= 250) { //ปรับให้ได้ค่าที่เหมาะสมต่อ�?าร�?ด

			if (BMS == 0b1) { //เช็คว่าตอนนี้�?ำลัง�?ดปุ่มใดอยู่ �?ละเป็น�?าร�?ำหนดว่า ให้ปุ่มนี้�?สดงค่าเลขอะไรออ�?มา หลังจา�?นั้นทำ�?าร +1 ให้ค่าของ data เพื่อเพิ่มค่าของตำ�?หน่งข้อมูลไปเรื่อยๆ �?ล้วจึงทำ�?ารเซ็ทค่า Timestamp
				Data[data] = 7;data += 1;Timestamp = HAL_GetTick();
			}
			else if (BMS == 0b10) {
				Data[data] = 8;data += 1;Timestamp = HAL_GetTick();
			}
			else if (BMS == 0b100) {
				Data[data] = 9;data += 1;Timestamp = HAL_GetTick();
			}
			else if (BMS == 0b10000) {
				Data[data] = 4;data += 1;Timestamp = HAL_GetTick();
			}
			else if (BMS == 0b100000) {
				Data[data] = 5;data += 1;Timestamp = HAL_GetTick();
			}
			else if (BMS == 0b1000000) {
				Data[data] = 6;data += 1;Timestamp = HAL_GetTick();
			}
			else if (BMS == 0b100000000) {
				Data[data] = 1;data += 1;Timestamp = HAL_GetTick();
			}
			else if (BMS == 0b1000000000) {
				Data[data] = 2;data += 1;Timestamp = HAL_GetTick();
			}
			else if (BMS == 0b10000000000) {
				Data[data] = 3;data += 1;Timestamp = HAL_GetTick();
			}
			else if (BMS == 0b1000000000000) {
				Data[data] = 0;data += 1;Timestamp = HAL_GetTick();
			}

			else if (BMS == 0b1000) { //ถ้ามี�?าร�?ดปุ่ม clear จะทำ�?ารรีเซ็ทข้อมูลทุ�?ตำ�?หน่งให้เป็น 0 �?ละไฟจะดับใน�?รณีที่�?่อนหน้านี้ไฟติด
				int i ;
				for (i = 0; i < 12; ++i) {
				Data[i] = 0;
				}
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				Timestamp = HAL_GetTick();
			}

			else if (BMS == 0b1000000000000000) { //ถ้า�?ดปุ่ม ok �?ล้วใส่รหัสถู�?ตามที่�?ำหนด ไฟจะติดสว่างขึ้น
				if (Data[0] == 6 && Data[1] == 2 && Data[2] == 3 && Data[3] == 4 && Data[4] == 0 && Data[5] == 5
					&& Data[6] == 0 && Data[7] == 0 && Data[8] == 0 && Data[9] == 5 && Data[10] == 8)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				}

				Timestamp = HAL_GetTick();
			}

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//port/pin array , 0-3 input , 4-7 output
GPIO_TypeDef *BMPort[8] = {GPIOA,GPIOB,GPIOB,GPIOB,
							GPIOA,GPIOC,GPIOB,GPIOA};
uint16_t BMPin[8] = {GPIO_PIN_10,GPIO_PIN_3,GPIO_PIN_5,GPIO_PIN_4,
						GPIO_PIN_9,GPIO_PIN_7,GPIO_PIN_6,GPIO_PIN_7};
uint8_t BMLine = 0;
void BMU()
{
 if (HAL_GetTick() - BMT >= 50)  //Delay For electronic Change
 {
  BMT = HAL_GetTick();
  int i;
  for(i=0; i<4 ; ++i) //A10 B3 B5 B4
  { //Position/Button Read 0-3
   GPIO_PinState PinState = HAL_GPIO_ReadPin(BMPort[i], BMPin[i]);
   if(PinState == GPIO_PIN_RESET) //Press Button
   {
    BMS |= (uint16_t)0x1 <<(i + BMLine * 4); //Change BMS , 0x1 is 1 ,Ex.(0b0000000000000000 | 0b1) << i = 0b1000 --> (i = 3)

   }
   else //don't press
   {
    BMS &= ~((uint16_t)0x1 <<(i + BMLine * 4)); //Set Bit to 0,Ex.(0b0000000000000000 & ~(0b1)) << i = 0b0111 --> (i = 3)
   }
  }

  uint8_t NowOutPin = BMLine +4; //Now Button 0-3 [+4] for go to Button 4-7 [R2]

  HAL_GPIO_WritePin(BMPort[NowOutPin], BMPin[NowOutPin], GPIO_PIN_SET); //Set R1

  BMLine = (BMLine +1) % 4; // หารเอาเศษ เพื่อรีเซ็ท�?ลับมา 0 อัตโนมัติ ถ้ามันเ�?ิน

  uint8_t NextOutPin = BMLine +4;

  HAL_GPIO_WritePin(BMPort[NextOutPin], BMPin[NextOutPin], GPIO_PIN_RESET); //Reset R2 for Run Continue
 }

 //Reset R1 อัตโนมัติเมื่อถึง Reset R4 ถ้าจะเขียน  Loop �?ร�?ด้วย �?็เขียนไว้�?่อน While เพื่อตั้งค่าไว้ตั้ง�?ต่�?ร�?
}
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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
