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
#include "lcd.h"
#include "matrix.h"
#include "String.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t cntr = 0;
uint8_t tmp = 0;
uint8_t irany = 1;
uint8_t ki = 0;
uint8_t pwm = 0;

char row_1[20];
char row_2[20];
char row_3[20];
char row_4[20];
char szoveg[20];

uint8_t tilt = 0;

uint8_t uart = 0;
uint8_t ch = 0;
char str[50];

uint8_t a_off = 0;

uint16_t fesz = 0;

char adc_ki[5];
uint8_t write = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
		cntr++;

		if(cntr == 100)
		{
			if(irany == 0 && tmp == 2)
			{
				irany = 1;
	  			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	  			cntr = 0;
			}
			else
			{
				if(irany == 1 && tmp == 2)
				{
					irany = 0;
		  			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,100);
		  			cntr = 0;
				}
			}
		}

		if(cntr == 500)
		{
			if(irany == 0 && tmp == 3)
			{
				irany = 1;
	  			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	  			cntr = 0;
			}
			else
			{
				if(irany == 1 && tmp == 3)
				{
					irany = 0;
		  			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,100);
		  			cntr = 0;
				}
			}
			cntr = 0;
		}

		if(tmp == 7)
		{
			if(irany == 0 && pwm <= 100)
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm);
				pwm++;
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm);
				pwm--;
			}
			if(pwm == 100)
			{
				irany = 1;
			}
			if(pwm == 0)
			{
				irany = 0;
			}
		}

		if(htim->Instance == TIM14)
		{
			HAL_ADC_Start(&hadc1);
			uint16_t adc =  HAL_ADC_GetValue(&hadc1);
			fesz = (3300*adc)/4096;

			if(fesz >= 0 && fesz <= 500)
			{
				strcpy(adc_ki,"    ");
				write = 1;
			}
			else
			{
				if(fesz >= 501 && fesz <= 1000)
				{
					strcpy(adc_ki,"X   ");
					write = 1;
				}
				else
				{
					if(fesz >= 1001 && fesz <= 2000)
					{
						strcpy(adc_ki,"XX  ");
						write = 1;
					}
					else
					{
						if(fesz >= 2001 && fesz <= 3000)
						{
							strcpy(adc_ki,"XXX ");
							write = 1;
						}
						else
						{
							strcpy(adc_ki,"XXXX");
							write = 1;
						}
					}
				}
			}


			HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);

		}
}



void LCD ()
{
	char out[2];
	sprintf(out,"%d",tmp);

	HAL_UART_Transmit(&huart3, (uint8_t*) out, sizeof(out), HAL_MAX_DELAY);

	strcpy(row_1,row_2);
	strcpy(row_2,row_3);
	strcpy(row_3,row_4);
	strcpy(row_4,szoveg);

	LCD_cmd(0x01);

	LCD_xy(0,1);
	LCD_string(row_1);

	LCD_xy(0,2);
	LCD_string(row_2);

	LCD_xy(0,3);
	LCD_string(row_3);

	LCD_xy(0,4);
	LCD_string(row_4);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(uart == 13)
		{
			if(strcmp(str,"") == 0)
			{
				tilt = 0;
				ch = 0;
			}
			else
			{
				if(strcmp(str,"A_OFF") == 0)
				{
					a_off = 1;
					ch = 0;
					tilt = 0;
				}
				else
				{
					if(strcmp(str,"D_OFF") == 0)
					{
						a_off = 0;
						ch = 0;
						tilt = 0;
					}
				}
			}

		}
		else
		{
			str[ch++] = uart;
		}

		HAL_UART_Receive_IT(&huart3, &uart, 1);

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
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  LCD_init();
  LCD_cmd(0x01);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  matrix_search();
	  int num = get_key();
	  if(num < 12)
	  {
		  switch(num)
		  {
		  	case 1:
		  		if(tilt == 0)
		  		{
			  		if(tmp != 1 && ki == 0)
			  		{
			  			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,20);
			  			ki = 1;
			  			tmp = 1;
			  		}
			  		else
			  		{
			  			if(tmp == 1 && ki ==1)
			  			{
			  				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
			  				ki = 0;
			  				strcpy(szoveg,"OFF");
			  				LCD();
			  			}
			  		}
		  		}
		  	break;

		  	case 2:
		  		if(tmp!=2 && tilt == 0 && a_off == 0)
		  		{
		  			tmp = 2;
		  			strcpy(szoveg,"Connecting");
		  			LCD();
		  		}
		  	break;

		  	case 3:
		  		if(tmp!=3 && tilt == 0 && a_off == 0)
		  		{
		  			tmp = 3;
		  			strcpy(szoveg,"Normal");
		  			LCD();
		  		}
		  	break;

		  	case 4:
		  		if(tmp!=4 && tilt == 0 && a_off == 0)
		  		{
		  			tmp = 4;
		  			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,5);
		  			strcpy(szoveg,"5%");
		  			LCD();
		  		}
		  	break;

		  	case 5:
		  		if(tmp!=5 && tilt == 0 && a_off == 0)
		  		{
		  			tmp = 5;
		  			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,50);
		  			strcpy(szoveg,"50%");
		  			LCD();
		  		}
		  	break;

		  	case 6:
		  		if(tmp!=6 && tilt == 0 && a_off == 0)
		  		{
		  			tmp = 6;
		  			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,98);
		  			strcpy(szoveg,"98% PWM");
		  			LCD();
		  		}
		  	break;

		  	case 7:
		  		if(tmp!=7 && tilt == 0 && a_off == 0)
		  		{
		  			tmp = 7;
		  			strcpy(szoveg,"DANGER!!!!");
		  			LCD();
		  		}
		  	break;

		  	case 8:
		  		if(tilt == 0)
		  		{
			  		if(tmp != 8 && ki == 0)
			  		{
			  			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,20);
			  			ki = 1;
			  			tmp = 8;
			  		}
			  		else
			  		{
			  			if(tmp == 8 && ki ==1)
			  			{
			  				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,100);
			  				ki = 0;
			  				strcpy(szoveg,"ON");
			  				LCD();
			  			}
			  		}
		  		}
		  	break;

			case 11:
				  tilt = 1;
				  char text[] = "PROGRAM Active OFF – \"A_OFF\"\n\rProgram Deactive OFF – \"D_OFF\"";
				  HAL_UART_Transmit(&huart3, (uint8_t*) text, sizeof(text), HAL_MAX_DELAY);
				  HAL_UART_Receive_IT(&huart3, &uart, 1);
			break;

		  }
	  }
	  if(write == 1)
	  {
		  write = 0;
		  LCD_xy(12,4);
		  LCD_string(adc_ki);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 50;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 30;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
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
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 16000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, KB_PC3_OUT_row1_Pin|KB_PC5_OUT_row3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_RS_Pin|LCD_E_Pin|LCD_DATA_4_Pin|KB_PC4_OUT_row2_Pin
                          |LCD_DATA_5_Pin|LCD_DATA_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DATA_7_Pin|RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KB_PC6_OUT_row4_GPIO_Port, KB_PC6_OUT_row4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KB_PC0_IN_LEFT_Pin */
  GPIO_InitStruct.Pin = KB_PC0_IN_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KB_PC0_IN_LEFT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KB_PC3_OUT_row1_Pin KB_PC5_OUT_row3_Pin */
  GPIO_InitStruct.Pin = KB_PC3_OUT_row1_Pin|KB_PC5_OUT_row3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin LCD_DATA_4_Pin KB_PC4_OUT_row2_Pin
                           LCD_DATA_5_Pin LCD_DATA_6_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|LCD_DATA_4_Pin|KB_PC4_OUT_row2_Pin
                          |LCD_DATA_5_Pin|LCD_DATA_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : KB_PC1_IN_CENTER_Pin KB_PC2_IN_RIGHT_Pin */
  GPIO_InitStruct.Pin = KB_PC1_IN_CENTER_Pin|KB_PC2_IN_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DATA_7_Pin RED_Pin */
  GPIO_InitStruct.Pin = LCD_DATA_7_Pin|RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KB_PC6_OUT_row4_Pin */
  GPIO_InitStruct.Pin = KB_PC6_OUT_row4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KB_PC6_OUT_row4_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
