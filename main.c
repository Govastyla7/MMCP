/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MMCP_MY_ADDRESS 69
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* GLOBALS */
uint8_t myAddress = 111;
uint8_t buffer [L1_PDU_size];
uint8_t L1_PDU_GLOBAL [L1_PDU_size];
uint8_t cnt=0;
bool empfangen = false;
bool LED_AN = false;
bool TASTER_PRELLT = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void L1_receive(uint8_t L1_PDU[]);
void L1_send(uint8_t L1_SDU[]);
void L2_receive(uint8_t L2_PDU[]);
void L2_send(uint8_t L2_SDU[]);
void L3_receive(uint8_t L3_PDU[]);
void L3_send(uint8_t L3_SDU[]);
void L7_receive(uint8_t L7_PDU[]);
void L7_send(uint8_t Id, uint8_t L7_SDU[]);

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_UART_Receive_IT(&huart2, buffer, L1_PDU_size);
	  if(empfangen==true){
		  L1_receive(L1_PDU_GLOBAL);
		  empfangen = false;
	  }

	  if(LED_AN == true){
		  HAL_GPIO_WritePin ( GPIOA , GPIO_PIN_5 , GPIO_PIN_SET );
	  }
	  if(LED_AN == false){
		  HAL_GPIO_WritePin ( GPIOA , GPIO_PIN_5 , GPIO_PIN_RESET );
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600000 ;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_TIM_Base_Stop_IT(&htim2);
	TASTER_PRELLT = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	int i;
	for(i=0; i<L1_PDU_size;i++){
		L1_PDU_GLOBAL[i]=buffer[i];

	}
	empfangen = true;
}

void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin ) {
	if ( GPIO_Pin == B1_Pin && TASTER_PRELLT == false ){
		cnt ++;
		TASTER_PRELLT=true;
		HAL_TIM_Base_Start_IT(&htim2);
	}
}

void L1_receive(uint8_t L1_PDU[]){
	uint8_t L1_SDU[L1_SDU_size];
	int i;
	for(i=0; i<14;i++){
		L1_SDU[i]=L1_PDU[i+1];
	}
	L2_receive(L1_SDU);

}

void L2_receive(uint8_t L2_PDU[]){
	uint8_t pruefsumme = 0x0;
	uint8_t rpruefsumme;
	uint8_t L2_SDU[L2_SDU_size];
	int i;
	for(i=0;i<13;i++){
		pruefsumme = pruefsumme + L2_PDU[i];
	}
	rpruefsumme = ~pruefsumme;
	if(L2_PDU[13] == rpruefsumme){
		for(i=0; i<13;i++){
			L2_SDU[i]=L2_PDU[i];
		}
		L3_receive(L2_SDU);
	};
}

void L3_receive(uint8_t L3_PDU[]){
	uint8_t L3_SDU[L3_SDU_size];
	int i;
	for(i=0;i<9;i++){
		L3_SDU[i]=L3_PDU[i+4];
	}
	if((L3_PDU[2]!=4) || (L3_PDU[0]==0 && L3_PDU[1]==0))return;
	if(L3_PDU[0]==myAddress && L3_PDU[1]==0){
		L7_receive(L3_SDU);
	}
	if(L3_PDU[1]==0 && L3_PDU[0]!=myAddress){
		L3_PDU[3]++;
		L2_send(L3_PDU);
	}
}

void L7_receive(uint8_t L7_PDU[]){
	uint8_t L7_SDU[L7_SDU_size];
	uint8_t id = L7_PDU[0];
	int i;
	for(i=0;i<8;i++){
		L7_SDU[i]=L7_PDU[i+1];
	}
	switch(id){
		case 100:
			if(L7_SDU[7]!=0) LED_AN = true;
			else LED_AN = false;
			L7_send(100, L7_SDU);
			break;
		case 101:
			L7_SDU[7]= cnt;
			cnt = 0;
			L7_send(101, L7_SDU);
			break;
		case 102:
			for(i=0;i<4;i++){
				L7_SDU[i] = (uint8_t) (HAL_GetUIDw0() >> (8*i));
			}
			for(i=4;i<7;i++){
				L7_SDU[i] = (uint8_t) (HAL_GetUIDw1() >> (8*(i-4)));
			}
			L7_send(102, L7_SDU);
			break;
		case 103:
			for(i=0;i<4;i++){
				L7_SDU[i] = (uint8_t) (HAL_GetUIDw2() >> (8*i));
			}
			L7_send(102, L7_SDU);
			break;
	}
}

void L7_send(uint8_t Id, uint8_t L7_SDU[]){
	uint8_t L7_PDU [L7_PDU_size];
	int i;
	L7_PDU[0] = Id;
	for(i=0; i<8;i++){
		L7_PDU[i+1] = L7_SDU[i];
	}
	L3_send(L7_PDU);
}

void L3_send(uint8_t L3_SDU[]){
	int i;
	uint8_t L3_PDU [L3_PDU_size];
	L3_PDU[0] = 0;
	L3_PDU[1] = myAddress;
	L3_PDU[2] = 4;
	L3_PDU[3] = 0;
	for(i=4;i<13;i++){
		L3_PDU[i]= L3_SDU[i-4];
	}
	L2_send(L3_PDU);

}

void L2_send(uint8_t L2_SDU[]){
	uint8_t pruefsumme = 0;
	uint8_t rpruefsumme;
	uint8_t L2_PDU[L2_PDU_size];
	int i;
	for(i=0;i<13;i++){
		L2_PDU[i]=L2_SDU[i];
		pruefsumme = pruefsumme + L2_SDU[i];
	}
	rpruefsumme = ~pruefsumme;
	L2_PDU[13] = rpruefsumme;
	L1_send(L2_PDU);
}

void L1_send(uint8_t L1_SDU[]){
	uint8_t L1_PDU [L1_PDU_size];
	int i;
	for(i=1;i<15;i++){
		L1_PDU[i]= L1_SDU[i-1];
	}
	L1_PDU[0] = 0x0;
	L1_PDU[15] = 0x0;
	HAL_UART_Transmit_IT(&huart2, L1_PDU, 16);
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
