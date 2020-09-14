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
#include "main.h"
#include "usb_device.h"

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
TSC_HandleTypeDef htsc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TSC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t row_key[3] = {TSC_GROUP1_IO1,TSC_GROUP1_IO2,TSC_GROUP1_IO3};
uint32_t column_key[3] = {TSC_GROUP2_IO2,TSC_GROUP2_IO3,TSC_GROUP2_IO4};
uint32_t threshold_x = 700;
uint32_t threshold_y = 700;
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
  MX_TSC_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  uint32_t data_x[3];
  uint32_t data_y[3];

  uint32_t data_calib_x = 0;
  uint32_t data_calib_y = 0;

  for(int i = 0; i<100; i++){
  	  htsc.Instance -> CR |= TSC_IODEF_OUT_PP_LOW;
  	  HAL_Delay(1);
  	  htsc.Instance -> IOCCR |= TSC_GROUP1;
  	  htsc.Instance -> CR |= TSC_CR_START;
  	  while((htsc.Instance->ISR & TSC_ISR_EOAF)!= 1){

  	  }
  	    data_calib_x += htsc.Instance->IOGXCR[0];
        htsc.Instance-> ICR = TSC_ICR_EOAIC;
     }
  htsc.Instance -> IOCCR = 0;
  threshold_x = (uint32_t) data_calib_x/100 - 50;

  for(int i = 0; i<50; i++){
  	  htsc.Instance -> CR |= TSC_IODEF_OUT_PP_LOW;
  	  HAL_Delay(1);
  	  htsc.Instance -> IOCCR |= TSC_GROUP2;
  	  htsc.Instance -> CR |= TSC_CR_START;
  	  while((htsc.Instance->ISR & TSC_ISR_EOAF)!= 1){

  	  }
  	    data_calib_y += htsc.Instance->IOGXCR[1];
        htsc.Instance-> ICR = TSC_ICR_EOAIC;
     }
  threshold_y = (uint32_t) data_calib_y/50 - 50;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,GPIO_PIN_SET);
  htsc.Instance -> IOCCR = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    for(int i = 0; i<3; i++){
	  htsc.Instance -> CR |= TSC_IODEF_OUT_PP_LOW;
	  HAL_Delay(1);
	  htsc.Instance -> IOCCR |= row_key[i];
	  htsc.Instance -> CR |= TSC_CR_START;
	  while((htsc.Instance->ISR & TSC_ISR_EOAF)!= 1){

	  }
      data_x[i] = htsc.Instance->IOGXCR[0];
      htsc.Instance -> IOCCR = 0;
      htsc.Instance-> ICR = TSC_ICR_EOAIC;
       }
    for(int j = 0; j<3; j++){
      	  htsc.Instance -> CR |= TSC_IODEF_OUT_PP_LOW;
      	 	  HAL_Delay(1);
      	 	  htsc.Instance -> IOCCR |= column_key[j];
      	 	  htsc.Instance -> CR |= TSC_CR_START;
      	 	  while((htsc.Instance->ISR & TSC_ISR_EOAF)!= 1){

      	 	  }
      	       data_y[j] = htsc.Instance->IOGXCR[1];
      	       htsc.Instance-> ICR = TSC_ICR_EOAIC;
      	     htsc.Instance -> IOCCR = 0;
      	       }
        for (int j = 0; j <3; j++){
        	 if(data_y[j] < 800){
        	                       HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
        	                       HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */
  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP1_IO1|TSC_GROUP1_IO2|TSC_GROUP1_IO3|TSC_GROUP2_IO2
                    |TSC_GROUP2_IO3|TSC_GROUP2_IO4;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO1;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
