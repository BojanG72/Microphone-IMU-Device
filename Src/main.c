
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "fatfs.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
HAL_SD_CardInfoTypeDef SDCardInfo;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void mountSdCard(void);
static void createNewFiles(void);
static void initImu(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//State Variables
uint8_t progState = 0;

// progState is the current state of the device.
// progState = 0 on start up.
// when progState = 0, no data is being recorded and you can access the memory card via USB
// progState = 1 recording is started and files need to be initialized
// progState = 2 data is being recorded, USB is not active and the device cannot be charged

//ADC variables for the microphone
uint32_t adcVal[15000];
uint32_t writeCount = 0;

//I2C Variables for the IMU
uint8_t statBuf[2];
uint8_t justOne = 1;
uint8_t statusBit;
uint8_t i2cBuf[8];
uint8_t gyroBuf[8];
int16_t ax,ay,az;
int16_t gx,gy,gz;
uint16_t accBufPointer = 0;
int accHalfFull = 0;
int accFull = 0;
float accDataBuf1[300];
float accDataBuf2[300];
float accData[3];

float gyroDataBuf1[300];
float gyroDataBuf2[300];
float gyroData[3];

//SDIO variables
int noMount = 0;
char myPath[] = "SND.BIN\0";
char accPath[] = "ACC.BIN\0";
char gyroPath[] = "GYRO.BIN\0";
FATFS myFATFS;
FIL myFILE;
FIL accFILE;
FIL gyroFILE;
UINT testByte;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_Delay(100);

  // Mount SD card
  mountSdCard();
  
  // Initialize the IMU
  initImu();

  //start timer for the ADC, currently set to 10kHz
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
    if (noMount > 0){
      for (;;){
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(500);
      }
    }
    
    while(progState == 0){
      // Do nothing until the button is pressed
    }
    
    if(progState == 1){
      // Create all the Data Files
      createNewFiles();
      //Start ADC as DMA
      HAL_ADC_Start_DMA(&hadc1, adcVal, 15000);
      HAL_Delay(100);
      progState = 2;
    }
                
		
    while (accBufPointer <= 297){
      
      // Read from the status register of the IMU to know when new data is ready
      statBuf[0] = 0x1E;
      HAL_I2C_Master_Transmit(&hi2c1, 0xD6, statBuf,1,10);
      statBuf[1] = 0x00;
      HAL_I2C_Master_Receive(&hi2c1, 0xD6, &statBuf[1],1, 10);
      
      statusBit = statBuf[1] & justOne;
      
      if (statusBit == 1){
        statusBit = 0;
        // Begin reading the acc data from the IMU
        // Save the acc data to a buffer for writing to SD card
        i2cBuf[0] = 0x28;
        HAL_I2C_Master_Transmit(&hi2c1, 0xD6, i2cBuf,1,10);
        i2cBuf[1] = 0x00;
        HAL_I2C_Master_Receive(&hi2c1, 0xD6, &i2cBuf[1],6, 10);
        
        ax = -(i2cBuf[2]<<8 | i2cBuf[1]);
        ay = -(i2cBuf[4]<<8 | i2cBuf[3]);
        az = (i2cBuf[6]<<8 | i2cBuf[5]);
        
        accDataBuf1[accBufPointer] = ax/8192.0;
        accDataBuf1[accBufPointer+1] = ay/8192.0;
        accDataBuf1[accBufPointer+2] = az/8192.0;
        
        // Begin reading the gyro data from the IMU
        // Save the gyro data to a buffer for writing to SD card
        gyroBuf[0] = 0x22;
        HAL_I2C_Master_Transmit(&hi2c1, 0xD6, gyroBuf,1,10);
        gyroBuf[1] = 0x00;
        HAL_I2C_Master_Receive(&hi2c1, 0xD6, &gyroBuf[1],6, 10);
        
        gx = (gyroBuf[2]<<8 | gyroBuf[1]);
        gy = (gyroBuf[4]<<8 | gyroBuf[3]);
        gz = (gyroBuf[6]<<8 | gyroBuf[5]);
        
        gyroDataBuf1[accBufPointer] = gx/1.0;
        gyroDataBuf1[accBufPointer+1] = gy/1.0;
        gyroDataBuf1[accBufPointer+2] = gz/1.0;
        
        accBufPointer = accBufPointer + 3;
      }
    }
    
    //Set a flag that the first buffer is full before saving to the second buffer
    accHalfFull = 1;
    //Reset the buffer pointer to 0
    accBufPointer = 0;
    
    while (accBufPointer <= 297){
      
      // Read from the status register of the IMU to know when new data is ready
      statBuf[0] = 0x1E;
      HAL_I2C_Master_Transmit(&hi2c1, 0xD6, statBuf,1,10);
      statBuf[1] = 0x00;
      HAL_I2C_Master_Receive(&hi2c1, 0xD6, &statBuf[1],1, 10);
      
      statusBit = statBuf[1] & justOne;
      
      if (statusBit == 1){
        
        statusBit = 0;
        // Begin reading the acc data from the IMU
        // Save the acc data to a buffer for writing to SD card
        i2cBuf[0] = 0x28;
        HAL_I2C_Master_Transmit(&hi2c1, 0xD6, i2cBuf,1,10);
        i2cBuf[1] = 0x00;
        HAL_I2C_Master_Receive(&hi2c1, 0xD6, &i2cBuf[1],6, 10);
        
        ax = -(i2cBuf[2]<<8 | i2cBuf[1]);
        ay = -(i2cBuf[4]<<8 | i2cBuf[3]);
        az = (i2cBuf[6]<<8 | i2cBuf[5]);
        
        accDataBuf2[accBufPointer] = ax/8192.0;
        accDataBuf2[accBufPointer+1] = ay/8192.0;
        accDataBuf2[accBufPointer+2] = az/8192.0;
        
        // Begin reading the gyro data from the IMU
        // Save the gyro data to a buffer for writing to SD card
        gyroBuf[0] = 0x22;
        HAL_I2C_Master_Transmit(&hi2c1, 0xD6, gyroBuf,1,10);
        gyroBuf[1] = 0x00;
        HAL_I2C_Master_Receive(&hi2c1, 0xD6, &gyroBuf[1],6, 10);
        
        gx = (gyroBuf[2]<<8 | gyroBuf[1]);
        gy = (gyroBuf[4]<<8 | gyroBuf[3]);
        gz = (gyroBuf[6]<<8 | gyroBuf[5]);
        
        gyroDataBuf2[accBufPointer] = gx/1.0;
        gyroDataBuf2[accBufPointer+1] = gy/1.0;
        gyroDataBuf2[accBufPointer+2] = gz/1.0;
        
        accBufPointer = accBufPointer + 3;
      }
      
      
    }
    
    //Set flag that the second buffer is full
    accFull = 1;
    //Reset the buffer pointer to 0 
    accBufPointer = 0;
			
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SDMMC1 init function */
static void MX_SDMMC1_SD_Init(void)
{

  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 479;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
	writeCount++;
	
	f_write(&myFILE, &adcVal[7500], sizeof(adcVal)/2, &testByte);
	f_sync(&myFILE);
	
	if (accHalfFull == 1){
		f_write(&accFILE, accDataBuf1, sizeof(accDataBuf1), &testByte);
		f_sync(&accFILE);
		f_write(&gyroFILE, gyroDataBuf1, sizeof(gyroDataBuf1), &testByte);
		f_sync(&gyroFILE);
		accHalfFull = 0;
	}
	
	if (accFull == 1){
		f_write(&accFILE, accDataBuf2, sizeof(accDataBuf2), &testByte);
		f_sync(&accFILE);
		f_write(&gyroFILE, gyroDataBuf2, sizeof(gyroDataBuf2), &testByte);
		f_sync(&gyroFILE);
		accFull = 0;
	}
	
	//Check if 8hrs have passed since writting began (8hrs = 21600 writes)
	if (writeCount == 21600 && noMount == 0){
                progState = 0;
		HAL_ADC_Stop_DMA(&hadc1);
		f_close(&myFILE);
		f_close(&accFILE);
		f_close(&gyroFILE);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
	writeCount++;
	
	f_write(&myFILE, adcVal, sizeof(adcVal)/2, &testByte);
	f_sync(&myFILE);
	
	if (accHalfFull == 1){
		f_write(&accFILE, accDataBuf1, sizeof(accDataBuf1), &testByte);
		f_sync(&accFILE);
		f_write(&gyroFILE, gyroDataBuf1, sizeof(gyroDataBuf1), &testByte);
		f_sync(&gyroFILE);
		accHalfFull = 0;
	}
	
	if (accFull == 1){
		f_write(&accFILE, accDataBuf2, sizeof(accDataBuf2), &testByte);
		f_sync(&accFILE);
		f_write(&gyroFILE, gyroDataBuf2, sizeof(gyroDataBuf2), &testByte);
		f_sync(&gyroFILE);
		accFull = 0;
	}
	
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  
  // Wait for button to be pressed to begin recording data
  if(progState == 0){
   progState = 1;
   HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  
  // Stop recording data and close all the files
  if(progState == 2){
    progState = 0;
    HAL_ADC_Stop_DMA(&hadc1);
    f_close(&myFILE);
    f_close(&accFILE);
    f_close(&gyroFILE);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  
}

void mountSdCard(){
  
  if(f_mount(&myFATFS, myPath, 1) == FR_OK){
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
  }else{
    noMount = 1;
  }

  if(f_mount(&myFATFS, accPath, 1) == FR_OK){
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
  }else{
    noMount = 2;
  }
	
  if(f_mount(&myFATFS, gyroPath, 1) == FR_OK){
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);
  }else{
    noMount = 3;
  }
}

void createNewFiles(){
  
  //This function will create and open the data files
  if(f_open(&myFILE, myPath, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK){
    HAL_Delay(100);
  }
	
  if(f_open(&accFILE, accPath, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK){
    HAL_Delay(100);
  }
		
  if(f_open(&gyroFILE, gyroPath, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK){
    HAL_Delay(100);
  }
  
}

void initImu(){
  
  i2cBuf[0] = 0x10; // Write to register 10h
  i2cBuf[1] = 0x38; // Set the accelerometer sample rate to 52Hz and the range to +/-4g 
  HAL_I2C_Master_Transmit(&hi2c1, 0xD6, i2cBuf,2,10);
  
//	// For debugging purposes, check if 0x38 was written to register 10h
//	i2cBuf[0] = 0x10;
//	HAL_I2C_Master_Transmit(&hi2c1, 0xD6, i2cBuf,1,10);
//	i2cBuf[1] = 0x00;
//	HAL_I2C_Master_Receive(&hi2c1, 0xD6, &i2cBuf[1],1, 10);
	
  i2cBuf[0] = 0x11; // Write to register 11h
  i2cBuf[1] = 0x30; // Set the gyroscope sample rate to 52Hz and the range to 250 dps
  HAL_I2C_Master_Transmit(&hi2c1, 0xD6, i2cBuf,2,10);
  
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
