/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
uint8_t configuration;
uint8_t shunt_voltage;
uint8_t bus_voltage;
uint8_t power;
uint8_t current;
uint8_t calibration;
}registerPtr_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/********DHT***********/
#define DHT_port GPIOD
#define DHT_pin GPIO_PIN_11
#define TWITCH_port GPIOD
#define TWITCH_pin GPIO_PIN_12
#define DHT_DETECT_port GPIOD
#define DHT_DETECT_pin GPIO_PIN_14
/**********************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TWITCH  HAL_GPIO_TogglePin(TWITCH_port, TWITCH_pin);\
				delayUS(1);\
				HAL_GPIO_TogglePin(TWITCH_port, TWITCH_pin);

/********INA*********/
#define slave_addr 0x40
#define currentLSB 0.06
#define powerLSB   1.2
#define shuntVLSB  0.010
#define busVLSB    4
/********************/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/**********DHT*********/
GPIO_InitTypeDef gpioINIT;
uint8_t DHTdataBuff[5];
/********************/
/********INA*********/
uint8_t txBuff[3];
uint8_t rxBuff[2];
/********************/
/********CAN*********/
CAN_TxHeaderTypeDef  TxHeader;
CAN_RxHeaderTypeDef  RxHeader;
uint8_t              TxData[8];
uint8_t              RxData[8];
uint32_t             TxMailbox;
/********************/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
/*********DHT***********/
static void delayUS(uint16_t);
void startBit();
void sensorPinOUT();
void sensorPinIN();
uint8_t detect();
void acquireData(uint8_t *);
/**********************/


/********INA*********/
uint8_t changeRegPtr(I2C_HandleTypeDef * , uint16_t , uint8_t * );
uint8_t getBytes(I2C_HandleTypeDef * , uint16_t , uint8_t * , uint8_t);
uint8_t putBytes(I2C_HandleTypeDef * , uint16_t , uint8_t * , uint8_t);
/********************/
/*********CAN********/
void CAN_filterConfig(void);
/********************/

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
	uint8_t retval = 0;
	float temp;
	registerPtr_t reg_ptr = {
			.configuration = 0x00,
			.shunt_voltage = 0x01,
			.bus_voltage   = 0x02,
			.power         = 0x03,
			.current       = 0x04,
			.calibration   = 0x05
	};
	float current_mA;
	float bus_mV;
	float bus_V;
	float shunt_mV;
	float bus_mW;
	uint16_t current_regV;
	uint16_t busV_regV;
	uint16_t shunt_regV;
	uint16_t busP_regV;
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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);

  //read and print default configuration register
  retval = changeRegPtr(&hi2c1,slave_addr,&(reg_ptr.configuration));
  printf("Change address to config_reg:%u\n",retval);
  retval = getBytes(&hi2c1 , slave_addr, rxBuff , 2);
  printf("Read success:%u\n",retval);
  printf("B1:%x\t B2:%x\n",rxBuff[0],rxBuff[1]);

  //writing configuration register accordingly
  txBuff[0]= reg_ptr.configuration;
  txBuff[1]= 0x9E; //0x15;
  txBuff[2]= 0x1F; //0x0f;
  retval = putBytes(&hi2c1, slave_addr, txBuff,3);
  printf("Write success at config_reg:%u\n",retval);


  //read an print updated configuration register
  retval = changeRegPtr(&hi2c1,slave_addr,&(reg_ptr.configuration));
  printf("Change address to config_reg:%u\n",retval);
  retval = getBytes(&hi2c1 , slave_addr, rxBuff , 2);
  printf("Read success:%u\n",retval);
  printf("B1:%x\t B2:%x\n",rxBuff[0],rxBuff[1]);


  //writing calibration register accordingly
  txBuff[0]= reg_ptr.calibration;
  txBuff[1]= 0x1A; //0x35;
  txBuff[2]= 0xAA; //0x54;

  //0x1AAA is left shifted by 1
  retval = putBytes(&hi2c1, slave_addr, txBuff,3);
  printf("Write success at calib_reg:%u\n",retval);

  //read an print updated calibration register
  retval = changeRegPtr(&hi2c1,slave_addr,&(reg_ptr.calibration));
  printf("Change address to config_reg:%u\n",retval);
  retval = getBytes(&hi2c1 , slave_addr, rxBuff , 2);
  printf("Read success:%u\n",retval);
  printf("B1:%x\t B2:%x\n",rxBuff[0],rxBuff[1]);

  CAN_filterConfig();
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  	  {
	  /* Start Error */
	  Error_Handler();
  	  	 }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  	  {
  // Notification Error
  Error_Handler();
  	  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
	  startBit();
	  retval = detect();
	  if(retval)
	  {
	  acquireData(DHTdataBuff);

	// uint32_t str[32];
	  temp = (float)(DHTdataBuff[2] << 8 | DHTdataBuff[3])/10.0;
	  printf("Temp:%f\n",temp);

	//  uint32_t count = sprintf(str, "ADC= %f\r\n", temp);
	  //	HAL_UART_Transmit(&huart2, (uint8_t*)str, count, HAL_MAX_DELAY);

	  TxData[0] = DHTdataBuff[2];
	  TxData[1] = DHTdataBuff[3];

	  }
	  else
	  {
	  printf("Temp:%f\n",99.99);
	  TxData[0] = 0x00;
	  TxData[1] = 0x00;
	  }
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
	  HAL_Delay(1000);

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);


	  //read current register
	  retval = changeRegPtr(&hi2c1,slave_addr,&(reg_ptr.current));
	  printf("Change address to current_reg:%u\n",retval);
	  retval = getBytes(&hi2c1 , slave_addr, rxBuff , 2);
	  printf("Read success:%u\n",retval);
	  printf("B1:%x\t B2:%x\n",rxBuff[0],rxBuff[1]);
	  current_regV = (rxBuff[0]<<8) + (rxBuff[1]);
	  current_mA = current_regV*currentLSB;
	  printf("Current Value:%f\n",current_mA);
	  TxData[2] = rxBuff[0];
	  TxData[3] = rxBuff[1];

	  //read busVoltage register
	  retval = changeRegPtr(&hi2c1,slave_addr,&(reg_ptr.bus_voltage));
	  printf("Change address to busVoltage_reg:%u\n",retval);
	  retval = getBytes(&hi2c1 , slave_addr, rxBuff , 2);
	  printf("Read success:%u\n",retval);
	  printf("B1:%x\t B2:%x\n",rxBuff[0],rxBuff[1]);
	  busV_regV = ((rxBuff[0]<<8) + (rxBuff[1]));
	  busV_regV = busV_regV >> 3;
	  bus_mV = busV_regV*busVLSB;
	  bus_V = bus_mV/1000;
	  printf("busVolt Value:%f\n",bus_V);
	  TxData[2] = rxBuff[0];
	  TxData[3] = rxBuff[1];

	  //read shuntVoltage register
	  retval = changeRegPtr(&hi2c1,slave_addr,&(reg_ptr.shunt_voltage));
	  printf("Change address to shuntVoltage:%u\n",retval);
	  retval = getBytes(&hi2c1 , slave_addr, rxBuff , 2);
	  printf("Read success:%u\n",retval);
	  printf("B1:%x\t B2:%x\n",rxBuff[0],rxBuff[1]);
	  shunt_regV = (rxBuff[0]<<8) + (rxBuff[1]);
	  shunt_mV = shunt_regV*shuntVLSB;
	  printf("shuntVolt Value:%f\n",shunt_mV);

	  //read busWattage register
	  retval = changeRegPtr(&hi2c1,slave_addr,&(reg_ptr.power));
	  printf("Change address to busWattage:%u\n",retval);
	  retval = getBytes(&hi2c1 , slave_addr, rxBuff , 2);
	  printf("Read success:%u\n",retval);

	  printf("B1:%x\t B2:%x\n",rxBuff[0],rxBuff[1]);
	  busP_regV = (rxBuff[0]<<8) + (rxBuff[1]);
	  bus_mW = busP_regV*powerLSB;
	  printf("busWatt Value:%f\n",bus_mW);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
	  HAL_Delay(2000);
	  TxHeader.StdId = 0x123;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.DLC = 6;
	  TxHeader.TransmitGlobalTime = DISABLE;
	  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  {
	  /* Transmission request Error */
	  Error_Handler();
	  }
	  for(int i=0;i<10;HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12),HAL_Delay(200),i++);
	  HAL_Delay(180000);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void delayUS(uint16_t delay)
{
	htim6.Instance->CNT = 0;
	while(htim6.Instance->CNT < delay);
}


void sensorPinOUT()
{
gpioINIT.Pin = DHT_pin;
gpioINIT.Mode = GPIO_MODE_OUTPUT_PP;
HAL_GPIO_Init(DHT_port, &gpioINIT);
}

void sensorPinIN()
{
gpioINIT.Pin = DHT_pin;
gpioINIT.Mode = GPIO_MODE_INPUT;
gpioINIT.Pull = GPIO_PULLUP;
HAL_GPIO_Init(DHT_port, &gpioINIT);
}

void startBit()
{
	sensorPinOUT();
	HAL_GPIO_WritePin(DHT_port, DHT_pin, RESET);
	delayUS(18000);
	sensorPinIN();
}

uint8_t detect()
{
	uint8_t presence = 0;
	delayUS(40);
	TWITCH
	if(!HAL_GPIO_ReadPin(DHT_port, DHT_pin))
	{
	presence = 1;
	HAL_GPIO_WritePin(DHT_DETECT_port, DHT_DETECT_pin, SET);
	}
	else
	{
	HAL_GPIO_WritePin(DHT_DETECT_port, DHT_DETECT_pin, RESET);
	return presence;
	}
	while(HAL_GPIO_ReadPin(DHT_port, DHT_pin) != GPIO_PIN_SET);
	TWITCH
	while(HAL_GPIO_ReadPin(DHT_port, DHT_pin) != GPIO_PIN_RESET);
	TWITCH
	return presence;
}

void acquireData(uint8_t * dataBuff)
{
	uint8_t temp;
	for(int i=0 ; i<5;i++)
	{
	temp = 0;
	for(int j=0;j<8;j++)
	{
	while(!HAL_GPIO_ReadPin(DHT_port, DHT_pin));
	delayUS(45);
	TWITCH
	if(HAL_GPIO_ReadPin(DHT_port, DHT_pin) == GPIO_PIN_RESET)
	{
	TWITCH
	//bit is '0'
	temp = temp & ~(1<<(7-j));
	}
	else
	{
	TWITCH
	//bit is '1'
	temp = temp | 1<<(7-j);
	delayUS(30);
	}
	}
	*(dataBuff+i) = temp;
	while(HAL_GPIO_ReadPin(DHT_port, DHT_pin) != GPIO_PIN_RESET);
	}
}

uint8_t changeRegPtr(I2C_HandleTypeDef * hi2c, uint16_t i2c_add, uint8_t * reg_ptr_val)
{
	if(!HAL_I2C_Master_Transmit(hi2c, i2c_add<<1 , reg_ptr_val, 1, 1000))
	return HAL_OK;
	else
	return HAL_ERROR;
	}
uint8_t getBytes(I2C_HandleTypeDef * hi2c , uint16_t i2c_add, uint8_t * buffPtr , uint8_t count)
{
	if(!HAL_I2C_Master_Receive(hi2c, i2c_add<<1, buffPtr, count, 10000))
	return HAL_OK;
	else
	return HAL_ERROR;
}
uint8_t putBytes(I2C_HandleTypeDef * hi2c , uint16_t i2c_add, uint8_t * buffPtr , uint8_t count)
{
	if(!HAL_I2C_Master_Transmit(hi2c, i2c_add<<1, buffPtr, count, 10000))
	return HAL_OK;
	else
	return HAL_ERROR;
}


void CAN_filterConfig(void)
{
	/*##- Setup the Filter to receive ANY MSG ID #####################################*/
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* Get RX message from Fifo0 as message is Pending in Fifo to be Read */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
	/* Reception Error */
	Error_Handler();
	}
	/* Display LEDx */
	if ((RxHeader.StdId == 0x123) && (RxHeader.IDE == CAN_ID_STD))
	{
	for(int i=0;i<10;HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13),HAL_Delay(100),i++);
	}


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
