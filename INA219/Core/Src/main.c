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
#define INA219_SHUNT_RESISTANCE 0.1  // Shunt resistance value in ohms
#define FIXED_POINT_SCALE 1000       // Fixed-point scale for representation
#define INA219_ADDRESS 0x40
#define INA219_CONFIG_REG 0x00
#define INA219_SHUNT_VOLTAGE_REG 0x01
#define INA219_BUS_VOLTAGE_REG 0x02

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void INA219_Init();
int INA219_ReadCurrent();
float INA219_ReadBusVoltage();
float INA219_ReadShuntVoltage();
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  INA219_Init();
  /* USER CODE END 2 */
  void UART_Gets(char uart_buffer[]) {
    	char ch;
    	int i=0;
    	do {
    		HAL_UART_Receive(&huart2, (uint8_t*)&ch, sizeof(ch), HAL_MAX_DELAY);
    		uart_buffer[i++] = ch;
    	} while(ch != '\r');
    	uart_buffer[i++] = '\n';
    	uart_buffer[i++] = '\0';
    }
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  int current = INA219_ReadCurrent();
	     float busVoltage = INA219_ReadBusVoltage();
	     float shuntVoltage = INA219_ReadShuntVoltage();

	     // Calculate power and voltage
	     float power = (current / FIXED_POINT_SCALE) * busVoltage;
	     float voltage = busVoltage + (shuntVoltage * INA219_SHUNT_RESISTANCE);

	     char uart_buffer[100];
	     sprintf(uart_buffer, "Current: %d mA, Power: %.3f mW, Voltage: %.3f V\r\n", current, power, voltage);

	     HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

	     HAL_Delay(1000);  // Delay for 1 second
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void INA219_Init()
{
  // Configure INA219
  uint16_t config = 0x399F; // 32V range, 1A shunt, continuous conversion
  HAL_I2C_Mem_Write(&hi2c1, INA219_ADDRESS << 1, INA219_CONFIG_REG, 1, (uint8_t*)&config, 2, HAL_MAX_DELAY);
}

int INA219_ReadCurrent()
{
  // Read shunt voltage
  uint16_t shunt_voltage_raw;
  HAL_I2C_Mem_Read(&hi2c1, INA219_ADDRESS << 1, INA219_SHUNT_VOLTAGE_REG, 1, (uint8_t*)&shunt_voltage_raw, 2, HAL_MAX_DELAY);

  int16_t shunt_voltage = (int16_t)shunt_voltage_raw;
  shunt_voltage = (shunt_voltage >> 3); // 12-bit resolution

  // Read bus voltage
  uint16_t bus_voltage_raw;
  HAL_I2C_Mem_Read(&hi2c1, INA219_ADDRESS << 1, INA219_BUS_VOLTAGE_REG, 1, (uint8_t*)&bus_voltage_raw, 2, HAL_MAX_DELAY);

  int16_t bus_voltage = (int16_t)(bus_voltage_raw >> 3); // 12-bit resolution

  // Calculate current
  int current = (shunt_voltage * 100) / 10; // Adjust scaling factor
  return current;
}


float INA219_ReadBusVoltage()
{
  // Read bus voltage
  uint16_t bus_voltage_raw;
  HAL_I2C_Mem_Read(&hi2c1, INA219_ADDRESS << 1, INA219_BUS_VOLTAGE_REG, 1, (uint8_t*)&bus_voltage_raw, 2, HAL_MAX_DELAY);

  int16_t bus_voltage = (int16_t)(bus_voltage_raw >> 3); // 12-bit resolution

  return (float)bus_voltage * 4e-3; // LSB = 4mV
}

float INA219_ReadShuntVoltage()
{
  // Read shunt voltage
  uint16_t shunt_voltage_raw;
  HAL_I2C_Mem_Read(&hi2c1, INA219_ADDRESS << 1, INA219_SHUNT_VOLTAGE_REG, 1, (uint8_t*)&shunt_voltage_raw, 2, HAL_MAX_DELAY);

  int16_t shunt_voltage = (int16_t)shunt_voltage_raw;
  shunt_voltage = (shunt_voltage >> 3); // 12-bit resolution

  return (float)shunt_voltage * 10e-6; // LSB = 10uV
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
