/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "crc.h"
#include "usart.h"
#include "gpio.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "custom_motion_sensors.h"
#include "custom_motion_sensors_ex.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small msg_info (option LD Linker->Libraries->Small msg_info
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

#define BSP_RET_CHECK(x)	do { if (x != BSP_ERROR_NONE)	Error_Handler(); } while(0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
LSM6DSL_Object_t lsm6dsl_obj;
CUSTOM_MOTION_SENSOR_Axes_t AccData, GyrData, MagData;
uint8_t AccGyrReadRequest, MagReadRequest;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MX_MEMS_Init(void);
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
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */
  MX_MEMS_Init();
  printf("リンクスタート！\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  int32_t ret_32;
//	  uint8_t ret_u8;
////	  ret = CUSTOM_MOTION_SENSOR_GetAxes(CUSTOM_LSM6DSL_0, MOTION_GYRO, &GyrData);
////	  ret = CUSTOM_MOTION_SENSOR_GetAxes(CUSTOM_LSM6DSL_0, MOTION_ACCELERO, &AccData);
//	  HAL_Delay(100);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_STATUS_REG, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_MASTER_CONFIG, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_FIFO_CTRL1, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_FIFO_CTRL2, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_FIFO_CTRL3, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_FIFO_CTRL4, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_FIFO_CTRL5, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_INT1_CTRL, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_CTRL1_XL, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_CTRL2_G, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_CTRL7_G, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_FIFO_STATUS1, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_FIFO_STATUS2, &ret_u8);
//	  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSL_0, LSM6DSL_FIFO_STATUS3, &ret_u8);
//	  ret_32 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
	  if(AccGyrReadRequest)
	  {
		  for(uint8_t i=0; i<10; i++)
		  {
			  CUSTOM_MOTION_SENSOR_FIFO_Get_Axis(CUSTOM_LSM6DSL_0, MOTION_GYRO, &GyrData.x);
			  CUSTOM_MOTION_SENSOR_FIFO_Get_Axis(CUSTOM_LSM6DSL_0, MOTION_GYRO, &GyrData.y);
			  CUSTOM_MOTION_SENSOR_FIFO_Get_Axis(CUSTOM_LSM6DSL_0, MOTION_GYRO, &GyrData.z);
			  CUSTOM_MOTION_SENSOR_FIFO_Get_Axis(CUSTOM_LSM6DSL_0, MOTION_ACCELERO, &AccData.x);
			  CUSTOM_MOTION_SENSOR_FIFO_Get_Axis(CUSTOM_LSM6DSL_0, MOTION_ACCELERO, &AccData.y);
			  CUSTOM_MOTION_SENSOR_FIFO_Get_Axis(CUSTOM_LSM6DSL_0, MOTION_ACCELERO, &AccData.z);
		  }
		  printf("AccGyr data fetched.\r\n");
//		  HAL_UART_Transmit(&huart1, (uint8_t*)"A\r\n", 3, HAL_MAX_DELAY);
		  AccGyrReadRequest = 0U;
	  }
//	  if(MagReadRequest)
//	  {
//		  CUSTOM_MOTION_SENSOR_GetAxes(CUSTOM_LIS3MDL_0, MOTION_MAGNETO, &MagData);
//		  MagReadRequest = 0U;
//	  }
    /* USER CODE END WHILE */

  MX_X_CUBE_AI_Process();
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  while (HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY))
  {
    ;
  }
  return ch;
}

GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY))
  {
    ;
  }
  return ch;
}

__attribute__((weak)) int _write(int file, char* ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

void MX_MEMS_Init(void)
{
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_Init(CUSTOM_LSM6DSL_0, MOTION_ACCELERO));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_Init(CUSTOM_LSM6DSL_0, MOTION_GYRO));
//	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_FIFO_Set_Mode(CUSTOM_LSM6DSL_0, LSM6DSL_BYPASS_MODE));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_SetOutputDataRate(CUSTOM_LSM6DSL_0, MOTION_ACCELERO, 104.0f));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_SetOutputDataRate(CUSTOM_LSM6DSL_0, MOTION_GYRO, 104.0f));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_SetFullScale(CUSTOM_LSM6DSL_0, MOTION_ACCELERO, 4));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_SetFullScale(CUSTOM_LSM6DSL_0, MOTION_GYRO, 1000));
//	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_Write_Register(CUSTOM_LSM6DSL_0, LSM6DSL_MASTER_CONFIG, 0x0));
//	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_Write_Register(CUSTOM_LSM6DSL_0, LSM6DSL_FIFO_CTRL2, 0x0));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_FIFO_Set_ODR_Value(CUSTOM_LSM6DSL_0, 104.0f));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_FIFO_Set_Decimation(CUSTOM_LSM6DSL_0, MOTION_ACCELERO, LSM6DSL_FIFO_XL_NO_DEC));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_FIFO_Set_Decimation(CUSTOM_LSM6DSL_0, MOTION_GYRO, LSM6DSL_FIFO_GY_NO_DEC));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_FIFO_Set_Watermark_Level(CUSTOM_LSM6DSL_0, 90));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_Write_Register(CUSTOM_LSM6DSL_0, LSM6DSL_INT1_CTRL, 0x8));
	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_FIFO_Set_Mode(CUSTOM_LSM6DSL_0, LSM6DSL_STREAM_MODE));

//	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_Init(CUSTOM_LIS3MDL_0, MOTION_MAGNETO));
//	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_SetOutputDataRate(CUSTOM_LIS3MDL_0, MOTION_MAGNETO, 80.0f));
//	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_SetFullScale(CUSTOM_LIS3MDL_0, MOTION_MAGNETO, 4));
//	BSP_RET_CHECK(CUSTOM_MOTION_SENSOR_Write_Register(CUSTOM_LIS3MDL_0, LIS3MDL_CTRL_REG3, 0x0));

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case GPIO_PIN_8:
		MagReadRequest = 1U;
	case GPIO_PIN_11:
		AccGyrReadRequest = 1U;
		break;

	default:
		break;
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
