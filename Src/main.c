/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "base/bsp/bsp_led.h"
#include "base/bsp/bsp_adc.h"

#include "base/imu/driver/ist8310driver.h"
#include "base/imu/driver/BMI088driver.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BoardLed led;
Board_Monitor board_monitor;

fp32 mag[3];
fp32 gyro[3], accel[3], temp;

long time = 0;
long mag_cnt = 0;
float mag_freq = 0;
long accel_int_cnt = 0;
float accel_int_freq = 0;
long gyro_int_cnt = 0;
float gyro_int_freq = 0;

uint8_t sumcheck(uint8_t* buf, uint8_t len);
struct sensor_data_t {
  float x;
  float y;
  float z;
}__attribute__((packed));

struct imu_raw_t{
  uint8_t sign;
  sensor_data_t acc;
  sensor_data_t gyro;
  //float temp;
  uint8_t check;
}__attribute__((packed));

imu_raw_t txdata;
size_t data_size = sizeof(txdata);

uint8_t tx_freq_cnt = 0;
float tx_freq = 0.f;
float lt = 0;
float nt = 0;
float dt;
float avg_dt = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == IST8310_DRDY_Pin)
    {
      mag_cnt++;
			
			static long ltm = 0;
			long ntm = time;
			if (ntm - ltm >= 1000) {
				mag_freq = (float) mag_cnt / (ntm - ltm) * 1000.0f;
				mag_cnt = 0;
				ltm = ntm;
			}
      ist8310_read_mag(mag);
    }

    if (GPIO_Pin == INT1_GRYO_Pin) {
      static long ltg = 0;
      long ntg = time;

      gyro_int_cnt++;

      if (ntg - ltg >= 1000) {
        gyro_int_freq = (float)gyro_int_cnt / (float)(ntg - ltg) * 1000.f;
        gyro_int_cnt = 0;
        ltg = ntg;
      }    
    }
		
		
    if (GPIO_Pin == INT1_ACCEL_Pin) {
      static long lta = 0;
      long nta = time;

      accel_int_cnt++;

      if (nta - lta >= 1000) {
        accel_int_freq = (float)accel_int_cnt / (float)(nta - lta) * 1000.f;
        accel_int_cnt = 0;
        lta = nta;
      }
			BMI088_read(gyro, accel, &temp);
			memcpy(&txdata.acc, accel, 12);
			memcpy(&txdata.gyro, gyro, 12);
			//txdata.temp = temp;
			static int tx_cnt = 0;
			tx_cnt++;
			if (tx_cnt % 2 == 0) {
				txdata.check = sumcheck((uint8_t*)&txdata, sizeof(txdata) - 2);
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) &txdata, sizeof(txdata));
				tx_cnt = 0;
				tx_freq_cnt ++;
				nt = time;
				dt += nt - lt;
				lt = nt;
			}
		}


}
uint8_t init_bmi088 = 0;

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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim8);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

		//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  //receive interrupt
		
    //__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);  //idle interrupt
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //idle interrupt
    //__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);  //receive interrupt
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  //idle interrupt

	txdata.sign = 0X23;
  led.init();
	led.setColor(0xFF, 0xFF, 0xff);

	led.setColor(0x00, 0x00, 0xff);
  led.setModeBreath(1000);
  
  ist8310_init();
	board_monitor.init_vrefint_reciprocal();
	HAL_Delay(1000);

	while (!init_bmi088) {
		init_bmi088 = BMI088_init();
	}

  led.setColor(0x00, 0xff, 0x00);
	led.setModeBlink(1, 100, 300);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		uint8_t data[4] = {0x73, 0x74, 0x75, 0x76};


  while (1)
  {
//				HAL_UART_Transmit(&huart1, data, 4, 100);
//        HAL_Delay(100);
//        HAL_UART_Transmit(&huart6, data, 4, 100);
//        HAL_Delay(100);
		
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
long cnt = 0;
int ist_rdy_cnt = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim == &htim1) {
    time++;
		cnt ++;
		//HAL_UART_Transmit_IT(&huart1, (uint8_t*) &txdata, sizeof(txdata));
    if (cnt % 10 == 0) {
      board_monitor.sampleBatteryVoltage();
      board_monitor.sampleTemperate();
    }
		
		if (time % 1000 == 0) {
			tx_freq = tx_freq_cnt;
			tx_freq_cnt = 0;
			avg_dt = dt / (float)tx_freq;
			dt = 0;
		}

		
//		if (cnt == 10000) {
//			led.setModeBreath(1000);
//			led.setColor(0xff, 0x00, 0x00);
//		} else if (cnt == 20000) {
//			led.setColor(0x00, 0x00, 0xff);
//			led.setModeBlink(5, 50, 50);
//		}	else if (cnt < 5000) {
//			led.setColor(0xff, 0xff, 0xff);
//			led.setModeOn();
//		} else if (cnt < 10000) {
//			led.setModeOff();
//		} else if (cnt >= 30000) {
//			cnt = 0;
//		}
		
		
//		if (HAL_GPIO_ReadPin(IST8310_DRDY_GPIO_Port, IST8310_DRDY_Pin) == GPIO_PIN_RESET) {
//			ist_rdy_cnt++;
//		}
		
		led.handle();
	}
}

uint8_t sumcheck(uint8_t* buf, uint8_t len) {
  uint8_t i = 0;
  uint8_t sum = 0;
  for (i = 0; i < len; ++i) {
    sum += *(buf + i);
  }
  return sum;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
