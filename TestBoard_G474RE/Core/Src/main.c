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
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SpO2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Activate one example to test the code */
//#define EXAMPLE_1			// [Example 1] Get RED/IR/GREEN LEDs
//#define EXAMPLE_2			// [Example 2] Presence Sensing (Detection of object)
//#define EXAMPLE_3			// [Example 3] Temperature Sensing
//#define EXAMPLE_4			// [Example 4] Heart Rate Detection
#define EXAMPLE_5			// [Example 5] FIFO Readings
//#define EXAMPLE_6
//#define EXAMPLE_7
//#define EXAMPLE_8
//#define EXAMPLE_9


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SPO2_Obj_t spo2Obj1;
uint8_t state1 = 0;

float start = 0;
float codeTime = 0;

/*------------------------------ For Test the Examples ------------------------------*/
typedef struct _Ex_Obj_t {
	/*---------- For [Ex1] ----------*/
	uint32_t RED;
	uint32_t IR;
	uint32_t GREEN;
	/*-------------------------------*/


	/*---------- For [Ex2] ----------*/
	uint32_t unblockedValue;	// offset of IR
	uint32_t samplesTaken;
	uint8_t sampleFreq;			// Hz
	int32_t currentDelta;		// Current value of IR (If it detects something, then this value should be high)
	uint8_t detectObject;		// 1: detect object, 0: does NOT detect anything
	/*-------------------------------*/


	/*---------- For [Ex3] ----------*/
	float temperatureC;
	float temperatureF;
	/*-------------------------------*/


	/*---------- For [Ex4] ----------*/
	uint8_t fingerIsOn;
	uint8_t heartRates[4];		// You can change the buffer size
	uint8_t rateSpot;
	uint8_t rateSize;
	uint32_t lastBeat;

	uint8_t BPM;
	uint8_t BPM_avg;
	/*-------------------------------*/


	/*---------- For [Ex5] ----------*/

	/*-------------------------------*/

} Ex_Obj_t;

Ex_Obj_t exampleObj;
/*-----------------------------------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_LPUART1_UART_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  /*----------------------------- Initialize SPO2 sensor ------------------------------*/
  state1 = SPO2_Init(&spo2Obj1, &hi2c3);
#ifdef EXAMPLE_1
  SPO2_Setup(&spo2Obj1, SPO2_SAMPLEAVG_4, SPO2_LEDMODE_MULTILED, SPO2_ADCRANGE_12bit, SPO2_SAMPLERATE_400, SPO2_PULSEWIDTH_411, SPO2_CURRAMP_6p4);   // Change the parameters
#endif
#ifdef EXAMPLE_2
  SPO2_Setup(&spo2Obj1, SPO2_SAMPLEAVG_4, SPO2_LEDMODE_REDIRONLY, SPO2_ADCRANGE_11bit, SPO2_SAMPLERATE_400, SPO2_PULSEWIDTH_411, SPO2_CURRAMP_50);   // Use maximum current(50mA) to detect up to 18 inches
  SPO2_SetPulseAmpRed(&spo2Obj1, 0);		// Turn off RED LED
  SPO2_SetPulseAmpGreen(&spo2Obj1, 0);		// Turn off GREEN LED (Not necessary for MAX30102 since MAX30102 does NOT have GREEN LED)

  /* Check averaged initial unblocked value of IR (for offset) */
  exampleObj.unblockedValue = 0;
  for (uint8_t i = 0; i < 32; i++) {
	  exampleObj.unblockedValue += SPO2_GetIR(&spo2Obj1);
  }
  exampleObj.unblockedValue /= 32;
#endif
#ifdef EXAMPLE_3
  /* Use zero current(0mA) because we don't have to use any LED for temperature sensing -> Turn off ALL LEDs to reduce heat from them (Other settings do not matter) */
  SPO2_Setup(&spo2Obj1, SPO2_SAMPLEAVG_4, SPO2_LEDMODE_REDONLY, SPO2_ADCRANGE_11bit, SPO2_SAMPLERATE_50, SPO2_PULSEWIDTH_411, SPO2_CURRAMP_0);
  SPO2_EnableDIETEMPRDY(&spo2Obj1);
#endif
#ifdef EXAMPLE_4
  SPO2_Setup(&spo2Obj1, SPO2_SAMPLEAVG_4, SPO2_LEDMODE_MULTILED, SPO2_ADCRANGE_12bit, SPO2_SAMPLERATE_400, SPO2_PULSEWIDTH_411, SPO2_CURRAMP_6p4);
  SPO2_SetPulseAmpRed(&spo2Obj1, 0x0A);		// Turn on RED LED to low to indicate the sensor is running
  SPO2_SetPulseAmpGreen(&spo2Obj1, 0x00);	// Turn off GREEN LED
#endif
  /*-----------------------------------------------------------------------------------*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef EXAMPLE_1
	  exampleObj.RED = SPO2_GetRED(&spo2Obj1);
	  exampleObj.IR = SPO2_GetIR(&spo2Obj1);
	  exampleObj.GREEN = SPO2_GetGREEN(&spo2Obj1);
#endif
#ifdef EXAMPLE_2
	  DWT->CYCCNT = 0;
	  start = DWT->CYCCNT / 170;

	  exampleObj.IR = SPO2_GetIR(&spo2Obj1);
	  exampleObj.sampleFreq = 1 / ( (DWT->CYCCNT/170 - start)/1000000 );

	  exampleObj.currentDelta = exampleObj.IR - exampleObj.unblockedValue;

	  if (exampleObj.currentDelta > 100) {
		  exampleObj.detectObject = 1;
	  }
	  else {
		  exampleObj.detectObject = 0;
	  }
#endif
#ifdef EXAMPLE_3
	  SPO2_ReadTemperature(&spo2Obj1);
	  exampleObj.temperatureC = spo2Obj1.temperatureC;
	  exampleObj.temperatureF = spo2Obj1.temperatureF;
#endif
#ifdef EXAMPLE_4
	  /* Set the rate size */
	  exampleObj.rateSize = 4;

	  exampleObj.IR = SPO2_GetIR(&spo2Obj1);
	  if (exampleObj.IR < 50000) {
		  exampleObj.fingerIsOn = 0;	// Finger is NOT detected
		  exampleObj.BPM = 0;
		  exampleObj.BPM_avg = 0;
	  }
	  else {
		  exampleObj.fingerIsOn = 1;	// Finger is detected
	  }

	  if (SPO2_CheckForBeat(&spo2Obj1, exampleObj.IR) == 1 && exampleObj.fingerIsOn == 1) {
		  uint32_t delta = DWT->CYCCNT / 170 - exampleObj.lastBeat;
		  exampleObj.lastBeat = DWT->CYCCNT / 170;

		  exampleObj.BPM = 60 / (delta/1000000.0);

		  /* Averaging (ex)4-sample */
		  if ((exampleObj.BPM < 255) & (exampleObj.BPM > 20)) {
			  exampleObj.heartRates[exampleObj.rateSpot++] = exampleObj.BPM;
			  exampleObj.rateSpot %= exampleObj.rateSize;

			  uint16_t tempBPM = 0;
			  for (uint8_t i = 0; i < exampleObj.rateSize; i++) {
				  tempBPM += exampleObj.heartRates[i];
				  if (i == exampleObj.rateSize-1) {
					  exampleObj.BPM_avg = tempBPM / exampleObj.rateSize;
				  }
			  }
		  }
	  }
#endif

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
