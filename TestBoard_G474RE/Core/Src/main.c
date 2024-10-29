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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pMMG.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _pMMG_pressure_t {
	double pMMG1_press;
	double pMMG2_press;
	double pMMG3_press;
	double pMMG4_press;
	double pMMG5_press;
	double pMMG6_press;
	double pMMG7_press;
	double pMMG8_press;
} pMMG_pressure_t;

typedef struct _pMMG_temperature_t {
	double pMMG1_temp;
	double pMMG2_temp;
	double pMMG3_temp;
	double pMMG4_temp;
	double pMMG5_temp;
	double pMMG6_temp;
	double pMMG7_temp;
	double pMMG8_temp;
} pMMG_temperature_t;

typedef struct _pMMG_err_t {
	uint32_t err1;
	uint32_t err2;
	uint32_t err3;
	uint32_t err4;
	uint32_t err5;
	uint32_t err6;
	uint32_t err7;
	uint32_t err8;
	uint32_t totalErr;
} pMMG_err_t;

typedef struct _pMMG_t {
	pMMG_pressure_t pMMG_press;
	pMMG_temperature_t pMMG_temp;
	pMMG_err_t pMMG_err;
} pMMG_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* SPI1 port */
pMMG_Obj_t pMMGObj1;
pMMG_Obj_t pMMGObj2;
pMMG_Obj_t pMMGObj3;
uint8_t state1 = 0;
uint8_t state2 = 0;
uint8_t state3 = 0;

/* SPI2 port */
pMMG_Obj_t pMMGObj4;
pMMG_Obj_t pMMGObj5;
pMMG_Obj_t pMMGObj6;
uint8_t state4 = 0;
uint8_t state5 = 0;
uint8_t state6 = 0;

/* SPI3 port */
pMMG_Obj_t pMMGObj7;
pMMG_Obj_t pMMGObj8;
uint8_t state7 = 0;
uint8_t state8 = 0;

/* Total pMMG Obj */
pMMG_t totalpMMG;

/* FSR DMA memory */
uint16_t FSRvalue[2] = {0};		// 0: PA0, 1: PA1
uint16_t FSR_L = 0; 			// 0: PA0
uint16_t FSR_R = 0;				// 1: PA1

float start = 0;
float codeTime = 0;			// usec

float totalCodeTime = 0; 	// sec

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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  /* Initialize pMMG */
//  state1 = pMMG_Init(&pMMGObj1, &hspi1, GPIOC, GPIO_PIN_8);
//  state4 = pMMG_Init(&pMMGObj4, &hspi2, GPIOA, GPIO_PIN_12);
//  state7 = pMMG_Init(&pMMGObj7, &hspi3, GPIOB, GPIO_PIN_11);

//  state2 = pMMG_Init(&pMMGObj2, &hspi1, GPIOC, GPIO_PIN_6);
//  state5 = pMMG_Init(&pMMGObj5, &hspi2, GPIOA, GPIO_PIN_11);
//  state8 = pMMG_Init(&pMMGObj8, &hspi3, GPIOB, GPIO_PIN_2);


  /* ------------------ Extended G474RE Board Version ------------------ */
  state1 = pMMG_Init(&pMMGObj1, &hspi1, GPIOC, GPIO_PIN_8);
  state2 = pMMG_Init(&pMMGObj2, &hspi1, GPIOC, GPIO_PIN_6);
  state3 = pMMG_Init(&pMMGObj3, &hspi1, GPIOC, GPIO_PIN_5);
  state4 = pMMG_Init(&pMMGObj4, &hspi2, GPIOA, GPIO_PIN_12);
  state5 = pMMG_Init(&pMMGObj5, &hspi2, GPIOA, GPIO_PIN_11);
  state6 = pMMG_Init(&pMMGObj6, &hspi2, GPIOB, GPIO_PIN_12);
  state7 = pMMG_Init(&pMMGObj7, &hspi3, GPIOB, GPIO_PIN_11);
  state8 = pMMG_Init(&pMMGObj8, &hspi3, GPIOB, GPIO_PIN_2);

  /* ---------------------- Start FSR ADC read ------------------------- */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)FSRvalue, 2);

  /* If you use Timer Interrupt */
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		DWT->CYCCNT = 0;
		start = DWT->CYCCNT / 170;


		/* (4) 8-pMMG measurement with Extended G474RE Board by 3-phases [ 4.22ms = (1.220ms + 3*70us)*2 + (1.220ms + 2*70us) ] */
		pMMG_Update_multiple_3(&pMMGObj1, &pMMGObj4, &pMMGObj7);
		pMMG_Update_multiple_3(&pMMGObj2, &pMMGObj5, &pMMGObj8);
		pMMG_Update_multiple_2(&pMMGObj3, &pMMGObj6);

		// Mapping //
		totalpMMG.pMMG_press.pMMG1_press = pMMGObj1.pMMGData.pressureKPa;
		totalpMMG.pMMG_press.pMMG2_press = pMMGObj2.pMMGData.pressureKPa;
		totalpMMG.pMMG_press.pMMG3_press = pMMGObj3.pMMGData.pressureKPa;
		totalpMMG.pMMG_press.pMMG4_press = pMMGObj4.pMMGData.pressureKPa;
		totalpMMG.pMMG_press.pMMG5_press = pMMGObj5.pMMGData.pressureKPa;
		totalpMMG.pMMG_press.pMMG6_press = pMMGObj6.pMMGData.pressureKPa;
		totalpMMG.pMMG_press.pMMG7_press = pMMGObj7.pMMGData.pressureKPa;
		totalpMMG.pMMG_press.pMMG8_press = pMMGObj8.pMMGData.pressureKPa;

		totalpMMG.pMMG_temp.pMMG1_temp = pMMGObj1.pMMGData.temperatureC;
		totalpMMG.pMMG_temp.pMMG2_temp = pMMGObj2.pMMGData.temperatureC;
		totalpMMG.pMMG_temp.pMMG3_temp = pMMGObj3.pMMGData.temperatureC;
		totalpMMG.pMMG_temp.pMMG4_temp = pMMGObj4.pMMGData.temperatureC;
		totalpMMG.pMMG_temp.pMMG5_temp = pMMGObj5.pMMGData.temperatureC;
		totalpMMG.pMMG_temp.pMMG6_temp = pMMGObj6.pMMGData.temperatureC;
		totalpMMG.pMMG_temp.pMMG7_temp = pMMGObj7.pMMGData.temperatureC;
		totalpMMG.pMMG_temp.pMMG8_temp = pMMGObj8.pMMGData.temperatureC;
		///////////////////////////////////////////////////////////////////////////////


		codeTime = DWT->CYCCNT / 170 - start;
		totalCodeTime += (float)codeTime / 1000000;


		if (pMMGObj1.pMMGData.pressureKPa > 150 || pMMGObj1.pMMGData.pressureKPa < 90) {
			totalpMMG.pMMG_err.err1++;
			totalpMMG.pMMG_err.totalErr++;
		}
		if (pMMGObj2.pMMGData.pressureKPa > 150 || pMMGObj2.pMMGData.pressureKPa < 90) {
			totalpMMG.pMMG_err.err2++;
			totalpMMG.pMMG_err.totalErr++;
		}
		if (pMMGObj3.pMMGData.pressureKPa > 150 || pMMGObj3.pMMGData.pressureKPa < 90) {
			totalpMMG.pMMG_err.err3++;
			totalpMMG.pMMG_err.totalErr++;
		}
		if (pMMGObj4.pMMGData.pressureKPa > 150 || pMMGObj4.pMMGData.pressureKPa < 90) {
			totalpMMG.pMMG_err.err4++;
			totalpMMG.pMMG_err.totalErr++;
		}
		if (pMMGObj5.pMMGData.pressureKPa > 150 || pMMGObj5.pMMGData.pressureKPa < 90) {
			totalpMMG.pMMG_err.err5++;
			totalpMMG.pMMG_err.totalErr++;
		}
		if (pMMGObj6.pMMGData.pressureKPa > 150 || pMMGObj6.pMMGData.pressureKPa < 90) {
			totalpMMG.pMMG_err.err6++;
			totalpMMG.pMMG_err.totalErr++;
		}
		if (pMMGObj7.pMMGData.pressureKPa > 150 || pMMGObj7.pMMGData.pressureKPa < 90) {
			totalpMMG.pMMG_err.err7++;
			totalpMMG.pMMG_err.totalErr++;
		}
		if (pMMGObj8.pMMGData.pressureKPa > 150 || pMMGObj8.pMMGData.pressureKPa < 90) {
			totalpMMG.pMMG_err.err8++;
			totalpMMG.pMMG_err.totalErr++;
		}


		/* Check FSR values */
		FSR_L = FSRvalue[0];
		FSR_R = FSRvalue[1];
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
