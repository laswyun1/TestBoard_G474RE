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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct _MPU6050Data_t {
    float accX;
    float accY;
    float accZ;

    float temp;

    float gyrX;
    float gyrY;
    float gyrZ;

    float accXoffset;
    float accYoffset;
    float accZoffset;

    float gyrXoffset;
    float gyrYoffset;
    float gyrZoffset;
} MPU6050Data_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Device Address */
#define MPU6050_DEV_ADDR						0xD0U

/* Register Address */
#define MPU6050_PWR_MGMT_1						0x6BU
#define MPU6050_PWR_MGMT_2						0x6CU
#define MPU6050_GYRO_CONFIG						0x1BU
#define MPU6050_ACCEL_CONFIG					0x1CU

#define MPU6050_ACCEL_XOUT_H					0x3BU	// Information MSB
#define MPU6050_ACCEL_XOUT_L					0x3CU	// Information LSB
#define MPU6050_ACCEL_YOUT_H					0x3DU	// Information MSB
#define MPU6050_ACCEL_YOUT_L					0x3EU	// Information LSB
#define MPU6050_ACCEL_ZOUT_H					0x3FU	// Information MSB
#define MPU6050_ACCEL_ZOUT_L					0x40U	// Information LSB

#define MPU6050_TEMP_OUT_H						0x41U	// Information MSB
#define MPU6050_TEMP_OUT_L						0x42U	// Information LSB

#define MPU6050_GYRO_XOUT_H						0x43U	// Information MSB
#define MPU6050_GYRO_XOUT_L						0x44U	// Information LSB
#define MPU6050_GYRO_YOUT_H						0x45U	// Information MSB
#define MPU6050_GYRO_YOUT_L						0x46U	// Information LSB
#define MPU6050_GYRO_ZOUT_H						0x47U	// Information MSB
#define MPU6050_GYRO_ZOUT_L						0x48U	// Information LSB

/* Register Configuration Values */
#define MPU6050_PWR_MGMT_1_DATA				    0b00000000		// For 0x6B register
#define MPU6050_PWR_MGMT_2_DATA				    0b00000000		// For 0x6C register
#define MPU6050_GYR_CONFIG_250dps				0b00000000		// For 0x1B register
#define MPU6050_GYR_CONFIG_500dps				0b00001000		// For 0x1B register
#define MPU6050_GYR_CONFIG_1000dps				0b00010000		// For 0x1B register
#define MPU6050_GYR_CONFIG_2000dps				0b00011000		// For 0x1B register
#define MPU6050_ACC_CONFIG_2g				   	0b00000000		// For 0x1C register
#define MPU6050_ACC_CONFIG_4g				    0b00001000		// For 0x1C register
#define MPU6050_ACC_CONFIG_8g				    0b00010000		// For 0x1C register
#define MPU6050_ACC_CONFIG_16g				    0b00011000		// For 0x1C register

#define MPU6050_CONTROL_SIZE					1U
#define MPU6050_MEMADD_SIZE						1U
#define MPU6050_READDATA_SIZE					14
#define MPU6050_TOTAL_DATA_NUM					7


/* Data Scaling Factors */
#define MPU6050_ACCEL_SCALE_FACTOR_2g			16384.0f	// 16384 LSB/g
#define MPU6050_ACCEL_SCALE_FACTOR_4g    		8192.0f		// 8192  LSB/g
#define MPU6050_ACCEL_SCALE_FACTOR_8g			4096.0f		// 4096	 LSB/g
#define MPU6050_ACCEL_SCALE_FACTOR_16g			2048.0f		// 2048  LSB/g

#define MPU6050_GYRO_SCALE_FACTOR_250dps		131.0f		// 131  LSB/dps
#define MPU6050_GYRO_SCALE_FACTOR_500dps     	65.5f		// 65.5 LSB/dps
#define MPU6050_GYRO_SCALE_FACTOR_1000dps		32.8f		// 32.8 LSB/dps
#define MPU6050_GYRO_SCALE_FACTOR_2000dps		16.4f		// 16.4 LSB/dps

#define MPU6050_TEMP_SCALE_FACTOR     			340.0f		// Temperature(Celsius) = (Temp register value) / 340.0 + 36.53
#define MPU6050_TEMP_OFFSET      				36.53f



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rawData[14] = {0};
MPU6050Data_t IMUData;

uint16_t msTime = 0;
uint32_t codeTime = 0;
uint32_t totalElapsedTime = 0;
uint32_t breakRT = 0;

uint8_t cmd = 0;
uint8_t calibDone = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t Init6Axis(I2C_HandleTypeDef* hi2c);
uint8_t WriteReg(I2C_HandleTypeDef* hi2c);
uint8_t ReadReg(I2C_HandleTypeDef* hi2c);
uint8_t GetIMUValues(I2C_HandleTypeDef* hi2c, uint8_t CalibON);
void CalibrateIMU(I2C_HandleTypeDef* hi2c);

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
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;



  /* Initialize the IMU */
  Init6Axis(&hi2c1);

  /* Calibration with -y axis */
//  CalibrateIMU(&hi2c1);

  /* Used Calibrated Values */
  IMUData.accXoffset = 0.0542423353;
  IMUData.accYoffset = 0.0239739418;
  IMUData.accZoffset = -0.245294675;
  IMUData.gyrXoffset = -0.131917208;
  IMUData.gyrYoffset = -0.965673268;
  IMUData.gyrZoffset = 0.522041917;




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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim3.Instance){

		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


		/* Choose the Function you want to RUN */
		GetIMUValues(&hi2c1, 1);
		/* ----------------------------------- */

		if (msTime == 1000){
			msTime = 0;
			totalElapsedTime++;
		}

		/* Code End */
		codeTime = DWT->CYCCNT/170;
		msTime++;

		if (codeTime > 1000){
			breakRT++;
		}
	}
}


uint8_t Init6Axis(I2C_HandleTypeDef* hi2c)
{
	uint8_t state = 0;
	state = HAL_I2C_IsDeviceReady(hi2c, MPU6050_DEV_ADDR, 10, 10);

	if (state == 0){
		state = WriteReg(hi2c);
	}

	return state;
}

uint8_t WriteReg(I2C_HandleTypeDef* hi2c)
{
	uint8_t state = 0;

	uint8_t conf_1 = MPU6050_PWR_MGMT_1_DATA;
	state = HAL_I2C_Mem_Write(hi2c, MPU6050_DEV_ADDR, MPU6050_PWR_MGMT_1, MPU6050_MEMADD_SIZE, &conf_1, MPU6050_CONTROL_SIZE, 10);
	uint8_t conf_2 = MPU6050_PWR_MGMT_2_DATA;
	state = HAL_I2C_Mem_Write(hi2c, MPU6050_DEV_ADDR, MPU6050_PWR_MGMT_2, MPU6050_MEMADD_SIZE, &conf_2, MPU6050_CONTROL_SIZE, 10);
	uint8_t conf_3 = MPU6050_GYR_CONFIG_500dps;
	state = HAL_I2C_Mem_Write(hi2c, MPU6050_DEV_ADDR, MPU6050_GYRO_CONFIG, MPU6050_MEMADD_SIZE, &conf_3, MPU6050_CONTROL_SIZE, 10);
	uint8_t conf_4 = MPU6050_ACC_CONFIG_4g;
	state = HAL_I2C_Mem_Write(hi2c, MPU6050_DEV_ADDR, MPU6050_ACCEL_CONFIG, MPU6050_MEMADD_SIZE, &conf_4, MPU6050_CONTROL_SIZE, 10);

	return state;
}

uint8_t ReadReg(I2C_HandleTypeDef* hi2c)
{
	uint8_t state = 0;

	state = HAL_I2C_Mem_Read(hi2c, MPU6050_DEV_ADDR, MPU6050_ACCEL_XOUT_H, MPU6050_MEMADD_SIZE, rawData, MPU6050_READDATA_SIZE, 1);

	return state;
}

uint8_t GetIMUValues(I2C_HandleTypeDef* hi2c, uint8_t CalibON)
{
	uint8_t state = 0;
	state = ReadReg(hi2c);

	if (state == 0) {
		int16_t rawValues[MPU6050_TOTAL_DATA_NUM] = {0};

		for (uint8_t i = 0; i < MPU6050_TOTAL_DATA_NUM; i++) {
			rawValues[i] = (int16_t)(rawData[i * 2] << 8 | rawData[i * 2 + 1]);
		}

	    IMUData.accX = (float)(rawValues[0] / MPU6050_ACCEL_SCALE_FACTOR_4g);
	    IMUData.accY = (float)(rawValues[1] / MPU6050_ACCEL_SCALE_FACTOR_4g);
	    IMUData.accZ = (float)(rawValues[2] / MPU6050_ACCEL_SCALE_FACTOR_4g);
	    IMUData.temp = (float)(rawValues[3] / MPU6050_TEMP_SCALE_FACTOR) + MPU6050_TEMP_OFFSET;
	    IMUData.gyrX = (float)(rawValues[4] / MPU6050_GYRO_SCALE_FACTOR_500dps);
	    IMUData.gyrY = (float)(rawValues[5] / MPU6050_GYRO_SCALE_FACTOR_500dps);
	    IMUData.gyrZ = (float)(rawValues[6] / MPU6050_GYRO_SCALE_FACTOR_500dps);

		if (CalibON == 1){
		    IMUData.accX -= IMUData.accXoffset;
		    IMUData.accY -= IMUData.accYoffset;
		    IMUData.accZ -= IMUData.accZoffset;

		    IMUData.gyrX -= IMUData.gyrXoffset;
		    IMUData.gyrY -= IMUData.gyrYoffset;
		    IMUData.gyrZ -= IMUData.gyrZoffset;
		}
	}

	return state;
}

/* Let the vector forwarding ground is equal to [-y axis] */
void CalibrateIMU(I2C_HandleTypeDef* hi2c)
{
	uint8_t state = 0;
	uint16_t calibNum = 5000;
	uint16_t correctNum = 0;

	while (cmd == 0) {

	}

	if (cmd == 1) {
		for (int i = 0; i < calibNum; i++) {
			state = GetIMUValues(hi2c, 0);
			if (state == 0) {
				IMUData.accXoffset += IMUData.accX;
				IMUData.accYoffset += IMUData.accY;
				IMUData.accZoffset += IMUData.accZ;
				IMUData.gyrXoffset += IMUData.gyrX;
				IMUData.gyrYoffset += IMUData.gyrY;
				IMUData.gyrZoffset += IMUData.gyrZ;

				correctNum++;
			}
			HAL_Delay(1);
		}

		/* After total calibration */
		IMUData.accXoffset /= correctNum;
		IMUData.accYoffset /= correctNum;
		IMUData.accZoffset /= correctNum;
		IMUData.gyrXoffset /= correctNum;
		IMUData.gyrYoffset /= correctNum;
		IMUData.gyrZoffset /= correctNum;

		IMUData.accYoffset -= 1;

		calibDone = 1;
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
