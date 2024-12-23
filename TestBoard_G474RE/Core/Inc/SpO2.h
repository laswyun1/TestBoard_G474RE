/*
 * SpO2.h
 *
 *  Created on: Dec 23, 2024
 *      Author: INVINCIBLE
 */

#ifndef INC_SPO2_H_
#define INC_SPO2_H_

#include "stm32g4xx_hal.h"

// ------------------------------------------- Register Address ------------------------------------------- //
/* DEVICE ADDRESS */
#define MAX30102_DEV_ADDR							0x57U
#define MAX30102_I2C_WRITE_ADDR						0xAEU
#define MAX30102_I2C_READ_ADDR						0xAFU

/* STATUS */
#define MAX30102_INTERRUPT_STATUS_1 				0x00U
#define MAX30102_INTERRUPT_STATUS_2 				0x01U
#define MAX30102_INTERRUPT_ENABLE_1		            0x02U
#define MAX30102_INTERRUPT_ENABLE_2 				0x03U

/* FIFO */
#define MAX30102_FIFO_WRITE_POINTER					0x04U
#define MAX30102_OVERFLOW_COUNTER 					0x05U
#define MAX30102_FIFO_READ_POINTER					0x06U
#define MAX30102_FIFO_DATA_REGISTER					0x07U

/* CONFIGURATION */
#define MAX30102_FIFO_CONFIG						0x08U
#define MAX30102_MODE_CONFIG						0x09U
#define MAX30102_SPO2_CONFIG						0x0AU
#define MAX30102_LED1_PULSEAMP						0x0CU
#define MAX30102_LED2_PULSEAMP						0x0DU
#define MAX30102_PROXIMITY_PULSEAMP					0x10U
#define MAX30102_MULTILED_CONTROL_1					0x11U
#define MAX30102_MULTILED_CONTROL_2					0x12U

/* DIE TEMPERATURE */
#define MAX30102_DIETEMP_INT						0x1FU
#define MAX30102_DIETEMP_FRAC						0x20U
#define MAX30102_DIETEMP_CONFIG						0x21U

/* PROXIMITY FUNCTION */
#define MAX30102_PROXIMITY_INTTERRUPT_THRES			0x30U

/* PART ID */
#define MAX30102_REVISION_ID						0xFEU
#define MAX30102_PART_ID							0xFFU
// -------------------------------------------------------------------------------------------------------- //


// ------------------------------------------- Register Commands ------------------------------------------ //
#define MAX30102_MODE_HEARTRATE					    0x02U
#define MAX30102_MODE_SPO2							0x03U
#define MAX30102_MODE_MULTILED						0x07U

#define MAX30102_SPO2_ADC_11BIT						0x00U			// 7.81,  2048
#define MAX30102_SPO2_ADC_12BIT						0x20U			// 15.63, 4096
#define MAX30102_SPO2_ADC_13BIT						0x40U			// 31.25, 8192
#define MAX30102_SPO2_ADC_14BIT						0x60U			// 62.5,  16384

#define MAX30102_SPO2_SAMPLERATE_50					0x00U
#define MAX30102_SPO2_SAMPLERATE_100				0x04U
#define MAX30102_SPO2_SAMPLERATE_200				0x08U
#define MAX30102_SPO2_SAMPLERATE_400				0x0CU
#define MAX30102_SPO2_SAMPLERATE_800				0x10U
#define MAX30102_SPO2_SAMPLERATE_1000				0x14U
#define MAX30102_SPO2_SAMPLERATE_1600				0x18U
#define MAX30102_SPO2_SAMPLERATE_3200				0x1CU

#define MAX30102_PULSEWIDTH_69						0x00U			// ADC Resolution: 15-bit
#define MAX30102_PULSEWIDTH_118						0x01U			// ADC Resolution: 16-bit
#define MAX30102_PULSEWIDTH_215						0x02U			// ADC Resolution: 17-bit
#define MAX30102_PULSEWIDTH_411						0x03U			// ADC Resolution: 18-bit
// -------------------------------------------------------------------------------------------------------- //


typedef struct _SPO2_Obj_t {
	I2C_HandleTypeDef* SPO2_i2c;

};

#endif /* INC_SPO2_H_ */
