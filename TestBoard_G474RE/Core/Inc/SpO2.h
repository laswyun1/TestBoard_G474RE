/*
 * SpO2.h
 *
 *  Created on: Dec 23, 2024
 *      Author: INVINCIBLE
 */

/* OSTSen-30102 */

#ifndef INC_SPO2_H_
#define INC_SPO2_H_

#include "stm32g4xx_hal.h"
#include "string.h"

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
#define MAX30102_LED3_PULSEAMP						0x0EU			// It is for MAX30105 (No Green LED in MAX30102)
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
/* INTERRUPT CONFIGURATION */
#define MAX30102_INTERRUPT_A_FULL_MASK				0b01111111
#define MAX30102_INTERRUPT_A_FULL_ENABLE			0x80U
#define MAX30102_INTERRUPT_A_FULL_DISABLE			0x00U

#define MAX30102_INTERRUPT_DATA_RDY_MASK			0b10111111
#define MAX30102_INTERRUPT_DATA_RDY_ENABLE			0x40U
#define MAX30102_INTERRUPT_DATA_RDY_DISABLE			0x00U

#define MAX30102_INTERRUPT_ALC_OVF_MASK				0b11011111
#define MAX30102_INTERRUPT_ALC_OVF_ENABLE			0x20U
#define MAX30102_INTERRUPT_ALC_OVF_DISABLE			0x00U

#define MAX30102_INTERRUPT_PROX_MASK				0b11101111
#define MAX30102_INTERRUPT_PROX_ENABLE 				0x10U
#define MAX30102_INTERRUPT_PROX_DISABLE				0x00U

#define MAX30102_INTERRUPT_DIE_TEMP_RDY_MASK		0b11111101
#define MAX30102_INTERRUPT_DIE_TEMP_RDY_ENABLE		0x02U
#define MAX30102_INTERRUPT_DIE_TEMP_RDY_DISABLE		0x00U


/* FIFO CONFIGURATION */
#define MAX30102_FIFO_SAMPLEAVG_MASK				0b00011111
#define MAX30102_FIFO_SAMPLEAVG_1					0x00U
#define MAX30102_FIFO_SAMPLEAVG_2					0x20U
#define MAX30102_FIFO_SAMPLEAVG_4					0x40U
#define MAX30102_FIFO_SAMPLEAVG_8					0x60U
#define MAX30102_FIFO_SAMPLEAVG_16					0x80U
#define MAX30102_FIFO_SAMPLEAVG_32					0xA0U

#define MAX30102_FIFO_ROLLOVER_MASK					0xEFU
#define MAX30102_FIFO_ROLLOVER_ENABLE				0x10U
#define MAX30102_FIFO_ROLLOVER_DISABLE				0x00U

#define MAX30102_FIFO_A_FULL_MASK					0xF0U


/* MODE CONFIGURATION */
#define MAX30102_MODE_SHUTDOWN_MASK					0x7FU
#define MAX30102_MODE_SHUTDOWN_ENABLE				0x80U
#define MAX30102_MODE_WAKEUP						0x00U

#define MAX30102_MODE_RESET_MASK					0xBFU
#define MAX30102_MODE_RESET_ENABLE					0x40U

#define MAX30102_MODE_MASK							0xF8U
#define MAX30102_MODE_REDONLY_HR					0x02U
#define MAX30102_MODE_REDIRONLY_SPO2				0x03U
#define MAX30102_MODE_MULTILED						0x07U


/* PARTICLE SENSING CONFIGURATION */
#define MAX30102_ADCRANGE_MASK						0x9FU

#define MAX30102_ADCRANGE_11BIT						0x00U			// 7.81pA per LSB,  2048
#define MAX30102_ADCRANGE_12BIT						0x20U			// 15.63pA per LSB, 4096
#define MAX30102_ADCRANGE_13BIT						0x40U			// 31.25pA per LSB, 8192
#define MAX30102_ADCRANGE_14BIT						0x60U			// 62.5pA per LSB,  16384

#define MAX30102_SAMPLERATE_MASK					0xE3U

#define MAX30102_SAMPLERATE_50						0x00U
#define MAX30102_SAMPLERATE_100						0x04U
#define MAX30102_SAMPLERATE_200						0x08U
#define MAX30102_SAMPLERATE_400						0x0CU
#define MAX30102_SAMPLERATE_800						0x10U
#define MAX30102_SAMPLERATE_1000					0x14U
#define MAX30102_SAMPLERATE_1600					0x18U
#define MAX30102_SAMPLERATE_3200					0x1CU

#define MAX30102_PULSEWIDTH_MASK					0xFCU

#define MAX30102_PULSEWIDTH_69						0x00U			// ADC Resolution: 15-bit
#define MAX30102_PULSEWIDTH_118						0x01U			// ADC Resolution: 16-bit
#define MAX30102_PULSEWIDTH_215						0x02U			// ADC Resolution: 17-bit
#define MAX30102_PULSEWIDTH_411						0x03U			// ADC Resolution: 18-bit


/* MULTI-LED MODE CONFIGURATION */
#define MAX30102_SLOT1_MASK							0xF8U
#define MAX30102_SLOT2_MASK							0x8FU
#define MAX30102_SLOT3_MASK							0xF8U
#define MAX30102_SLOT4_MASK							0x8FU

#define MAX30102_SLOT_NONE							0x00U
#define MAX30102_SLOT_RED_LED						0x01U
#define MAX30102_SLOT_IR_LED						0x02U
#define MAX30102_SLOT_GREEN_LED						0x03U
#define MAX30102_SLOT_RED_PILOT						0x05U
#define MAX30102_SLOT_IR_PILOT						0x06U

#define MAX_30102_EXPECTEDPARTID					0x15U
// -------------------------------------------------------------------------------------------------------- //

#define READ_DATA_LENGTH_MAX		32
#define WRITE_DATA_LENGTH_MAX		1
#define I2C_TIMEOUT					1
#define I2C_MEM_ADD_SIZE			1
#define I2C_BUFFER_LENGTH 			32
#define STORAGE_SIZE				4


typedef enum _SPO2_State_t {
	SPO2_STATE_OK,
	SPO2_STATE_ERROR
} SPO2_State_t;

typedef enum _SPO2_SampleAvg_t {
	SPO2_SAMPLEAVG_1,
	SPO2_SAMPLEAVG_2,
	SPO2_SAMPLEAVG_4,
	SPO2_SAMPLEAVG_8,
	SPO2_SAMPLEAVG_16,
	SPO2_SAMPLEAVG_32
} SPO2_SampleAvg_t;

typedef enum _SPO2_LEDMODE_t {
	SPO2_LEDMODE_REDONLY,
	SPO2_LEDMODE_REDIRONLY,
	SPO2_LEDMODE_MULTILED
} SPO2_LEDMode_t;

typedef enum _SPO2_SampleRate_t {
	SPO2_SAMPLERATE_50,
	SPO2_SAMPLERATE_100,
	SPO2_SAMPLERATE_200,
	SPO2_SAMPLERATE_400,
	SPO2_SAMPLERATE_800,
	SPO2_SAMPLERATE_1000,
	SPO2_SAMPLERATE_1600,
	SPO2_SAMPLERATE_3200
} SPO2_SampleRate_t;

typedef enum _SPO2_PulseWidth_t {
	SPO2_PULSEWIDTH_69,
	SPO2_PULSEWIDTH_118,
	SPO2_PULSEWIDTH_215,
	SPO2_PULSEWIDTH_411
} SPO2_PulseWidth_t;

typedef enum _SPO2_ADCrange_t {
	SPO2_ADCRANGE_11bit,
	SPO2_ADCRANGE_12bit,
	SPO2_ADCRANGE_13bit,
	SPO2_ADCRANGE_14bit
} SPO2_ADCrange_t;

typedef enum _SPO2_CurrAmp_t {
	SPO2_CURRAMP_0,
	SPO2_CURRAMP_0p2,
	SPO2_CURRAMP_0p4,
	SPO2_CURRAMP_3p1,
	SPO2_CURRAMP_6p4,
	SPO2_CURRAMP_12p5,
	SPO2_CURRAMP_25p4,
	SPO2_CURRAMP_50
} SPO2_CurrAmp_t;

typedef struct _SPO2_HR_t {
	uint8_t BPM;
	uint8_t BPM_avg;
} SPO2_HR_t;

typedef struct _SPO2_Obj_t {
	I2C_HandleTypeDef* SPO2_i2c;
	uint8_t devReadAddr;
	uint8_t devWriteAddr;

	uint8_t activeLEDNum;

	uint32_t RED[STORAGE_SIZE];
	uint32_t IR[STORAGE_SIZE];
	uint32_t GREEN[STORAGE_SIZE];

	uint8_t head;
	uint8_t tail;

	float temperatureC;
	float temperatureF;

	SPO2_HR_t HRobj;
} SPO2_Obj_t;


/* Declaration of Functions */
SPO2_State_t SPO2_Init(SPO2_Obj_t* spo2_Obj, I2C_HandleTypeDef* hi2c);
void SPO2_EnableDIETEMPRDY(SPO2_Obj_t* spo2_Obj);
void SPO2_DisableDIETEMPRDY(SPO2_Obj_t* spo2_Obj);
void SPO2_SoftReset(SPO2_Obj_t* spo2_Obj);
void SPO2_SetLEDMode(SPO2_Obj_t* spo2_Obj, uint8_t LEDMode);
void SPO2_SetADCRange(SPO2_Obj_t* spo2_Obj, uint8_t ADCRange);
void SPO2_SetSampleRate(SPO2_Obj_t* spo2_Obj, uint8_t sampleRate);
void SPO2_SetPulseWidth(SPO2_Obj_t* spo2_Obj, uint8_t pulseWidth);
void SPO2_SetPulseAmpRed(SPO2_Obj_t* spo2_Obj, uint8_t pulseAmp);
void SPO2_SetPulseAmpIR(SPO2_Obj_t* spo2_Obj, uint8_t pulseAmp);
void SPO2_SetPulseAmpGreen(SPO2_Obj_t* spo2_Obj, uint8_t pulseAmp);
void SPO2_SetPulseAmpProximity(SPO2_Obj_t* spo2_Obj, uint8_t pulseAmp);
void SPO2_EnableSlot(SPO2_Obj_t* spo2_Obj, uint8_t slotNum, uint8_t device);
void SPO2_SetFIFOAverage(SPO2_Obj_t* spo2_Obj, uint8_t sampleNum);
void SPO2_ClearFIFO(SPO2_Obj_t* spo2_Obj);
void SPO2_EnableFIFORollover(SPO2_Obj_t* spo2_Obj);
void SPO2_DisbleFIFORollover(SPO2_Obj_t* spo2_Obj);
uint8_t SPO2_GetWritePtr(SPO2_Obj_t* spo2_Obj);
uint8_t SPO2_GetReadPtr(SPO2_Obj_t* spo2_Obj);
void SPO2_ReadTemperature(SPO2_Obj_t* spo2_Obj);
uint8_t SPO2_ReadPartID(SPO2_Obj_t* spo2_Obj);
uint8_t SPO2_ReadRevisionID(SPO2_Obj_t* spo2_Obj);
void SPO2_Setup(SPO2_Obj_t* spo2_Obj, SPO2_SampleAvg_t sampleAvg, SPO2_LEDMode_t LEDMode, SPO2_ADCrange_t ADCrange, SPO2_SampleRate_t sampleRate, SPO2_PulseWidth_t pulseWidth, SPO2_CurrAmp_t powerLevel);
uint32_t SPO2_GetRED(SPO2_Obj_t* spo2_Obj);
uint32_t SPO2_GetIR(SPO2_Obj_t* spo2_Obj);
uint32_t SPO2_GetGREEN(SPO2_Obj_t* spo2_Obj);
uint16_t SPO2_Check(SPO2_Obj_t* spo2_Obj);
uint8_t SPO2_SafeCheck(SPO2_Obj_t* spo2_Obj, uint32_t timeOut);
void SPO2_BitMask(SPO2_Obj_t* spo2_Obj, uint8_t regAddr, uint8_t mask, uint8_t setBit);
uint8_t SPO2_ReadReg(I2C_HandleTypeDef* hi2c, uint8_t devReadAddr, uint8_t regAddr, uint8_t* readBuff, uint8_t size);
uint8_t SPO2_WriteReg(I2C_HandleTypeDef* hi2c, uint8_t devWriteAddr, uint8_t regAddr, uint8_t* writeBuff, uint8_t size);
uint8_t SPO2_CheckForBeat(SPO2_Obj_t* spo2_Obj, int32_t sample);
int16_t SPO2_AverageDCEstimator(int32_t *p, uint16_t x);
int16_t SPO2_LowPassFIR(int16_t input);
int32_t SPO2_Multiply16Bit(int16_t x, int16_t y);


#endif /* INC_SPO2_H_ */
