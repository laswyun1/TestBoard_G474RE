/*
 * SpO2.c
 *
 *  Created on: Dec 23, 2024
 *      Author: INVINCIBLE
 */

#include "SpO2.h"

/* For I2C Receive/Transmit */
static uint8_t I2CRxData[READ_DATA_LENGTH_MAX] = {0};
static uint8_t I2CTxData[WRITE_DATA_LENGTH_MAX] = {0};

/* For DWT clock check */
float sysMHz = 170;		// Change this value according to MCU

/* For Heart Rate */
int16_t IR_AC_MAX = 20;
int16_t IR_AC_MIN = -20;

int16_t IR_AC_Signal_Curr = 0;
int16_t IR_AC_Signal_Prev;
int16_t IR_AC_Signal_Min = 0;
int16_t IR_AC_Signal_Max = 0;
int16_t IR_Average_Estimated;

int16_t positiveEdge = 0;
int16_t negativeEdge = 0;
int32_t ir_avg_reg = 0;

int16_t cBuf[32];
uint8_t offset = 0;

static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};



/* Initialize the sensor */
SPO2_State_t SPO2_Init(SPO2_Obj_t* spo2_Obj, I2C_HandleTypeDef* hi2c)
{
	spo2_Obj->SPO2_i2c = hi2c;
	spo2_Obj->devReadAddr = MAX30102_I2C_READ_ADDR;
	spo2_Obj->devWriteAddr = MAX30102_I2C_WRITE_ADDR;

	if (SPO2_ReadPartID(spo2_Obj) != MAX_30102_EXPECTEDPARTID) {
		return SPO2_STATE_ERROR;
	}

	SPO2_ReadRevisionID(spo2_Obj);

	return SPO2_STATE_OK;
}

/*--------------------------------------------------------------------------- Interrupt Configuration ---------------------------------------------------------------------------*/
/* Enable the temperature ready interrupt */
void SPO2_EnableDIETEMPRDY(SPO2_Obj_t* spo2_Obj)
{
	SPO2_BitMask(spo2_Obj, MAX30102_INTERRUPT_ENABLE_2, MAX30102_INTERRUPT_DIE_TEMP_RDY_MASK, MAX30102_INTERRUPT_DIE_TEMP_RDY_ENABLE);
}


/* Enable the temperature ready interrupt */
void SPO2_DisableDIETEMPRDY(SPO2_Obj_t* spo2_Obj)
{
	SPO2_BitMask(spo2_Obj, MAX30102_INTERRUPT_ENABLE_2, MAX30102_INTERRUPT_DIE_TEMP_RDY_MASK, MAX30102_INTERRUPT_DIE_TEMP_RDY_DISABLE);
}

/*--------------------------------------------------------------------------- End of Interrupt CONFIG ----------------------------------------------------------------------------*/


/* Reset the sensor */
void SPO2_SoftReset(SPO2_Obj_t* spo2_Obj)
{
	SPO2_BitMask(spo2_Obj, MAX30102_MODE_CONFIG, MAX30102_MODE_RESET_MASK, MAX30102_MODE_RESET_ENABLE);

	float usStart = (float)DWT->CYCCNT / sysMHz;

	while ( (float)DWT->CYCCNT/sysMHz - usStart < 100000) {
		uint8_t state = 0;
		uint8_t rxData = 0;
		state = SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_MODE_CONFIG, I2CRxData, 1);

		if (state == 0) {
			rxData = I2CRxData[0];
			if ((rxData & MAX30102_MODE_RESET_ENABLE) == 0) {
				break;
				HAL_Delay(1);
			}
		}
	}
}


/* Set the LED mode */
/* [NOTE] RED only, RED + IR, Multi-LED */
void SPO2_SetLEDMode(SPO2_Obj_t* spo2_Obj, uint8_t LEDMode)
{
	SPO2_BitMask(spo2_Obj, MAX30102_MODE_CONFIG, MAX30102_MODE_MASK, LEDMode);
}


/* Set the ADC range */
/* [NOTE] ADC Range : 2048(11-bit), 4096(12-bit), 8192(13-bit), 16384(14-bit) */
void SPO2_SetADCRange(SPO2_Obj_t* spo2_Obj, uint8_t ADCRange)
{
	SPO2_BitMask(spo2_Obj, MAX30102_SPO2_CONFIG, MAX30102_ADCRANGE_MASK, ADCRange);
}


/* Set the sample rate */
/* [NOTE] Sample Rate : 50, 100, 200, 400, 800, 1000, 1600, 3200 */
void SPO2_SetSampleRate(SPO2_Obj_t* spo2_Obj, uint8_t sampleRate)
{
	SPO2_BitMask(spo2_Obj, MAX30102_SPO2_CONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}


/* Set the pulse width */
/* [NOTE] Pulse Width : 69, 118, 215, 411 */
void SPO2_SetPulseWidth(SPO2_Obj_t* spo2_Obj, uint8_t pulseWidth)
{
	SPO2_BitMask(spo2_Obj, MAX30102_SPO2_CONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}


/* Set the amplitude of RED pulse */
/* [NOTE] Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical) */
void SPO2_SetPulseAmpRed(SPO2_Obj_t* spo2_Obj, uint8_t pulseAmp)
{
	I2CTxData[0] = pulseAmp;
	SPO2_WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, MAX30102_LED1_PULSEAMP, I2CTxData, 1);
}


/* Set the amplitude of IR pulse */
void SPO2_SetPulseAmpIR(SPO2_Obj_t* spo2_Obj, uint8_t pulseAmp)
{
	I2CTxData[0] = pulseAmp;
	SPO2_WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, MAX30102_LED2_PULSEAMP, I2CTxData, 1);
}


/* Set the amplitude of GREEN pulse */
void SPO2_SetPulseAmpGreen(SPO2_Obj_t* spo2_Obj, uint8_t pulseAmp)
{
	I2CTxData[0] = pulseAmp;
	SPO2_WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, MAX30102_LED3_PULSEAMP, I2CTxData, 1);
}


/* Set the amplitude of proximity pulse */
void SPO2_SetPulseAmpProximity(SPO2_Obj_t* spo2_Obj, uint8_t pulseAmp)
{
	I2CTxData[0] = pulseAmp;
	SPO2_WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, MAX30102_PROXIMITY_PULSEAMP, I2CTxData, 1);
}


/* Assign the slots for each device(RED, IR, GREEN) */
void SPO2_EnableSlot(SPO2_Obj_t* spo2_Obj, uint8_t slotNum, uint8_t device)
{
	switch (slotNum) {
		case (1):
			SPO2_BitMask(spo2_Obj, MAX30102_MULTILED_CONTROL_1, MAX30102_SLOT1_MASK, device);
			break;
		case (2):
			SPO2_BitMask(spo2_Obj, MAX30102_MULTILED_CONTROL_1, MAX30102_SLOT2_MASK, device<<4);
			break;
		case (3):
			SPO2_BitMask(spo2_Obj, MAX30102_MULTILED_CONTROL_2, MAX30102_SLOT3_MASK, device);
			break;
		case (4):
			SPO2_BitMask(spo2_Obj, MAX30102_MULTILED_CONTROL_2, MAX30102_SLOT4_MASK, device<<4);
			break;
		default:
			/* If the code goes into here, process has an error */
			break;
	}
}


/* Set sample average */
void SPO2_SetFIFOAverage(SPO2_Obj_t* spo2_Obj, uint8_t sampleNum)
{
	SPO2_BitMask(spo2_Obj, MAX30102_FIFO_CONFIG, MAX30102_FIFO_SAMPLEAVG_MASK, sampleNum);
}


/* Clearing FIFO after "finishing SETUP" before "starting to read" */
void SPO2_ClearFIFO(SPO2_Obj_t* spo2_Obj)
{
	I2CTxData[0] = 0b00000000;
	SPO2_WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, MAX30102_FIFO_WRITE_POINTER, I2CTxData, 1);
	SPO2_WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, MAX30102_OVERFLOW_COUNTER, I2CTxData, 1);
	SPO2_WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, MAX30102_FIFO_READ_POINTER, I2CTxData, 1);
}


/* Enable Roll-over if FIFO overflows */
void SPO2_EnableFIFORollover(SPO2_Obj_t* spo2_Obj)
{
	SPO2_BitMask(spo2_Obj, MAX30102_FIFO_CONFIG, MAX30102_FIFO_ROLLOVER_MASK, MAX30102_FIFO_ROLLOVER_ENABLE);
}


/* Disable Roll-over */
void SPO2_DisbleFIFORollover(SPO2_Obj_t* spo2_Obj)
{
	SPO2_BitMask(spo2_Obj, MAX30102_FIFO_CONFIG, MAX30102_FIFO_ROLLOVER_MASK, MAX30102_FIFO_ROLLOVER_DISABLE);
}


/* Get Write Pointer */
uint8_t SPO2_GetWritePtr(SPO2_Obj_t* spo2_Obj)
{
	uint8_t state = 0;
	uint8_t writePtr = 0;

	state = SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_FIFO_WRITE_POINTER, I2CRxData, 1);

	if (state == 0) {
		writePtr = I2CRxData[0];
		return writePtr;
	}

	/* Needs error handler */
	return writePtr;
}


/* Get Read Pointer */
uint8_t SPO2_GetReadPtr(SPO2_Obj_t* spo2_Obj)
{
	uint8_t state = 0;
	uint8_t readPtr = 0;

	state = SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_FIFO_READ_POINTER, I2CRxData, 1);

	if (state == 0) {
		readPtr = I2CRxData[0];
		return readPtr;
	}

	/* Needs error handler */
	return readPtr;
}


/* Get DIE Temperature */
void SPO2_ReadTemperature(SPO2_Obj_t* spo2_Obj)
{
	// DIE_TEMP_RDY interrupt must be enabled

	I2CTxData[0] = 0x01;
	SPO2_WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, MAX30102_DIETEMP_CONFIG, I2CTxData, 1);

	float usStart = (float)DWT->CYCCNT / sysMHz;
	while ( (float)DWT->CYCCNT / sysMHz - usStart < 100000) {
		SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_INTERRUPT_STATUS_2, I2CRxData, 1);

		uint8_t interrupt = I2CRxData[0];
		if ( (interrupt & MAX30102_INTERRUPT_DIE_TEMP_RDY_ENABLE) > 0) {
			break;
		}

		HAL_Delay(1);
	}

	SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_DIETEMP_INT, I2CRxData, 1);
	int8_t tempInt = I2CRxData[0];

	SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_DIETEMP_FRAC, I2CRxData, 1);
	uint8_t tempFrac = I2CRxData[0];

	spo2_Obj->temperatureC = ( (float)tempInt + ((float)tempFrac * 0.0625) );
	spo2_Obj->temperatureF = spo2_Obj->temperatureC * 1.8 + 32.0;
}


/* Get PART ID */
uint8_t SPO2_ReadPartID(SPO2_Obj_t* spo2_Obj)
{
	uint8_t state = 0;
	uint8_t partID = 0;

	state = SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_PART_ID, I2CRxData, 1);

	if (state == 0){
		partID = I2CRxData[0];
	}

	return partID;
}


/* Get REVISION ID */
uint8_t SPO2_ReadRevisionID(SPO2_Obj_t* spo2_Obj)
{
	uint8_t state = 0;
	uint8_t revisionID = 0;

	state = SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_REVISION_ID, I2CRxData, 1);

	if (state == 0){
		revisionID = I2CRxData[0];
	}

	return revisionID;
}


/* Setup the sensor before the starting of reading */
void SPO2_Setup(SPO2_Obj_t* spo2_Obj, SPO2_SampleAvg_t sampleAvg, SPO2_LEDMode_t LEDMode, SPO2_ADCrange_t ADCrange, SPO2_SampleRate_t sampleRate, SPO2_PulseWidth_t pulseWidth, SPO2_CurrAmp_t powerLevel)
{
	SPO2_SoftReset(spo2_Obj);

	/* ------------------------------ FIFO Configuration ------------------------------ */
	if (sampleAvg == SPO2_SAMPLEAVG_1) {
		SPO2_SetFIFOAverage(spo2_Obj, MAX30102_FIFO_SAMPLEAVG_1);
	}
	else if (sampleAvg == SPO2_SAMPLEAVG_2) {
		SPO2_SetFIFOAverage(spo2_Obj, MAX30102_FIFO_SAMPLEAVG_2);
	}
	else if (sampleAvg == SPO2_SAMPLEAVG_4) {
		SPO2_SetFIFOAverage(spo2_Obj, MAX30102_FIFO_SAMPLEAVG_4);
	}
	else if (sampleAvg == SPO2_SAMPLEAVG_8) {
		SPO2_SetFIFOAverage(spo2_Obj, MAX30102_FIFO_SAMPLEAVG_8);
	}
	else if (sampleAvg == SPO2_SAMPLEAVG_16) {
		SPO2_SetFIFOAverage(spo2_Obj, MAX30102_FIFO_SAMPLEAVG_16);
	}
	else if (sampleAvg == SPO2_SAMPLEAVG_32) {
		SPO2_SetFIFOAverage(spo2_Obj, MAX30102_FIFO_SAMPLEAVG_32);
	}
	else {
		SPO2_SetFIFOAverage(spo2_Obj, MAX30102_FIFO_SAMPLEAVG_4);
	}
	/* -------------------------------------------------------------------------------- */


	/* ---------------------------- Enable FIFO Roll over ----------------------------- */
	SPO2_EnableFIFORollover(spo2_Obj);
	/* -------------------------------------------------------------------------------- */


	/* ------------------------------ Mode Configuration ------------------------------ */
	if (LEDMode == SPO2_LEDMODE_MULTILED) {
		spo2_Obj->activeLEDNum = 3;
		SPO2_SetLEDMode(spo2_Obj, MAX30102_MODE_MULTILED);
	}
	else if (LEDMode == SPO2_LEDMODE_REDIRONLY) {
		spo2_Obj->activeLEDNum = 2;
		SPO2_SetLEDMode(spo2_Obj, MAX30102_MODE_REDIRONLY_SPO2);
	}
	else if (LEDMode == SPO2_LEDMODE_REDONLY) {
		spo2_Obj->activeLEDNum = 1;
		SPO2_SetLEDMode(spo2_Obj, MAX30102_MODE_REDONLY_HR);
	}
	else {
		spo2_Obj->activeLEDNum = 1;
		SPO2_SetLEDMode(spo2_Obj, MAX30102_MODE_REDONLY_HR);
	}
	/* -------------------------------------------------------------------------------- */


	/* ------------------------ Particle Sensing Configuration ------------------------ */
	// ADC Range //
	if (ADCrange == SPO2_ADCRANGE_11bit) {
		SPO2_SetADCRange(spo2_Obj, MAX30102_ADCRANGE_11BIT);
	}
	else if (ADCrange == SPO2_ADCRANGE_12bit) {
		SPO2_SetADCRange(spo2_Obj, MAX30102_ADCRANGE_12BIT);
	}
	else if (ADCrange == SPO2_ADCRANGE_13bit) {
		SPO2_SetADCRange(spo2_Obj, MAX30102_ADCRANGE_13BIT);
	}
	else if (ADCrange == SPO2_ADCRANGE_14bit) {
		SPO2_SetADCRange(spo2_Obj, MAX30102_ADCRANGE_14BIT);
	}
	else {
		SPO2_SetADCRange(spo2_Obj, MAX30102_ADCRANGE_11BIT);
	}


	// Sample Rate //
	if (sampleRate == SPO2_SAMPLERATE_50) {
		SPO2_SetSampleRate(spo2_Obj, MAX30102_SAMPLERATE_50);
	}
	else if (sampleRate == SPO2_SAMPLERATE_100) {
		SPO2_SetSampleRate(spo2_Obj, MAX30102_SAMPLERATE_100);
	}
	else if (sampleRate == SPO2_SAMPLERATE_200) {
		SPO2_SetSampleRate(spo2_Obj, MAX30102_SAMPLERATE_200);
	}
	else if (sampleRate == SPO2_SAMPLERATE_400) {
		SPO2_SetSampleRate(spo2_Obj, MAX30102_SAMPLERATE_400);
	}
	else if (sampleRate == SPO2_SAMPLERATE_800) {
		SPO2_SetSampleRate(spo2_Obj, MAX30102_SAMPLERATE_800);
	}
	else if (sampleRate == SPO2_SAMPLERATE_1000) {
		SPO2_SetSampleRate(spo2_Obj, MAX30102_SAMPLERATE_1000);
	}
	else if (sampleRate == SPO2_SAMPLERATE_1600) {
		SPO2_SetSampleRate(spo2_Obj, MAX30102_SAMPLERATE_1600);
	}
	else if (sampleRate == SPO2_SAMPLERATE_3200) {
		SPO2_SetSampleRate(spo2_Obj, MAX30102_SAMPLERATE_3200);
	}
	else {
		SPO2_SetSampleRate(spo2_Obj, MAX30102_SAMPLERATE_50);
	}


	// Pulse Width //
	  /* The longer the pulse width the longer range of detection you'll have
	     At 69us and 0.4mA it's about 2 inches
	     At 411us and 0.4mA it's about 6 inches */
	if (pulseWidth == SPO2_PULSEWIDTH_69) {
		SPO2_SetPulseWidth(spo2_Obj, MAX30102_PULSEWIDTH_69);
	}
	else if (pulseWidth == SPO2_PULSEWIDTH_118) {
		SPO2_SetPulseWidth(spo2_Obj, MAX30102_PULSEWIDTH_118);
	}
	else if (pulseWidth == SPO2_PULSEWIDTH_215) {
		SPO2_SetPulseWidth(spo2_Obj, MAX30102_PULSEWIDTH_215);
	}
	else if (pulseWidth == SPO2_PULSEWIDTH_411) {
		SPO2_SetPulseWidth(spo2_Obj, MAX30102_PULSEWIDTH_411);
	}
	else {
		SPO2_SetPulseWidth(spo2_Obj, MAX30102_PULSEWIDTH_69);
	}
	/* -------------------------------------------------------------------------------- */


	/* ----------------------- LED Pulse Amplitude Configuration ---------------------- */
		/* Default is 0x1F which gets us 6.4mA
		   powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
	 	   powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
		   powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
		   powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch */

	uint8_t powerCmd = 0x00;

	if (powerLevel == SPO2_CURRAMP_0) {
		powerCmd = 0x00;
	}
	else if (powerLevel == SPO2_CURRAMP_0p2) {
		powerCmd = 0x01;
	}
	else if (powerLevel == SPO2_CURRAMP_0p4) {
		powerCmd = 0x02;
	}
	else if (powerLevel == SPO2_CURRAMP_3p1) {
		powerCmd = 0x0F;
	}
	else if (powerLevel == SPO2_CURRAMP_6p4) {
		powerCmd = 0x1F;
	}
	else if (powerLevel == SPO2_CURRAMP_12p5) {
		powerCmd = 0x3F;
	}
	else if (powerLevel == SPO2_CURRAMP_25p4) {
		powerCmd = 0x7F;
	}
	else if (powerLevel == SPO2_CURRAMP_50) {
		powerCmd = 0xFF;
	}
	else {
		powerCmd = 0x1F;
	}
	SPO2_SetPulseAmpRed(spo2_Obj, powerCmd);
	SPO2_SetPulseAmpIR(spo2_Obj, powerCmd);
	SPO2_SetPulseAmpProximity(spo2_Obj, powerCmd);
	/* -------------------------------------------------------------------------------- */


	/* ---------- Multi-LED Mode Configuration (Enable the reading of LEDs) --------- */
	SPO2_EnableSlot(spo2_Obj, 1, MAX30102_SLOT_RED_LED);
	if (spo2_Obj->activeLEDNum > 1) {
		SPO2_EnableSlot(spo2_Obj, 2, MAX30102_SLOT_IR_LED);
	}
	if (spo2_Obj->activeLEDNum > 2) {
		SPO2_EnableSlot(spo2_Obj, 3, MAX30102_SLOT_GREEN_LED);
	}
	/* -------------------------------------------------------------------------------- */


	/* -------------------------------- Reset the FIFO -------------------------------- */
	SPO2_ClearFIFO(spo2_Obj);
	/* -------------------------------------------------------------------------------- */
}


/* Take the recent RED value */
uint32_t SPO2_GetRED(SPO2_Obj_t* spo2_Obj)
{
	// Check the sensor for new data for 250,000usec(=250ms)
	if (SPO2_SafeCheck(spo2_Obj, 250000))
		return (spo2_Obj->RED[spo2_Obj->head]);
	else
		return 0; // Sensor failed to find new data
}


/* Take the recent IR value */
uint32_t SPO2_GetIR(SPO2_Obj_t* spo2_Obj)
{
	//Check the sensor for new data for 250,000usec(=250ms)
	if (SPO2_SafeCheck(spo2_Obj, 250000))
		return (spo2_Obj->IR[spo2_Obj->head]);
	else
		return 0; // Sensor failed to find new data
}


/* Take the recent GREEN value(for MAX30105) -> MAX30102 does NOT have GREEN */
uint32_t SPO2_GetGREEN(SPO2_Obj_t* spo2_Obj)
{
	//Check the sensor for new data for 250,000usec(=250ms)
	if (SPO2_SafeCheck(spo2_Obj, 250000))
		return (spo2_Obj->GREEN[spo2_Obj->head]);
	else
		return 0; // Sensor failed to find new data
}


/* Polling the new sensor data */
uint16_t SPO2_Check(SPO2_Obj_t* spo2_Obj)
{
	uint8_t readPtr = SPO2_GetReadPtr(spo2_Obj);
	uint8_t writePtr = SPO2_GetWritePtr(spo2_Obj);

	int sampleNum = 0;

	if (readPtr != writePtr) {
		sampleNum = writePtr - readPtr;
		if (sampleNum < 0) {
			sampleNum += 32;
		}

		int bytesLeftToRead = sampleNum * (spo2_Obj->activeLEDNum) * 3;

		while (bytesLeftToRead > 0) {
			int toGet = bytesLeftToRead;

			if (toGet > I2C_BUFFER_LENGTH) {
				toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (spo2_Obj->activeLEDNum * 3));
			}

			bytesLeftToRead -= toGet;
			SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, MAX30102_FIFO_DATA_REGISTER, I2CRxData, toGet);

			uint8_t cursor = 0;

			while (toGet > 0) {
				spo2_Obj->head++;
				spo2_Obj->head %= STORAGE_SIZE;

				uint8_t temp[4];
				uint32_t tempLong;

				// Burst Read 3 bytes for RED //
				temp[3] = 0;
				temp[2] = I2CRxData[cursor++];
				temp[1] = I2CRxData[cursor++];
				temp[0] = I2CRxData[cursor++];

				memcpy(&tempLong, temp, sizeof(tempLong));
				tempLong &= 0x3FFFF;	// use only 18-bit

				spo2_Obj->RED[spo2_Obj->head] = tempLong;

				if (spo2_Obj->activeLEDNum > 1) {
					// Burst Read 3 bytes for IR //
					temp[3] = 0;
					temp[2] = I2CRxData[cursor++];
					temp[1] = I2CRxData[cursor++];
					temp[0] = I2CRxData[cursor++];

					memcpy(&tempLong, temp, sizeof(tempLong));
					tempLong &= 0x3FFFF;	// use only 18-bit
					spo2_Obj->IR[spo2_Obj->head] = tempLong;
				}

				if (spo2_Obj->activeLEDNum > 2) {
					// Burst Read 3 bytes for GREEN //
					temp[3] = 0;
					temp[2] = I2CRxData[cursor++];
					temp[1] = I2CRxData[cursor++];
					temp[0] = I2CRxData[cursor++];

					memcpy(&tempLong, temp, sizeof(tempLong));
					tempLong &= 0x3FFFF;	// use only 18-bit
					spo2_Obj->GREEN[spo2_Obj->head] = tempLong;
				}

				toGet -= spo2_Obj->activeLEDNum * 3;
			}
		}
	}

	return sampleNum;		// Return total number of new samples
}


/* Check the new data */
uint8_t SPO2_SafeCheck(SPO2_Obj_t* spo2_Obj, uint32_t timeOut)
{
	float usStart = (float)DWT->CYCCNT / sysMHz;
	while (1) {
		if ((float)DWT->CYCCNT/sysMHz - usStart > timeOut) {
			return 0;
		}
		if (SPO2_Check(spo2_Obj) > 0)
			return 1;

		HAL_Delay(1);
	}
}


/* Masking & Setting the value of certain register */
void SPO2_BitMask(SPO2_Obj_t* spo2_Obj, uint8_t regAddr, uint8_t mask, uint8_t setBit)
{
	uint8_t originalData = 0;

	if (SPO2_ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, regAddr, I2CRxData, 1) == 0){
		originalData = I2CRxData[0];

		originalData = originalData & mask;		// Makes certain portion of register value zero.

		I2CTxData[0] = originalData | setBit;
		SPO2_WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, regAddr, I2CTxData, 1);
	}
}


/* I2C READ function */
uint8_t SPO2_ReadReg(I2C_HandleTypeDef* hi2c, uint8_t devReadAddr, uint8_t regAddr, uint8_t* readBuff, uint8_t size)
{
	uint8_t state = 0;

	state = HAL_I2C_Mem_Read(hi2c, devReadAddr, regAddr, I2C_MEM_ADD_SIZE, readBuff, size, I2C_TIMEOUT);

	return state;
}

/* I2C WRITE function */
uint8_t SPO2_WriteReg(I2C_HandleTypeDef* hi2c, uint8_t devWriteAddr, uint8_t regAddr, uint8_t* writeBuff, uint8_t size)
{
	uint8_t state = 0;

	state = HAL_I2C_Mem_Write(hi2c, devWriteAddr, regAddr, I2C_MEM_ADD_SIZE, writeBuff, size, I2C_TIMEOUT);

	return state;
}






/*---------------------------------------------------------------------------------- [For Heart Rate] ---------------------------------------------------------------------------------- */
uint8_t SPO2_CheckForBeat(SPO2_Obj_t* spo2_Obj, int32_t sample)
{
	uint8_t beatDetected = 0;

	IR_AC_Signal_Prev = IR_AC_Signal_Curr;

	IR_Average_Estimated = SPO2_AverageDCEstimator(&ir_avg_reg, sample);
	IR_AC_Signal_Curr = SPO2_LowPassFIR(sample - IR_Average_Estimated);

	/* Detect the rising edge */
	if ((IR_AC_Signal_Prev < 0) & (IR_AC_Signal_Curr >= 0)) {
		IR_AC_MAX = IR_AC_Signal_Max;
		IR_AC_MIN = IR_AC_Signal_Min;

		positiveEdge = 1;
		negativeEdge = 0;
		IR_AC_Signal_Max = 0;

		if ( ((IR_AC_MAX - IR_AC_MIN) > 20) & ((IR_AC_MAX - IR_AC_MIN) < 1000) )
		{
			beatDetected = 1;		// Heart-beat is detected
		}
	}

	/* Detect the falling edge */
	if ((IR_AC_Signal_Prev > 0) & (IR_AC_Signal_Curr <= 0)) {
		positiveEdge = 0;
		negativeEdge = 1;
		IR_AC_Signal_Min = 0;
	}

	/* Find Maximum value in positive cycle */
	if (positiveEdge & (IR_AC_Signal_Curr > IR_AC_Signal_Prev)) {
		IR_AC_Signal_Max = IR_AC_Signal_Curr;
	}

	/* Find Minimum value in negative cycle */
	if (negativeEdge & (IR_AC_Signal_Curr < IR_AC_Signal_Prev)) {
		IR_AC_Signal_Min = IR_AC_Signal_Curr;
	}

	return beatDetected;
}


/* Average DC Estimator */
int16_t SPO2_AverageDCEstimator(int32_t *p, uint16_t x)
{
	*p += ((((long) x << 15) - *p) >> 4);
	return (*p >> 15);
}


/* Low Pass FIR filter */
int16_t SPO2_LowPassFIR(int16_t input)
{
	cBuf[offset] = input;
	int32_t output = SPO2_Multiply16Bit(FIRCoeffs[11], cBuf[(offset - 11) & 0x1F]);

	for (uint8_t i = 0; i < 11; i++) {
		output += SPO2_Multiply16Bit(FIRCoeffs[i], cBuf[(offset - i) & 0x1F] + cBuf[(offset - 22 + i) & 0x1F]);
	}

	offset++;
	offset %= 32;

	return (output >> 15);
}



int32_t SPO2_Multiply16Bit(int16_t x, int16_t y)
{
	int32_t result = (int32_t)((long)x * (long)y);

	return result;
}

















