/*
 * pMMG.c
 *
 *  Created on: Jul 5, 2024
 *      Author: INVINCIBLE
 */


#include "pMMG.h"

/* SPI Transmission Data */
static uint8_t SPITxData;

/* OSR setting */
static uint8_t pressureOSR = OSR_256;
static uint8_t temperatureOSR = OSR_256;


pMMG_State_t pMMG_Init(pMMG_Obj_t* pMMG_Obj, SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	pMMG_Obj->pMMG_hspi = hspi;
	pMMG_Obj->pMMG_CS_GPIO_Port = GPIOx;
	pMMG_Obj->pMMG_CS_Pin = GPIO_Pin;

	pMMG_EnableCS(pMMG_Obj);
	SPITxData = RESET_CMD;
	HAL_SPI_Transmit(pMMG_Obj->pMMG_hspi, &SPITxData, 1, 10);
//	HAL_Delay(3);
	us_Delay(3000);
	pMMG_DisableCS(pMMG_Obj);

	pMMG_ReadPROM(pMMG_Obj);

	if (pMMG_Obj->promData.reserved == 0x00 || pMMG_Obj->promData.reserved == 0xFF) {
		return pMMG_STATE_ERROR;
	}
	else {
		return pMMG_STATE_OK;
	}
}


/* Reading the PROM data */
void pMMG_ReadPROM(pMMG_Obj_t* pMMG_Obj) {
	uint8_t address;
	uint8_t promPtr[16];
	uint8_t* cursor = promPtr;

	for (address = 0; address < 8; address++) {
		SPITxData = PROM_READ(address);
		pMMG_EnableCS(pMMG_Obj);
		HAL_SPI_Transmit(pMMG_Obj->pMMG_hspi, &SPITxData, 1, 10);
		HAL_SPI_Receive(pMMG_Obj->pMMG_hspi, cursor, 2, 10);
		pMMG_DisableCS(pMMG_Obj);

		cursor += 2;
	}

//	/* Byte swap on 16-bit integers */
//	cursor = promPtr;
//	for (address = 0; address < 8; address++) {
//		uint8_t* toSwap = cursor;
//		uint8_t secondByte = toSwap[0];
//		toSwap[0] = toSwap[1];
//		toSwap[1] = secondByte;
//
//		cursor += 2;
//	}

	/* Set corresponding data */
	pMMG_Obj->promData.reserved = (uint16_t)( ((uint16_t)promPtr[0] << 8) + (uint16_t)promPtr[1] );
	pMMG_Obj->promData.sens 	= (uint16_t)( ((uint16_t)promPtr[2] << 8) + (uint16_t)promPtr[3] );			// Typ : 46372
	pMMG_Obj->promData.off 		= (uint16_t)( ((uint16_t)promPtr[4] << 8) + (uint16_t)promPtr[5] );			// Typ : 43981
	pMMG_Obj->promData.tcs 		= (uint16_t)( ((uint16_t)promPtr[6] << 8) + (uint16_t)promPtr[7] );			// Typ : 29059
	pMMG_Obj->promData.tco 		= (uint16_t)( ((uint16_t)promPtr[8] << 8) + (uint16_t)promPtr[9] );			// Typ : 27842
	pMMG_Obj->promData.tref 	= (uint16_t)( ((uint16_t)promPtr[10] << 8) + (uint16_t)promPtr[11] );		// Typ : 31553
	pMMG_Obj->promData.tempsens = (uint16_t)( ((uint16_t)promPtr[12] << 8) + (uint16_t)promPtr[13] );		// Typ : 28165
	pMMG_Obj->promData.crc  	= (uint16_t)( ((uint16_t)promPtr[14] << 8) + (uint16_t)promPtr[15] );
}




/* Reading the uncompensated data */
void pMMG_ReadUncompValue(pMMG_Obj_t* pMMG_Obj) {

	/* Data buffer for sensor replies */
	uint8_t rxData[3];

	/* ---------------------------------------------------------------------------- */

	/* (1) Pressure part */
	pMMG_EnableCS(pMMG_Obj);

	/* Setting the OSR */
	SPITxData = CONVERT_D1_OSR_DEFAULT_CMD | pressureOSR;
	HAL_SPI_Transmit(pMMG_Obj->pMMG_hspi, &SPITxData, 1, 10);

	if (pressureOSR == 0x00) {				// 0.54ms
//		HAL_Delay(1);
//		us_Delay(1000);
		us_Delay(610);
	}
	else if (pressureOSR == 0x02) {			// 1.06ms
//		HAL_Delay(2);
		us_Delay(2000);
	}
	else if (pressureOSR == 0x04) {			// 2.08ms
//		HAL_Delay(3);
		us_Delay(3000);
	}
	else if (pressureOSR == 0x06) {			// 4.13ms
//		HAL_Delay(5);
		us_Delay(5000);
	}
	else {									// 8.22ms
//		HAL_Delay(10);
		us_Delay(10000);
	}

	pMMG_DisableCS(pMMG_Obj);

	/* Reading 24-bit ADC value */
	pMMG_EnableCS(pMMG_Obj);

	SPITxData = READ_ADC_CMD;
	HAL_SPI_Transmit(pMMG_Obj->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj->pMMG_hspi, rxData, 3, 10);

	pMMG_DisableCS(pMMG_Obj);

	/* Convert the 24-bit raw data into a 32-bit useful integer data */
	pMMG_Obj->uncompData.uncompPressure = ( ((uint32_t) rxData[0] << 16) | ((uint32_t) rxData[1] << 8) | ((uint32_t) rxData[2]) );


	/* ---------------------------------------------------------------------------- */


	/* (2) Temperature part */
	pMMG_EnableCS(pMMG_Obj);

	/* Setting the OSR */
	SPITxData = CONVERT_D2_OSR_DEFAULT_CMD | temperatureOSR;
	HAL_SPI_Transmit(pMMG_Obj->pMMG_hspi, &SPITxData, 1, 10);

	if (temperatureOSR == 0x00) {			// 0.54ms
//		HAL_Delay(1);
//		us_Delay(1000);
		us_Delay(610);
	}
	else if (temperatureOSR == 0x02) {		// 1.06ms
//		HAL_Delay(2);
		us_Delay(2000);
	}
	else if (temperatureOSR == 0x04) {		// 2.08ms
//		HAL_Delay(3);
		us_Delay(3000);
	}
	else if (temperatureOSR == 0x06) {		// 4.13ms
//		HAL_Delay(5);
		us_Delay(5000);
	}
	else {									// 8.22ms
//		HAL_Delay(10);
		us_Delay(10000);
	}

	pMMG_DisableCS(pMMG_Obj);

	/* Reading 24-bit ADC value */
	pMMG_EnableCS(pMMG_Obj);

	SPITxData = READ_ADC_CMD;
	HAL_SPI_Transmit(pMMG_Obj->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj->pMMG_hspi, rxData, 3, 10);

	pMMG_DisableCS(pMMG_Obj);


	/* Convert the 24-bit raw data into a 32-bit useful integer data */
	pMMG_Obj->uncompData.uncompTemperature = ( ((uint32_t) rxData[0] << 16) | ((uint32_t) rxData[1] << 8) | ((uint32_t) rxData[2]) );
}


/* Data Conversion */
void pMMG_Convert(pMMG_Obj_t* pMMG_Obj) {
	int32_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;

	dT = pMMG_Obj->uncompData.uncompTemperature - ( (int32_t)(pMMG_Obj->promData.tref << 8) );
	TEMP = 2000 + ( ((int64_t)dT * pMMG_Obj->promData.tempsens) >> 23 );
	OFF = ( ((int64_t)pMMG_Obj->promData.off) << 17 ) + ( ((int64_t)pMMG_Obj->promData.tco * dT) >> 6 );
	SENS = ( ((int64_t)pMMG_Obj->promData.sens) << 16 ) + ( ((int64_t)pMMG_Obj->promData.tcs * dT) >> 7 );


	if (TEMP < 2000) {
		int32_t T2 = ( ((int64_t)dT * (int64_t)dT) >> 31 );
		int32_t TEMPM = TEMP - 2000;
		int64_t OFF2 = ( (61 * (int64_t)TEMPM * (int64_t)TEMPM) >> 4 );
		int64_t SENS2 = ( 2 * (int64_t)TEMPM * (int64_t)TEMPM );

		if (TEMP < -1500) {
			int32_t TEMPP = TEMP + 1500;
			int32_t TEMPP2 = TEMPP * TEMPP;
		    OFF2 = OFF2 + (int64_t)15 * TEMPP2;
		    SENS2 = SENS2 + (int64_t)8 * TEMPP2;
		}
	    TEMP -=  T2;
	    OFF  -=  OFF2;
	    SENS -=  SENS2;
	}

	pMMG_Obj->pMMGData.pressure = ( ((((int64_t)pMMG_Obj->uncompData.uncompPressure * SENS) >> 21) - OFF ) >> 15 );
	pMMG_Obj->pMMGData.temperature = TEMP;
}


/* Update the pMMG sensor reading */
void pMMG_Update(pMMG_Obj_t* pMMG_Obj) {
	pMMG_ReadUncompValue(pMMG_Obj);
	pMMG_Convert(pMMG_Obj);

	pMMG_Obj->pMMGData.pressureKPa = ( (double)(pMMG_Obj->pMMGData.pressure) / 1000.0 );
	pMMG_Obj->pMMGData.temperatureC = ( (double)(pMMG_Obj->pMMGData.temperature) / 100.0 );
}


/* Enable CS Pin */
void pMMG_EnableCS(pMMG_Obj_t* pMMG_Obj) {
	HAL_GPIO_WritePin(pMMG_Obj->pMMG_CS_GPIO_Port, pMMG_Obj->pMMG_CS_Pin, GPIO_PIN_RESET);
}

/* Disable CS Pin */
void pMMG_DisableCS(pMMG_Obj_t* pMMG_Obj) {
	HAL_GPIO_WritePin(pMMG_Obj->pMMG_CS_GPIO_Port, pMMG_Obj->pMMG_CS_Pin, GPIO_PIN_SET);
}



void us_Delay(uint32_t us_delay)
{
    float usStart = (float)DWT->CYCCNT / 170;
    while ( (float)DWT->CYCCNT/170 - usStart < (float)us_delay)
    {
    }
}











/* -----------------------------------------------------------------------  Multiple pMMG in one SPI port version  ----------------------------------------------------------------------------------- */

void pMMG_ReadUncompValue_multiple_3(pMMG_Obj_t* pMMG_Obj1, pMMG_Obj_t* pMMG_Obj2, pMMG_Obj_t* pMMG_Obj3) {

	/* Data buffer for sensor replies */
	uint8_t rxData1[3];
	uint8_t rxData2[3];
	uint8_t rxData3[3];

	/* ---------------------------------------------------------------------------- */

	/* (1) Pressure part */
	pMMG_EnableCS(pMMG_Obj1);
	pMMG_EnableCS(pMMG_Obj2);
	pMMG_EnableCS(pMMG_Obj3);

	/* Setting the OSR */
	SPITxData = CONVERT_D1_OSR_DEFAULT_CMD | pressureOSR;
	HAL_SPI_Transmit(pMMG_Obj1->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Transmit(pMMG_Obj2->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Transmit(pMMG_Obj3->pMMG_hspi, &SPITxData, 1, 10);

	if (pressureOSR == 0x00) {				// 0.54ms
//		HAL_Delay(1);
//		us_Delay(1000);
		us_Delay(610);
	}
	else if (pressureOSR == 0x02) {			// 1.06ms
//		HAL_Delay(2);
		us_Delay(2000);
	}
	else if (pressureOSR == 0x04) {			// 2.08ms
//		HAL_Delay(3);
		us_Delay(3000);
	}
	else if (pressureOSR == 0x06) {			// 4.13ms
//		HAL_Delay(5);
		us_Delay(5000);
	}
	else {									// 8.22ms
//		HAL_Delay(10);
		us_Delay(10000);
	}

	pMMG_DisableCS(pMMG_Obj1);
	pMMG_DisableCS(pMMG_Obj2);
	pMMG_DisableCS(pMMG_Obj3);

	/* Reading 24-bit ADC value */
	pMMG_EnableCS(pMMG_Obj1);
	pMMG_EnableCS(pMMG_Obj2);
	pMMG_EnableCS(pMMG_Obj3);

	SPITxData = READ_ADC_CMD;
	HAL_SPI_Transmit(pMMG_Obj1->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj1->pMMG_hspi, rxData1, 3, 10);
	HAL_SPI_Transmit(pMMG_Obj2->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj2->pMMG_hspi, rxData2, 3, 10);
	HAL_SPI_Transmit(pMMG_Obj3->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj3->pMMG_hspi, rxData3, 3, 10);

	pMMG_DisableCS(pMMG_Obj1);
	pMMG_DisableCS(pMMG_Obj2);
	pMMG_DisableCS(pMMG_Obj3);

	/* Convert the 24-bit raw data into a 32-bit useful integer data */
	pMMG_Obj1->uncompData.uncompPressure = ( ((uint32_t) rxData1[0] << 16) | ((uint32_t) rxData1[1] << 8) | ((uint32_t) rxData1[2]) );
	pMMG_Obj2->uncompData.uncompPressure = ( ((uint32_t) rxData2[0] << 16) | ((uint32_t) rxData2[1] << 8) | ((uint32_t) rxData2[2]) );
	pMMG_Obj3->uncompData.uncompPressure = ( ((uint32_t) rxData3[0] << 16) | ((uint32_t) rxData3[1] << 8) | ((uint32_t) rxData3[2]) );
	/* ---------------------------------------------------------------------------- */


	/* (2) Temperature part */
	pMMG_EnableCS(pMMG_Obj1);
	pMMG_EnableCS(pMMG_Obj2);
	pMMG_EnableCS(pMMG_Obj3);

	/* Setting the OSR */
	SPITxData = CONVERT_D2_OSR_DEFAULT_CMD | temperatureOSR;
	HAL_SPI_Transmit(pMMG_Obj1->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Transmit(pMMG_Obj2->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Transmit(pMMG_Obj3->pMMG_hspi, &SPITxData, 1, 10);

	if (temperatureOSR == 0x00) {				// 0.54ms
//		HAL_Delay(1);
//		us_Delay(1000);
		us_Delay(610);
	}
	else if (temperatureOSR == 0x02) {			// 1.06ms
//		HAL_Delay(2);
		us_Delay(2000);
	}
	else if (temperatureOSR == 0x04) {			// 2.08ms
//		HAL_Delay(3);
		us_Delay(3000);
	}
	else if (temperatureOSR == 0x06) {			// 4.13ms
//		HAL_Delay(5);
		us_Delay(5000);
	}
	else {										// 8.22ms
//		HAL_Delay(10);
		us_Delay(10000);
	}

	pMMG_DisableCS(pMMG_Obj1);
	pMMG_DisableCS(pMMG_Obj2);
	pMMG_DisableCS(pMMG_Obj3);

	/* Reading 24-bit ADC value */
	pMMG_EnableCS(pMMG_Obj1);
	pMMG_EnableCS(pMMG_Obj2);
	pMMG_EnableCS(pMMG_Obj3);

	SPITxData = READ_ADC_CMD;
	HAL_SPI_Transmit(pMMG_Obj1->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj1->pMMG_hspi, rxData1, 3, 10);
	HAL_SPI_Transmit(pMMG_Obj2->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj2->pMMG_hspi, rxData2, 3, 10);
	HAL_SPI_Transmit(pMMG_Obj3->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj3->pMMG_hspi, rxData3, 3, 10);

	pMMG_DisableCS(pMMG_Obj1);
	pMMG_DisableCS(pMMG_Obj2);
	pMMG_DisableCS(pMMG_Obj3);


	/* Convert the 24-bit raw data into a 32-bit useful integer data */
	pMMG_Obj1->uncompData.uncompTemperature = ( ((uint32_t) rxData1[0] << 16) | ((uint32_t) rxData1[1] << 8) | ((uint32_t) rxData1[2]) );
	pMMG_Obj2->uncompData.uncompTemperature = ( ((uint32_t) rxData2[0] << 16) | ((uint32_t) rxData2[1] << 8) | ((uint32_t) rxData2[2]) );
	pMMG_Obj3->uncompData.uncompTemperature = ( ((uint32_t) rxData3[0] << 16) | ((uint32_t) rxData3[1] << 8) | ((uint32_t) rxData3[2]) );
}




void pMMG_Update_multiple_3(pMMG_Obj_t* pMMG_Obj1, pMMG_Obj_t* pMMG_Obj2, pMMG_Obj_t* pMMG_Obj3) {
	pMMG_ReadUncompValue_multiple_3(pMMG_Obj1, pMMG_Obj2, pMMG_Obj3);
	pMMG_Convert(pMMG_Obj1);
	pMMG_Convert(pMMG_Obj2);
	pMMG_Convert(pMMG_Obj3);

	pMMG_Obj1->pMMGData.pressureKPa = ( (double)(pMMG_Obj1->pMMGData.pressure) / 1000.0 );
	pMMG_Obj1->pMMGData.temperatureC = ( (double)(pMMG_Obj1->pMMGData.temperature) / 100.0 );

	pMMG_Obj2->pMMGData.pressureKPa = ( (double)(pMMG_Obj2->pMMGData.pressure) / 1000.0 );
	pMMG_Obj2->pMMGData.temperatureC = ( (double)(pMMG_Obj2->pMMGData.temperature) / 100.0 );

	pMMG_Obj3->pMMGData.pressureKPa = ( (double)(pMMG_Obj3->pMMGData.pressure) / 1000.0 );
	pMMG_Obj3->pMMGData.temperatureC = ( (double)(pMMG_Obj3->pMMGData.temperature) / 100.0 );
}



void pMMG_ReadUncompValue_multiple_2(pMMG_Obj_t* pMMG_Obj1, pMMG_Obj_t* pMMG_Obj2) {

	/* Data buffer for sensor replies */
	uint8_t rxData1[3];
	uint8_t rxData2[3];

	/* ---------------------------------------------------------------------------- */

	/* (1) Pressure part */
	pMMG_EnableCS(pMMG_Obj1);
	pMMG_EnableCS(pMMG_Obj2);

	/* Setting the OSR */
	SPITxData = CONVERT_D1_OSR_DEFAULT_CMD | pressureOSR;
	HAL_SPI_Transmit(pMMG_Obj1->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Transmit(pMMG_Obj2->pMMG_hspi, &SPITxData, 1, 10);

	if (pressureOSR == 0x00) {				// 0.54ms
//		HAL_Delay(1);
//		us_Delay(1000);
		us_Delay(610);
	}
	else if (pressureOSR == 0x02) {			// 1.06ms
//		HAL_Delay(2);
		us_Delay(2000);
	}
	else if (pressureOSR == 0x04) {			// 2.08ms
//		HAL_Delay(3);
		us_Delay(3000);
	}
	else if (pressureOSR == 0x06) {			// 4.13ms
//		HAL_Delay(5);
		us_Delay(5000);
	}
	else {									// 8.22ms
//		HAL_Delay(10);
		us_Delay(10000);
	}

	pMMG_DisableCS(pMMG_Obj1);
	pMMG_DisableCS(pMMG_Obj2);

	/* Reading 24-bit ADC value */
	pMMG_EnableCS(pMMG_Obj1);
	pMMG_EnableCS(pMMG_Obj2);

	SPITxData = READ_ADC_CMD;
	HAL_SPI_Transmit(pMMG_Obj1->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj1->pMMG_hspi, rxData1, 3, 10);
	HAL_SPI_Transmit(pMMG_Obj2->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj2->pMMG_hspi, rxData2, 3, 10);

	pMMG_DisableCS(pMMG_Obj1);
	pMMG_DisableCS(pMMG_Obj2);

	/* Convert the 24-bit raw data into a 32-bit useful integer data */
	pMMG_Obj1->uncompData.uncompPressure = ( ((uint32_t) rxData1[0] << 16) | ((uint32_t) rxData1[1] << 8) | ((uint32_t) rxData1[2]) );
	pMMG_Obj2->uncompData.uncompPressure = ( ((uint32_t) rxData2[0] << 16) | ((uint32_t) rxData2[1] << 8) | ((uint32_t) rxData2[2]) );
	/* ---------------------------------------------------------------------------- */


	/* (2) Temperature part */
	pMMG_EnableCS(pMMG_Obj1);
	pMMG_EnableCS(pMMG_Obj2);

	/* Setting the OSR */
	SPITxData = CONVERT_D2_OSR_DEFAULT_CMD | temperatureOSR;
	HAL_SPI_Transmit(pMMG_Obj1->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Transmit(pMMG_Obj2->pMMG_hspi, &SPITxData, 1, 10);

	if (temperatureOSR == 0x00) {				// 0.54ms
//		HAL_Delay(1);
//		us_Delay(1000);
		us_Delay(610);
	}
	else if (temperatureOSR == 0x02) {			// 1.06ms
//		HAL_Delay(2);
		us_Delay(2000);
	}
	else if (temperatureOSR == 0x04) {			// 2.08ms
//		HAL_Delay(3);
		us_Delay(3000);
	}
	else if (temperatureOSR == 0x06) {			// 4.13ms
//		HAL_Delay(5);
		us_Delay(5000);
	}
	else {										// 8.22ms
//		HAL_Delay(10);
		us_Delay(10000);
	}

	pMMG_DisableCS(pMMG_Obj1);
	pMMG_DisableCS(pMMG_Obj2);

	/* Reading 24-bit ADC value */
	pMMG_EnableCS(pMMG_Obj1);
	pMMG_EnableCS(pMMG_Obj2);

	SPITxData = READ_ADC_CMD;
	HAL_SPI_Transmit(pMMG_Obj1->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj1->pMMG_hspi, rxData1, 3, 10);
	HAL_SPI_Transmit(pMMG_Obj2->pMMG_hspi, &SPITxData, 1, 10);
	HAL_SPI_Receive(pMMG_Obj2->pMMG_hspi, rxData2, 3, 10);

	pMMG_DisableCS(pMMG_Obj1);
	pMMG_DisableCS(pMMG_Obj2);


	/* Convert the 24-bit raw data into a 32-bit useful integer data */
	pMMG_Obj1->uncompData.uncompTemperature = ( ((uint32_t) rxData1[0] << 16) | ((uint32_t) rxData1[1] << 8) | ((uint32_t) rxData1[2]) );
	pMMG_Obj2->uncompData.uncompTemperature = ( ((uint32_t) rxData2[0] << 16) | ((uint32_t) rxData2[1] << 8) | ((uint32_t) rxData2[2]) );
}




void pMMG_Update_multiple_2(pMMG_Obj_t* pMMG_Obj1, pMMG_Obj_t* pMMG_Obj2) {
	pMMG_ReadUncompValue_multiple_2(pMMG_Obj1, pMMG_Obj2);
	pMMG_Convert(pMMG_Obj1);
	pMMG_Convert(pMMG_Obj2);

	pMMG_Obj1->pMMGData.pressureKPa = ( (double)(pMMG_Obj1->pMMGData.pressure) / 1000.0 );
	pMMG_Obj1->pMMGData.temperatureC = ( (double)(pMMG_Obj1->pMMGData.temperature) / 100.0 );

	pMMG_Obj2->pMMGData.pressureKPa = ( (double)(pMMG_Obj2->pMMGData.pressure) / 1000.0 );
	pMMG_Obj2->pMMGData.temperatureC = ( (double)(pMMG_Obj2->pMMGData.temperature) / 100.0 );
}









