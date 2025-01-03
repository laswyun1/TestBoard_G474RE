/*
 * SpO2.c
 *
 *  Created on: Dec 23, 2024
 *      Author: INVINCIBLE
 */

#include "SpO2.h"

/* I2C Received Data */
static uint8_t I2CRxData[READ_DATA_LENGTH_MAX] = {0};
static uint8_t I2CTxData[WRITE_DATA_LENGTH_MAX] = {0};


SPO2_State_t SPO2_Init(SPO2_Obj_t* spo2_Obj, I2C_HandleTypeDef* hi2c)
{
	spo2_Obj->SPO2_i2c = hi2c;
	spo2_Obj->devReadAddr = MAX30102_I2C_READ_ADDR;
	spo2_Obj->devWriteAddr = MAX30102_I2C_WRITE_ADDR;

	if (ReadPartID(spo2_Obj) != MAX_30102_EXPECTEDPARTID) {
		return SPO2_STATE_ERROR;
	}

	ReadRevisionID(spo2_Obj);

	return SPO2_STATE_OK;
}


SPO2_State_t SPO2_Setup(SPO2_Obj_t* spo2_Obj, )
{

}


void bitMask(SPO2_Obj_t* spo2_Obj, uint8_t regAddr, uint8_t mask, uint8_t thing)
{
	uint8_t originalData = 0;

	if (ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, regAddr, I2CRxData, 1) == 0){
		originalData = I2CRxData[0];

		originalData = originalData & mask;


		I2CTxData = originalData | thing
		WriteReg(spo2_Obj->SPO2_i2c, spo2_Obj->devWriteAddr, regAddr, I2CTxData, 1)
	}
}


uint8_t WriteReg(I2C_HandleTypeDef* hi2c, uint8_t devWriteAddr, uint8_t regAddr, uint8_t* writeBuff, uint8_t size)
{
	uint8_t state = 0;

	state = HAL_I2C_Mem_Write(hi2c, devWriteAddr, regAddr, I2C_MEM_ADD_SIZE, writeBuff, size, I2C_TIMEOUT);

	return state;
}


uint8_t ReadReg(I2C_HandleTypeDef* hi2c, uint8_t devReadAddr, uint8_t regAddr, uint8_t* readBuff, uint8_t size)
{
	uint8_t state = 0;

	state = HAL_I2C_Mem_Read(hi2c, devReadAddr, regAddr, I2C_MEM_ADD_SIZE, readBuff, size, I2C_TIMEOUT);

	return state;
}

uint8_t ReadPartID(SPO2_Obj_t* spo2_Obj)
{
	uint8_t state = 0;
	uint8_t partID = 0;

	state = ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_PART_ID, I2CRxData, 1);

	if (state == 0){
		partID = I2CRxData[0];
	}

	return partID;
}

uint8_t ReadRevisionID(SPO2_Obj_t* spo2_Obj)
{
	uint8_t state = 0;
	uint8_t revisionID = 0;

	state = ReadReg(spo2_Obj->SPO2_i2c, spo2_Obj->devReadAddr, MAX30102_REVISION_ID, I2CRxData, 1);

	if (state == 0){
		revisionID = I2CRxData[0];
	}

	return revisionID;
}



