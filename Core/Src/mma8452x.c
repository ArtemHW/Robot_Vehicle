/*
 * mma8452x.c
 *
 *  Created on: 17.06.2023.
 *      Author: Artem Kagirov
 */
#include "mma8452x.h"

/*
 * @brief mma8452x_Standby - activate Standby mode
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 */
void mma8452x_Standby(I2C_HandleTypeDef * hi2c, uint16_t DevAddress)
{
	uint8_t register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	register_value &= ~CTRL_REG1_ACTIVE;
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
}

/*
 * @brief mma8452x_Active - activate Active mode
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 */
void mma8452x_Active(I2C_HandleTypeDef * hi2c, uint16_t DevAddress)
{
	uint8_t register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	register_value |= CTRL_REG1_ACTIVE;
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
}

/*
 * @brief mma8452x_DataFormat - Set the data format
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  d_format If d_format is 1 then Data format limited to single Byte.
 * 		   If d_format is 0 then Data format limited to 12 Bits.
 */
void mma8452x_DataFormat(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t d_format)
{
	mma8452x_Standby(hi2c, DevAddress);
	uint8_t register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	if(d_format == 1)
	{
		register_value |= CTRL_REG1_F_READ;
	}
	else
	{
		register_value &= ~CTRL_REG1_F_READ;
	}
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	mma8452x_Active(hi2c, DevAddress);
}

/*
 * @brief mma8452x_DataRateSelection - Select the data rate
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  d_rate Value from 0 to 7 for selection output data rate for acceleration samples.
 * 		   The default value is 000 for a data rate of 800 Hz.
 */
void mma8452x_DataRateSelection(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t d_rate)
{
	mma8452x_Standby(hi2c, DevAddress);
	uint8_t register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	register_value &= ~((7<<3));
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	register_value |= (d_rate<<3);
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	mma8452x_Active(hi2c, DevAddress);
}

/*
 * @brief mma8452x_InterruptPolarityConfig - Select the polarity of the interrupt signal
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  ipol Interrupt polarity ACTIVE high, or ACTIVE low. Default value: 0.
 *         0: ACTIVE low; 1: ACTIVE high
 */
void mma8452x_InterruptPolarityConfig(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t ipol)
{
	mma8452x_Standby(hi2c, DevAddress);
	uint8_t register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)CTRL_REG3, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	if(ipol == 1)
	{
		register_value |= CTRL_REG3_IPOL;
	}
	else
	{
		register_value &= ~CTRL_REG3_IPOL;
	}
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG3, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	mma8452x_Active(hi2c, DevAddress);
}

/*
 * @brief mma8452x_InterruptEnable - Enable selected interrupt
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  int_en The corresponding interrupt
 * @param  int_cfg The corresponding interrupt pin
 */
void mma8452x_InterruptEnable(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t int_en, uint8_t int_cfg)
{
	mma8452x_Standby(hi2c, DevAddress);
	uint8_t register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)CTRL_REG4, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	switch (int_en)
	{
		 case EN_ASLP:
			 register_value |= CTRL_REG4_ASLP;
			 break;
		 case EN_TRANS:
			 register_value |= CTRL_REG4_TRANS;
			 break;
		 case EN_LNDPRT:
			 register_value |= CTRL_REG4_LNDPRT;
			 break;
		 case EN_PULSE:
			 register_value |= CTRL_REG4_PULSE;
			 break;
		 case EN_FF_MT:
			 register_value |= CTRL_REG4_FF_MT;
			 break;
		 case EN_DRDY:
			 register_value |= CTRL_REG4_DRDY;
			 break;
		 default:
        	 __asm__ volatile("NOP");
	}
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG4, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)CTRL_REG5, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	switch (int_cfg)
	{
		 case CFG_ASLP:
			 register_value |= CTRL_REG5_ASLP;
			 break;
		 case CFG_TRANS:
			 register_value |= CTRL_REG5_TRANS;
			 break;
		 case CFG_LNDPRT:
			 register_value |= CTRL_REG5_LNDPRT;
			 break;
		 case CFG_PULSE:
			 register_value |= CTRL_REG5_PULSE;
			 break;
		 case CFG_FF_MT:
			 register_value |= CTRL_REG5_FF_MT;
			 break;
		 case CFG_DRDY:
			 register_value |= CTRL_REG5_DRDY;
			 break;
		 default:
			 __asm__ volatile("NOP");
	}
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG5, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	mma8452x_Active(hi2c, DevAddress);
}

/*
 * @brief mma8452x_InterruptDisableAll - Disable all interrupt
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 */
void mma8452x_InterruptDisableAll(I2C_HandleTypeDef * hi2c, uint16_t DevAddress)
{
	mma8452x_Standby(hi2c, DevAddress);
	uint8_t register_value = 0x0;
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG4, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)CTRL_REG5, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	mma8452x_Active(hi2c, DevAddress);
}

/*
 * @brief mma8452x_MotionDetectionConfig - Freefall/Motion configuration
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  byte_cfg The Freefall/Motion configuration register for setting up
 *         the conditions of the freefall or motion function.
 * @param  dbcntm Debounce counter mode selection. Default value: 0.
 *         0: increments or decrements debounce, 1: increments or clears counter.
 * @param  threshold Freefall /Motion Threshold: Default value: 000_0000.
 * 		   the threshold register has a range of 0 to 127
 * @param  d_count number of debounce sample counts for the event trigger
 */
void mma8452x_MotionDetectionConfig(I2C_HandleTypeDef * hi2c,
									uint16_t DevAddress,
									uint8_t byte_cfg,
									uint8_t dbcntm,
									uint8_t threshold,
									uint8_t d_count)
{
	mma8452x_Standby(hi2c, DevAddress);
	uint8_t register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)FF_MT_CFG, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	register_value |= byte_cfg;
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)FF_MT_CFG, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)FF_MT_THS, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	if(dbcntm == 1)
	{
		register_value |= FF_MT_THS_DBCNTM;
	}
	else if(dbcntm == 0)
	{
		register_value &= ~FF_MT_THS_DBCNTM;
	}
	register_value &= ~127;
	register_value |= threshold;
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)FF_MT_THS, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	register_value = 0x0;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)FF_MT_COUNT, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	register_value = d_count;
	HAL_I2C_Mem_Write(hi2c, DevAddress, (uint16_t)FF_MT_COUNT, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	mma8452x_Active(hi2c, DevAddress);
}

/*
 * @brief mma8452x_ReadData - Read data from Data registers of MMA8452x
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  pData Pointer to data buffer
 */
void mma8452x_ReadData(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData)
{
	uint8_t register_value = 0x0;
	uint16_t size_of_data;
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)CTRL_REG1, 1, (uint8_t*)&register_value, sizeof(uint8_t), 100);
	if((register_value & CTRL_REG1_F_READ) == CTRL_REG1_F_READ)
	{
		size_of_data = 3;
	}
	else
	{
		size_of_data = 6;
	}
	HAL_I2C_Mem_Read(hi2c, DevAddress, (uint16_t)OUT_X_MSB, 1, (uint8_t*)pData, size_of_data, 100);
}
