/*
 * mma8452x.h
 *
 *  Created on: 17.06.2023..
 *      Author: Artem Kagirov
 */

#ifndef INC_MMA8452X_H_
#define INC_MMA8452X_H_

#include "stm32f4xx_hal.h"

#define STATUS  0x00
#define OUT_X_MSB  0x01
#define OUT_X_LSB  0x02
#define OUT_Y_MSB  0x03
#define OUT_Y_LSB  0x04
#define OUT_Z_MSB  0x05
#define OUT_Z_LSB  0x06
#define SYSMOD  0x0B
#define INT_SOURCE  0x0C
#define WHO_AM_I  0x0D

#define FF_MT_CFG 0x15
#define FF_MT_CFG_ELE (1<<7)
#define FF_MT_CFG_OAE (1<<6)
#define FF_MT_CFG_ZEFE (1<<5)
#define FF_MT_CFG_YEFE (1<<4)
#define FF_MT_CFG_XEFE (1<<3)

#define FF_MT_SRC 0x16
#define FF_MT_SRC_EA (1<<7)
#define FF_MT_SRC_ZHE (1<<5)
#define FF_MT_SRC_ZHP (1<<4)
#define FF_MT_SRC_YHE (1<<3)
#define FF_MT_SRC_YHP (1<<2)
#define FF_MT_SRC_XHE (1<<1)
#define FF_MT_SRC_XHP (1<<0)

#define FF_MT_THS 0x17
#define FF_MT_THS_DBCNTM (1<<7)

#define FF_MT_COUNT 0x18

#define CTRL_REG1  0x2A
#define CTRL_REG1_ACTIVE (1<<0)
#define CTRL_REG1_F_READ (1<<1)

#define CTRL_REG3 0x2C
#define CTRL_REG3_IPOL (1<<1)

#define CTRL_REG4 0x2D
#define CTRL_REG4_ASLP (1<<7)
#define EN_ASLP 6
#define CTRL_REG4_TRANS (1<<5)
#define EN_TRANS 5
#define CTRL_REG4_LNDPRT (1<<4)
#define EN_LNDPRT 4
#define CTRL_REG4_PULSE (1<<3)
#define EN_PULSE 3
#define CTRL_REG4_FF_MT (1<<2)
#define EN_FF_MT 2
#define CTRL_REG4_DRDY (1<<0)
#define EN_DRDY 1
#define EN_NONE 0

#define CTRL_REG5 0x2E
#define CTRL_REG5_ASLP (1<<7)
#define CFG_ASLP 6
#define CTRL_REG5_TRANS (1<<5)
#define CFG_TRANS 5
#define CTRL_REG5_LNDPRT (1<<4)
#define CFG_LNDPRT 4
#define CTRL_REG5_PULSE (1<<3)
#define CFG_PULSE 3
#define CTRL_REG5_FF_MT (1<<2)
#define CFG_FF_MT 2
#define CTRL_REG5_DRDY (1<<0)
#define CFG_DRDY 1
#define CFG_DEFAULT 0

void mma8452x_Standby(I2C_HandleTypeDef * hi2c, uint16_t DevAddress);
void mma8452x_Active(I2C_HandleTypeDef * hi2c, uint16_t DevAddress);
void mma8452x_DataFormat(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t d_format);
void mma8452x_DataRateSelection(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t d_rate);
void mma8452x_InterruptPolarityConfig(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t ipol);
void mma8452x_InterruptEnable(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint8_t int_en, uint8_t int_cfg);
void mma8452x_InterruptDisableAll(I2C_HandleTypeDef * hi2c, uint16_t DevAddress);
void mma8452x_MotionDetectionConfig(I2C_HandleTypeDef * hi2c,
									uint16_t DevAddress,
									uint8_t byte_cfg,
									uint8_t dbcntm,
									uint8_t threshold,
									uint8_t d_count);
void mma8452x_ReadData(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData);

#endif /* INC_MMA8452X_H_ */
