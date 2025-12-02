	/**
  ******************************************************************************
  * @file    i2c_opt.h
  * @author  MCD Application Team
  * @version V0.0.3
  * @date    Feb 2010
  * @brief   This file contains definitions for optimized I2C software
  ******************************************************************************
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  *                     COPYRIGHT 2009 STMicroelectronics  
  */ 

/* Define to prevent recursive inclusion */

//////#define NO_RESTART
////////////////?????


#ifndef __I2C_OPT_H
#define __I2C_OPT_H

#include "stm8l15x.h"

// ************************** I2C Configuration Variables **************************

/* definition of fast or default standard mode (bus speed up to 400 or 100 kHz) */
//#define FAST_I2C_MODE

/* uncomment next line when stop request is required between device address sent and read data */
//#define NO_RESTART

extern int8_t i2cerr;					// I2C error flag

// ************************* Function Declaration ***************************
void I2C_Init(void);
//int8_t I2C_Read(u8 u8_i2cAddr, u8 u8_NumByteToRead, u8 *u8_ReadBuffer); 
//int8_t I2C_Read_DAC63508(u8 u8_i2cAddr, u8 u8_CommandByte, u8 *u8_DataBuffer);
int8_t I2C_Write(u8 u8_i2cAddr, u8 u8_NumByteToWrite, u8 *u8_DataBuffer);


#define dead_time() { _asm("nop"); _asm("nop"); }

#endif /* __I2C_OPT_H */
