// based on i2c_master_poll.c by MCD Application Team V0.0.3

#include "i2c_master_poll.h"

volatile uint16_t i2ctocnt,i2ctocnt2;     		// I2C timeout counter
//#define I2CTO 511					// 511-> ~>5ms @2MHz
#define I2CTO 4095					// 4095-> ~>5ms @16MHz
void clrto(void) {i2ctocnt=I2CTO;}			// clear timeout counter
bool tout(void) {if(--i2ctocnt == 0) return 1; else return 0;}	// decrement TO counter

void clrto2(void) {i2ctocnt2=I2CTO;}			// clear timeout counter
bool tout2(void) {if(--i2ctocnt2 == 0) return 1; else return 0;}	// decrement TO counter


//******************************************************************************
// Function name : I2C_Init
// Description   : Initialize I2C peripheral
// Input param   : None
// Return        : None
//******************************************************************************//
void I2C_Init(void) {
  GPIOC->ODR |= 3;                	//define SDA, SCL outputs, HiZ, Open drain, Fast
  GPIOC->DDR |= 3;
  GPIOC->CR2 |= 3;

  CLK->PCKENR1 |= 8;            	// I2C clock enable

//  I2C1->FREQR = 2;                	// input clock to I2C - 2MHz 
  I2C1->FREQR = 16;               	// input clock to I2C - 16MHz
//  I2C1->CCRL = 20;                	// CCR= 20 -> 50KHz @ I2C1->FREQR = 2; and cpu freq 2MHz
//  I2C1->CCRL = 40;                	// CCR= 40 - (SCLhi must be at least 4000+1000=5000ns!) 25KHz @ I2C1->FREQR = 2; and cpu freq 2MHz
  I2C1->CCRL = 0x50;              	// 0x50->100KHz 0x60->83.3 0x80->62.5  0xff->31.3 - see manual
  I2C1->CCRH = 0;                 	// standard mode, duty 1/1 bus speed 100kHz
  I2C1->TRISER = 9;               	// 1000ns/(125ns) + 1  (maximum 1000ns)

  I2C1->OARL = 0xA0;              	// own address A0;
  I2C1->OARH |= 0x40;
  I2C1->ITR = 1;                  	// enable error interrupts
  I2C1->CR2 |= 0x04;              	// ACK=1, Ack enable
  I2C1->CR1 |= 0x01;              	// PE=1 - Peripheral Enable
}

//******************************************************************************
// Function name : I2C_Read
// Description   : Read defined number bytes from slave 
// Input param   : number of bytes to read, starting address to store received data
// Return        : 0 - OK, !=0 error
//******************************************************************************//
/*
int8_t I2C_Read(u8 u8_i2cAddr, u8 u8_NumByteToRead, u8 *u8_DataBuffer)
{
  //--------------- BUSY? -> STOP request ---------------------//
  clrto2(); 
  while((I2C1->SR3 & 2))                                              	// Wait while the bus is busy 
  	{I2C1->CR2 |= 2;                                                  	// STOP=1, generate stop
  	clrto(); while((I2C1->CR2 & 2)) {if(tout()) {i2cerr=0; return 1;}};	// wait until stop is performed
  	if(tout2()) {i2cerr=0; return 2;}
  	}
  I2C1->CR2 |= I2C_CR2_ACK;                                                           // ACK=1, Ack enable
  //--------------- Start communication -----------------------//
  I2C1->CR2 |= 1;                                                     	// START=1, generate start
  clrto(); while(((I2C1->SR1 & 1)==0)){if(tout()) {i2cerr=0; return 3;}};      	// Wait for start bit detection (SB)

  //------------------ Address send ---------------------------//
	I2C1->DR = (u8)((u8_i2cAddr << 1) | 1);			// Send device address & Read (R/-W = 1 i.e. Reading)
  clrto(); while(!(I2C1->SR1 & 2)){if(tout()) {i2cerr=0; return 5;}};          	// Wait for address ack (ADDR)

  //------------------- Data Receive --------------------------//
  if (u8_NumByteToRead > 2)                                           	// *** more than 2 bytes are received? ***
  {
    I2C1->SR3;                                                        	// ADDR clearing sequence
  	clrto2(); 
    while(u8_NumByteToRead > 3)                                       	// not last three bytes?
    {
      clrto(); while(!(I2C1->SR1 & I2C_SR1_BTF)) {if(tout()) {i2cerr=0; return 6;}}; 	// Wait for BTF
      *u8_DataBuffer++ = I2C1->DR;                                    	// Reading next data byte
      --u8_NumByteToRead;                                             	// Decrease Numbyte to reade by 1
  	if(tout2()) {i2cerr=0; return 7;}
    }
                                                                      	//last three bytes should be read
    clrto(); while(!(I2C1->SR1 & I2C_SR1_BTF));  {if(tout()) {i2cerr=0; return 8;}}	// Wait for BTF
    I2C1->CR2 &=(uint8_t)(~I2C_CR2_ACK);                              	// Clear ACK  // Disable the acknowledgement  - from spl //
    disableInterrupts();                                              	// Errata workaround (Disable interrupt)
    *u8_DataBuffer++ = I2C1->DR;                                      	// Read 1st byte
    I2C1->CR2 |= I2C_CR2_STOP;                                        	// Generate stop here (STOP=1)
    *u8_DataBuffer++ = I2C1->DR;                                      	// Read 2nd byte
    enableInterrupts();                                               	// Errata workaround (Enable interrupt)
    clrto(); while(!(I2C1->SR1 & I2C_SR1_RXNE));  {if(tout()) {i2cerr=0; return 9;}} 	// Wait for RXNE
    *u8_DataBuffer++ = I2C1->DR;                                      	// Read 3rd Data byte
  }
  else
  {
    if(u8_NumByteToRead == 2)                                         	// just two bytes are received? 
    {
      I2C1->CR2 |= I2C_CR2_POS;                                       	// Set POS bit (NACK at next received byte)
      I2C1->SR3;                                                      	// Clear ADDR Flag
      I2C1->CR2 &=(uint8_t)(~I2C_CR2_ACK);                            	// Clear ACK
    	clrto(); while(!(I2C1->SR1 & I2C_SR1_BTF)){if(tout()) {i2cerr=0; return 10;}}; // Wait for BTF
      
      I2C1->CR2 |= I2C_CR2_STOP;                                      	// Generate stop here (STOP=1)
      *u8_DataBuffer++ = I2C1->DR;                            	        	// Read 1st Data byte
      *u8_DataBuffer = I2C1->DR;                                      	// Read 2nd Data byte
    }
    else                                                              	// *** only one byte is received ***
    {
      I2C1->CR2 &=(uint8_t)(~I2C_CR2_ACK);                            	// Clear ACK
      I2C1->SR3;                                                      	// Clear ADDR Flag
      I2C1->CR2 |= I2C_CR2_STOP;                                      	// generate stop here (STOP=1)
      clrto(); while(!(I2C1->SR1 & I2C_SR1_RXNE)){if(tout()) {i2cerr=0; return 11;}}; // test EV7, Wait for RXNE
      *u8_DataBuffer = I2C1->DR;                                      	// Read Data byte
    }
  }
  //--------------- All Data Received -----------------------//
 	clrto(); while((I2C1->CR2 & I2C_CR2_STOP)) {if(tout()) {i2cerr=0; return 12;}}; 	// wait until stop is performed (STOPF = 1)
  I2C1->CR2 &=(uint8_t)(~I2C_CR2_POS);                                	// return POS to default state (POS=0)
  return 0;
}
*/
//*****************************************************************************
// Function name : I2C_Write (working with 8 bit addr. only)
// Description   : write defined number bytes to slave 
// Input param   : I2C_address, number of bytes to write, starting address to send
// Return        : 0 - OK, !=0 error
//******************************************************************************//
int8_t I2C_Write(u8 u8_i2cAddr, u8 u8_NumByteToWrite, u8 *u8_DataBuffer)
{
  clrto2(); 
  while((I2C1->SR3 & 2))                                              	// Wait while the bus is busy 
  	{I2C1->CR2 |= 2;                                                  	// STOP=1, generate stop
  	clrto(); while((I2C1->CR2 & 2)) {if(tout()) {i2cerr=0; return 1;}};	// wait until stop is performed
  	if(tout2()) {i2cerr=0; return 2;}
  	}

  I2C1->CR2 |= 1;                                                     	// START=1, generate start
  clrto(); while(((I2C1->SR1 & 1)==0)){if(tout()) {i2cerr=0; return 3;}};      	// Wait for start bit detection (SB)
  dead_time();                                                        	// SB clearing sequence
	if (i2cerr) {i2cerr=0; return 4;} 

	I2C1->DR = (u8)(u8_i2cAddr << 1);				// Send device address & Write(R/-W = 0 i.e. Writing)
  clrto(); while(!(I2C1->SR1 & 2)){if(tout()) {i2cerr=0; return 5;}};          	// Wait for address ack (ADDR)
  dead_time();                                                        	// ADDR clearing sequence
  I2C1->SR3;

  clrto(); while(!(I2C1->SR1 & 0x80)){if(tout()) {i2cerr=0; return 6;}};         	// Wait for TxE
	if (i2cerr) {i2cerr=0; return 7;} 

  if(u8_NumByteToWrite)
  	{while(u8_NumByteToWrite--)				// write data loop start
    	{clrto(); while(!(I2C1->SR1 & 0x80)) {if(tout()) {i2cerr=0; return 8;}};  	// test EV8 - wait for TxE
      I2C1->DR = *u8_DataBuffer++;                                    	// send next data byte
    	}                                                               	// write data loop end
  	}
  
  clrto(); while(((I2C1->SR1 & 0x84) != 0x84)) {if(tout()) {i2cerr=0; return 9;}};	// Wait for TxE & BTF
  dead_time();                                                       		// clearing sequence

  I2C1->CR2 |= 2;                                                     	// generate stop here (STOP=1)
  clrto(); while((I2C1->CR2 & 2)) {if(tout()) {i2cerr=0; return 0xa;}};           	// wait until stop is performed
  return 0;
}

//*****************************************************************************
// Function name : I2C_Read_DAC63508 - somehow works 
// Description   : Read TWO bytes from DAC63508 slave 
// Input param   : i2cAddr, DAC63508 Command Byte , starting address to store received data
// Return        : 0 - OK, !=0 error
//******************************************************************************//
/*
int8_t I2C_Read_DAC63508(u8 u8_i2cAddr, u8 u8_CommandByte, u8 *u8_DataBuffer)
{
//--------------- BUSY? -> STOP request ---------------------//
  clrto2(); 
  while((I2C1->SR3 & 2))                                              	// Wait while the bus is busy 
  	{I2C1->CR2 |= 2;                                                  	// STOP=1, generate stop
  	clrto(); while((I2C1->CR2 & 2)) {if(tout()) {i2cerr=0; return 1;}};	// wait until stop is performed
  	if(tout2()) {i2cerr=0; return 2;}
  	}
  I2C1->CR2 |= I2C_CR2_ACK;                                                           // ACK=1, Ack enable

//--------------- Start communication -----------------------//
  I2C1->CR2 |= 1;                                                     	// START=1, generate start
  clrto(); while(((I2C1->SR1 & 1)==0)){if(tout()) {i2cerr=0; return 3;}};      	// Wait for start bit detection (SB)
  dead_time();                                                        	// SB clearing sequence
	if (i2cerr) {i2cerr=0; return 4;} 

//------------------ Address for writing send ---------------------------//
	I2C1->DR = (u8)(u8_i2cAddr << 1);				// Send device address & Write(R/-W = 0 i.e. Writing)
  clrto(); while(!(I2C1->SR1 & 2)){if(tout()) {i2cerr=0; return 5;}};          	// Wait for address ack (ADDR)
  dead_time();                                                        	// ADDR clearing sequence
  I2C1->SR3;

  clrto(); while(!(I2C1->SR1 & 0x80)){if(tout()) {i2cerr=0; return 6;}};         	// Wait for TxE
	if (i2cerr) {i2cerr=0; return 7;} 
//------------------ Start sending command byte ---------------------------//
	clrto(); while(!(I2C1->SR1 & 0x80)) {if(tout()) {i2cerr=0; return 8;}};  	// test EV8 - wait for TxE
  I2C1->DR = u8_CommandByte;                                    		// send commnad byte
  
  clrto(); while(((I2C1->SR1 & 0x84) != 0x84)) {if(tout()) {i2cerr=0; return 9;}};	// Wait for TxE & BTF
  dead_time();                                                       		// clearing sequence

//--------------- REStart communication -----------------------//
  I2C1->CR2 |= 1;                                                     	// START=1, generate start
  clrto(); while(((I2C1->SR1 & 1)==0)){if(tout()) {i2cerr=0; return 10;}};      	// Wait for start bit detection (SB)

//------------------ Address send ---------------------------//
	I2C1->DR = (u8)((u8_i2cAddr << 1) | 1);			// Send device address & Read (R/-W = 1 i.e. Reading)
  clrto(); while(!(I2C1->SR1 & 2)){if(tout()) {i2cerr=0; return 11;}};          	// Wait for address ack (ADDR)

//------------------- Data Receive --------------------------//
      I2C1->CR2 |= I2C_CR2_POS;                      		// Set POS bit (NACK at next received byte)
      disableInterrupts();                          		// Errata workaround (Disable interrupt)
      I2C1->SR3;                                       	// Clear ADDR Flag
      I2C1->CR2 &=(uint8_t)(~I2C_CR2_ACK);                        	// Clear ACK
      enableInterrupts();																// Errata workaround (Enable interrupt)
    	clrto(); while(!(I2C1->SR1 & I2C_SR1_BTF)){if(tout()) {i2cerr=0; return 12;}}; 	// Wait for BTF
      disableInterrupts();                          		// Errata workaround (Disable interrupt)
      I2C1->CR2 |= I2C_CR2_STOP;                       	// Generate stop here (STOP=1)
      *u8_DataBuffer++ = I2C1->DR;                     	// Read 1st Data byte
      enableInterrupts();																// Errata workaround (Enable interrupt)
			*u8_DataBuffer = I2C1->DR;													// Read 2nd Data byte

//--------------- Data Received -----------------------//
 	clrto(); while((I2C1->CR2 & I2C_CR2_STOP)) {if(tout()) {i2cerr=0; return 14;}}; 	// wait until stop is performed (STOPF = 1)
  I2C1->CR2 &=(uint8_t)(~I2C_CR2_POS);                                	// return POS to default state (POS=0)
  return 0;
}
*/
