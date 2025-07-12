/*****************************************************************************
 *   i2ctest.c:  main C entry file for NXP LPC11xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.12.07  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#include "LPC17xx.h"			/* LPC17xx Peripheral Registers */
#include "type.h"
#include "i2c.h"
#include "i2ctest.h"
#include "global.h"
#include <string.h>

#define MC24LC64_ADDR 0xa0		/* I2C address of the 24LC64 EEPROM */

/*******************************************************************************
**   Main Function  main()
*******************************************************************************/
int write_byte_to_24LC64_EEPROM(unsigned int address, unsigned char databyte, unsigned char bytes){
	
	if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		return 1;				/* Fatal error */
	}
	
	I2CWriteLength = bytes;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = MC24LC64_ADDR;
	I2CMasterBuffer[1] = address >> 8;			/* address MSB */
	I2CMasterBuffer[2] = address;				/* address LSB */
	I2CMasterBuffer[3] = databyte;				/* address LSB */	
	uint32_t fin_state = I2CEngine();
	
	return fin_state;
}


int read_byte_from_24LC64_EEPROM(unsigned int address, unsigned char bytes){
	uint32_t i;
	if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		return 1;				/* Fatal error */
	}
	for ( i = 0; i < Slave_Buffer_BUFSIZE; i++ ){
		I2CSlaveBuffer[i] = 0x00;
	}
	/* Write SLA(W), address, SLA(R), and read 4 bytes back. */

	I2CWriteLength = 3;
	I2CReadLength = bytes;
	I2CMasterBuffer[0] = MC24LC64_ADDR;
	I2CMasterBuffer[1] = address >> 8;			/* address MSB */
	I2CMasterBuffer[2] = address;				/* address LSB */
	I2CMasterBuffer[3] = MC24LC64_ADDR | RD_BIT;

	i=0;
	while (I2CEngine() == I2CSTATE_SLA_NACK)
		i++;

	return 0;
}

int write_page_to_24LC64_EEPROM(unsigned int address, unsigned char* pdata, unsigned char bytes){
	unsigned int i;
	unsigned char temp_str[bytes];
	if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		return 1;				/* Fatal error */
	}	
	
	I2CWriteLength = Master_Buffer_BUFSIZE;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = MC24LC64_ADDR;
	I2CMasterBuffer[1] = address >> 8;			/* address MSB */
	I2CMasterBuffer[2] = address;				/* address LSB */	
	
	memcpy(temp_str,pdata,bytes);		
	memcpy(I2CMasterBuffer+3,temp_str,bytes);	
	uint32_t fin_state = I2CEngine();
	
	return 0;
}

int read_page_from_24LC64_EEPROM(unsigned int address, unsigned char* pdata, unsigned char bytes){
	unsigned int i;
	if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		return 1;				/* Fatal error */
	}
	for ( i = 0; i < Slave_Buffer_BUFSIZE; i++ ){
		I2CSlaveBuffer[i] = 0x00;
	}
	/* Write SLA(W), address, SLA(R), and read 4 bytes back. */

	I2CWriteLength = 3;
	I2CReadLength = Slave_Buffer_BUFSIZE;
	I2CMasterBuffer[0] = MC24LC64_ADDR;
	I2CMasterBuffer[1] = address >> 8;			/* address MSB */
	I2CMasterBuffer[2] = address;				/* address LSB */
	I2CMasterBuffer[3] = MC24LC64_ADDR | RD_BIT;

	i=0;
	while (I2CEngine() == I2CSTATE_SLA_NACK)
		i++;
	
	memcpy(pdata,I2CSlaveBuffer,bytes);
	return 0;
}

int i2ctest_wr (void)
{
	  /* Basic chip initialization is taken care of in SystemInit() called
	   * from the startup code. SystemInit() and chip settings are defined
	   * in the CMSIS system_<part family>.c file.
	   */

  uint32_t i;

  if ( I2CInit( (uint32_t)I2CMASTER ) == 0 )	/* initialize I2c */
  {
	return 1;				/* Fatal error */
  }

  /* In order to start the I2CEngine, the all the parameters 
  must be set in advance, including I2CWriteLength, I2CReadLength,
  I2CCmd, and the I2cMasterBuffer which contains the stream
  command/data to the I2c slave device. 
  (1) If it's a I2C write only, the number of bytes to be written is 
  I2CWriteLength, I2CReadLength is zero, the content will be filled 
  in the I2CMasterBuffer. 
  (2) If it's a I2C read only, the number of bytes to be read is 
  I2CReadLength, I2CWriteLength is 0, the read value will be filled 
  in the I2CMasterBuffer. 
  (3) If it's a I2C Write/Read with repeated start, specify the 
  I2CWriteLength, fill the content of bytes to be written in 
  I2CMasterBuffer, specify the I2CReadLength, after the repeated 
  start and the device address with RD bit set, the content of the 
  reading will be filled in I2CMasterBuffer index at 
  I2CMasterBuffer[I2CWriteLength+2]. 
  
  e.g. Start, DevAddr(W), WRByte1...WRByteN, Repeated-Start, DevAddr(R), 
  RDByte1...RDByteN Stop. The content of the reading will be filled 
  after (I2CWriteLength + two devaddr) bytes. */

  /* Write SLA(W), address and one data byte */

  for (i = 3; i  < Master_Buffer_BUFSIZE; i++) I2CMasterBuffer[i] = myBuffer1[i-3];

  I2CWriteLength = Master_Buffer_BUFSIZE;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = MC24LC64_ADDR;
  I2CMasterBuffer[1] = 0x00;		/* address MSB */
  I2CMasterBuffer[2] = 0x00;		/* address LSB */
  // I2CMasterBuffer[3] = 0x55;		/* Data0 */
  // I2CMasterBuffer[4] = 0xAA;		/* Data1 */
  // I2CMasterBuffer[5] = 0x12;		/* Data2 */
  // I2CMasterBuffer[6] = 0x34;		/* Data3 */
  uint32_t fin_state = I2CEngine();

  return 0;
}

int i2ctest_rd (void)
{
	  uint32_t i;

	  if ( I2CInit( (uint32_t)I2CMASTER ) == 0 )	/* initialize I2c */
	  {
		return 1;				/* Fatal error */
	  }

	  for ( i = 0; i < Slave_Buffer_BUFSIZE; i++ )
	  {
		I2CSlaveBuffer[i] = 0x00;
	  }
	  /* Write SLA(W), address, SLA(R), and read 4 bytes back. */

	  I2CWriteLength = 3;
	  I2CReadLength = Slave_Buffer_BUFSIZE;
	  I2CMasterBuffer[0] = MC24LC64_ADDR;
	  I2CMasterBuffer[1] = 0x00;		/* address MSB */
	  I2CMasterBuffer[2] = 0x00;		/* address LSB */
	  I2CMasterBuffer[3] = MC24LC64_ADDR | RD_BIT;

	  i=0;
	  while (I2CEngine() == I2CSTATE_SLA_NACK)
		  i++;

	  return 0;
}
/******************************************************************************
**                            End Of File
******************************************************************************/
