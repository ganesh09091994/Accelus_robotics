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
#include <string.h>
#include "i2c_eeprom.h"

#define MC24LC64_ADDR_BLK0 0xA0		/* I2C address of the 24LC64 EEPROM */
#define MC24LC64_ADDR_BLK1 0xA2	//block 1 address
#define MC24LC64_ADDR_BLK2 0xA4	//block 2 address
#define MC24LC64_ADDR_BLK3 0xA6	//block 3 address
#define MC24LC64_ADDR_BLK4 0xA8	//block 4 address
#define MC24LC64_ADDR_BLK5 0xAA	//block 5 address
#define MC24LC64_ADDR_BLK6 0xAC	//block 6 address
#define MC24LC64_ADDR_BLK7 0xAE	//block 7 address

/*******************************************************************************
**   Main Function  main()
*******************************************************************************/
// int write_byte_to_24LC64_EEPROM(unsigned int address, unsigned char databyte, unsigned char bytes){
	
	// if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		// return 1;				/* Fatal error */
	// }
	
	// I2CWriteLength = bytes;
	// I2CReadLength = 0;
	// I2CMasterBuffer[0] = MC24LC64_ADDR;
	// I2CMasterBuffer[1] = address >> 8;			/* address MSB */
	// I2CMasterBuffer[2] = address;				/* address LSB */
	// I2CMasterBuffer[3] = databyte;				/* address LSB */	
	// uint32_t fin_state = I2CEngine();	
	// return fin_state;
// }


// int read_byte_from_24LC64_EEPROM(unsigned int address, unsigned char bytes){
	// uint32_t i;
	// if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		// return 1;				/* Fatal error */
	// }
	// for ( i = 0; i < I2C_SLAVE_BUFSIZE; i++ ){
		// I2CSlaveBuffer[i] = 0x00;
	// }
	// /* Write SLA(W), address, SLA(R), and read 4 bytes back. */

	// I2CWriteLength = 3;
	// I2CReadLength = bytes;
	// I2CMasterBuffer[0] = MC24LC64_ADDR;
	// I2CMasterBuffer[1] = address >> 8;			/* address MSB */
	// I2CMasterBuffer[2] = address;				/* address LSB */
	// I2CMasterBuffer[3] = MC24LC64_ADDR | RD_BIT;

	// i=0;
	// while (I2CEngine() == I2CSTATE_SLA_NACK)
		// i++;	
	// return 0;
// }

// int write_page_to_24LC64_EEPROM(unsigned int address, unsigned char* pdata, unsigned char bytes){
	// unsigned int i;
	// unsigned char temp_str[bytes];
	// if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		// return 1;				/* Fatal error */
	// }	
	
	// I2CWriteLength = I2C_MASTER_BUFSIZE;
	// I2CReadLength = 0;
	// I2CMasterBuffer[0] = MC24LC64_ADDR;
	// I2CMasterBuffer[1] = address >> 8;			/* address MSB */
	// I2CMasterBuffer[2] = address;				/* address LSB */	
	
	// memcpy(temp_str,pdata,bytes);		
	// memcpy((I2CMasterBuffer+3),temp_str,bytes);	
	// // uint32_t fin_state = I2CEngine();	
	// I2CEngine();	
	// return 0;
// }

// int read_page_from_24LC64_EEPROM(unsigned int address, unsigned char* pdata, unsigned char bytes){
	// unsigned int i;
	// if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		// return 1;				/* Fatal error */
	// }
	// for ( i = 0; i < I2C_SLAVE_BUFSIZE; i++ ){
		// I2CSlaveBuffer[i] = 0x00;
	// }
	// /* Write SLA(W), address, SLA(R), and read 4 bytes back. */

	// I2CWriteLength = 3;
	// I2CReadLength = I2C_SLAVE_BUFSIZE;
	// I2CMasterBuffer[0] = MC24LC64_ADDR;
	// I2CMasterBuffer[1] = address >> 8;			/* address MSB */
	// I2CMasterBuffer[2] = address;				/* address LSB */
	// I2CMasterBuffer[3] = MC24LC64_ADDR | RD_BIT;

	// i=0;
	// while (I2CEngine() == I2CSTATE_SLA_NACK)
		// i++;
		
	// // for ( i = 0; i < I2C_SLAVE_BUFSIZE; i++ ){
		// // printf("\nRCVD: %d",(I2CSlaveBuffer[i]));
	// // }
	// memcpy(pdata,I2CSlaveBuffer,bytes);	
	// return 0;
// }


int i2cEEPROM_read32(unsigned int address, char* pdata, unsigned char bytes){
	unsigned int i;
	if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		return 1;				/* Fatal error */
	}
	for ( i = 0; i < I2C_SLAVE_BUFSIZE; i++ ){
		I2CSlaveBuffer[i] = 0x00;
	}
	/* Write SLA(W), address, SLA(R), and read 4 bytes back. */

	I2CWriteLength = 3;
	I2CReadLength = I2C_SLAVE_BUFSIZE;
	I2CMasterBuffer[0] = MC24LC64_ADDR_BLK0;
	I2CMasterBuffer[1] = address>>8;			/* address MSB */
	I2CMasterBuffer[2] = address&0xff;				/* address LSB */
	I2CMasterBuffer[3] = MC24LC64_ADDR_BLK0 | RD_BIT;

	i=0;
	while (I2CEngine() == I2CSTATE_SLA_NACK)
		i++;
		
	// for ( i = 0; i < I2C_SLAVE_BUFSIZE; i++ ){
		// printf("%d, ",(I2CSlaveBuffer[i]));
	// }
	// printf("\n");
	memcpy(pdata,I2CSlaveBuffer,bytes);	
	return 0;
}

int i2cEEPROM_write32(int16_t addr, char *s){
	uint32_t i;
	uint32_t len;

	if ( I2CInit( (uint32_t)I2CMASTER ) == 0 )	/* initialize I2c */
	{
		return 1;				/* Fatal error */
	}
	
	I2CWriteLength = I2C_MASTER_BUFSIZE; // set the write len
	len = strlen(s)+3;
	if (I2CWriteLength > len)
		I2CWriteLength = len;
		
	I2CReadLength = 0;
	I2CMasterBuffer[0] = MC24LC64_ADDR_BLK0;
	I2CMasterBuffer[1] = addr>>8;		/* address MSB */
	I2CMasterBuffer[2] = addr&0xff;		/* address LSB */
	
	for (i = 3; i  < I2C_MASTER_BUFSIZE; i++) {
		if (s[i-3] == '\0')	
			break;
		I2CMasterBuffer[i] = s[i-3];
	}
	
	// uint32_t fin_state = I2CEngine();
	I2CEngine();
	return 0;
}

int i2cEEPROM_write8(int16_t addr, uint8_t *pdata, int size){
	// printf("\nWRblk=%d,%d,%d\t",block_num,addr,size);
	uint32_t id;
	uint32_t len;
	
	if ( I2CInit( (uint32_t)I2CMASTER ) == 0 )	/* initialize I2c */
	{
		return 1;				/* Fatal error */
	}
	
	I2CWriteLength=I2C_MASTER_BUFSIZE; // set the write len
	len=size+2;
	if (I2CWriteLength > len)
		I2CWriteLength = len;
		
	I2CReadLength = 0;
#if 0
	if(block_num < 16){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK0;
	}else if(block_num < 32){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK1;
		addr = addr - 256;
	}else if(block_num < 48){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK2;
		addr = addr - 512;
	}else{
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK3;
		addr = addr - 768;
	}
#else
	if(addr < 256){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK0;
	}else if(addr < 512){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK1;
		addr = addr - 256;
	}else if(addr < 768){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK2;
		addr = addr - 512;
	}else if(addr < 1024){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK3;
		addr = addr - 768;
	}else if(addr < 1280){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK4;
		addr = addr - 1024;
	}else if(addr < 1536){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK5;
		addr = addr - 1280;
	}else if(addr < 1792){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK6;
		addr = addr - 1536;
	}else{
		// printf("\n write");
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK7;
		addr = addr - 1792;
	}
#endif
	
	I2CMasterBuffer[1] = addr&0xff;		/* address LSB */
	// I2CMasterBuffer[1] = addr>>8;		/* address MSB */
	
	for (id = 2; id < I2CWriteLength; id++) 
		I2CMasterBuffer[id]=*pdata++;
	
	uint32_t fin_state = I2CEngine();

	return 0;
}

int8_t i2cEEPROM_read8(int16_t address, uint8_t *pdata, int size){
	// printf("\nRDblk=%d,%d\t",block_num, address);
	int id;	
	if ( I2CInit( (uint32_t)I2CMASTER ) == 0 ){		/* initialize I2c */
		return 1;				/* Fatal error */
	}
	for ( id = 0; id < I2C_SLAVE_BUFSIZE; id++ ){
		I2CSlaveBuffer[id] = 0x00;
		I2CMasterBuffer[0] = 0x00;
	}
	/* Write SLA(W), address, SLA(R), and read 4 bytes back. */

	I2CWriteLength = 2;	
	I2CReadLength = 0;
	
#if 0
	if(block_num < 16){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK0;
	}else if(block_num < 32){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK1;
		address = address-256;
	}else if(block_num < 48){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK2;
		address = address - 512;
	}else{
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK3;
		address = address - 768;
	}
#else
	if(address < 256){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK0;
	}else if(address < 512){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK1;
		address = address-256;
	}else if(address < 768){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK2;
		address = address - 512;
	}else if(address < 1024){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK3;
		address = address - 768;
	}else if(address < 1280){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK4;
		address = address - 1024;
	}else if(address < 1536){

		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK5;
		address = address - 1280;
	}else if(address < 1792){
		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK6;
		address = address - 1536;
	}else{

		I2CMasterBuffer[0] = MC24LC64_ADDR_BLK7;
		address = address - 1792;
	}
#endif
	
	// I2CMasterBuffer[0] = MC24LC64_ADDR_BLK0;
	I2CMasterBuffer[1] = address&0xff;			/* address MSB */
	// I2CMasterBuffer[2] = address&0xff;			/* address LSB */

	id=0;
	while (I2CEngine() == I2CSTATE_SLA_NACK)
		id++;
		
	I2CWriteLength = 1;
	I2CReadLength = I2C_SLAVE_BUFSIZE;
	I2CMasterBuffer[0] = I2CMasterBuffer[0] | RD_BIT;
	
	id=0;
	while (I2CEngine() == I2CSTATE_SLA_NACK)
		id++;	
		
	for ( id = 0; id < size; id++ )
		*pdata++=I2CSlaveBuffer[id];
	
	for ( id = 0; id < size; id++ ){
		// printf("%x, ",(I2CSlaveBuffer[id]));
	}
	// printf("\n");
	
	
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

  for (i = 3; i  < I2C_MASTER_BUFSIZE; i++) I2CMasterBuffer[i] = 0xAA;

  I2CWriteLength = I2C_MASTER_BUFSIZE;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = MC24LC64_ADDR_BLK0;
  I2CMasterBuffer[1] = 0x00;		/* address MSB */
  I2CMasterBuffer[2] = 0x00;		/* address LSB */
  I2CMasterBuffer[3] = 0x55;		/* Data0 */
  I2CMasterBuffer[4] = 0xAA;		/* Data1 */
  I2CMasterBuffer[5] = 0x12;		/* Data2 */
  I2CMasterBuffer[6] = 0x34;		/* Data3 */
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

	  for ( i = 0; i < I2C_SLAVE_BUFSIZE; i++ )
	  {
		I2CSlaveBuffer[i] = 0xbb;
	  }
	  /* Write SLA(W), address, SLA(R), and read 4 bytes back. */

	  I2CWriteLength = 3;
	  I2CReadLength = I2C_SLAVE_BUFSIZE;
	  I2CMasterBuffer[0] = MC24LC64_ADDR_BLK0;
	  I2CMasterBuffer[1] = 0x00;		/* address MSB */
	  I2CMasterBuffer[2] = 0x00;		/* address LSB */
	  I2CMasterBuffer[3] = MC24LC64_ADDR_BLK0 | RD_BIT;

	  i=0;
	  while (I2CEngine() == I2CSTATE_SLA_NACK)
		  i++;

	  return 0;
}
/******************************************************************************
**                            End Of File
******************************************************************************/
