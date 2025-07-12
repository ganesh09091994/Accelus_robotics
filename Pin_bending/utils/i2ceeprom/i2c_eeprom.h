/*
 * i2ctest.h
 *
 *  Created on: Feb 8, 2011
 *      Author: James
 */

#ifndef _I2C_EEPROM_H_
#define _I2C_EEPROM_H_

#include "i2c.h"
// #include "i2ctest.h"

int write_byte_to_24LC64_EEPROM(unsigned int address, unsigned char databyte, unsigned char bytes);
int read_byte_from_24LC64_EEPROM(unsigned int address, unsigned char bytes);
int write_page_to_24LC64_EEPROM(unsigned int address, unsigned char* pdata, unsigned char bytes);
int read_page_from_24LC64_EEPROM(unsigned int address, unsigned char* pdata, unsigned char bytes);

int i2ctest_wr (void);
int i2ctest_rd (void);

// string read and write..
int i2cEEPROM_write32(int16_t addr, char *s);
int i2cEEPROM_read32(unsigned int address, char* pdata, unsigned char bytes);

// int8_t read and write..
int8_t i2cEEPROM_read8(int16_t address, uint8_t *, int);
int i2cEEPROM_write8(int16_t addr, uint8_t *, int);
#endif /* I2CTEST_H_ */
