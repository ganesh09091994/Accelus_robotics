/*
 * i2ctest.h
 *
 *  Created on: Feb 8, 2011
 *      Author: James
 */

#ifndef _I2C_EEPROM_H_
#define _I2C_EEPROM_H_

int write_byte_to_24LC64_EEPROM(unsigned int address, unsigned char databyte, unsigned char bytes);
int read_byte_from_24LC64_EEPROM(unsigned int address, unsigned char bytes);
int write_page_to_24LC64_EEPROM(unsigned int address, unsigned char* pdata, unsigned char bytes);
int read_page_from_24LC64_EEPROM(unsigned int address, unsigned char* pdata, unsigned char bytes);

int i2ctest_wr (void);
int i2ctest_rd (void);

#endif /* I2CTEST_H_ */
