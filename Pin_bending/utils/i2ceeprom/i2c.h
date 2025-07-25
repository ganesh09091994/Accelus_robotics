/*****************************************************************************
 *   i2c.h:  Header file for NXP LPC11xx Family Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   parts of this code are (C) 2010, MyVoice CAD/CAM Services
 *   All rights reserved.
 *
 *   History
 *   2006.07.19  ver 1.00    Preliminary version, first Release
 *   2010.07.19  ver 1.10    Rob Jansen - MyVoice CAD/CAM Services
 *                           Updated to reflect new code
 *   2011.03.07  ver 1.210   Larry Viesse - Corrected Buffer Sizes to accommodate writing 32 data bytes (a full page)
 *
******************************************************************************/
#ifndef __I2C_H 
#define __I2C_H

/*
 * These are states returned by the I2CEngine:
 *
 * IDLE     - is never returned but only used internally
 * PENDING  - is never returned but only used internally in the I2C functions
 * ACK      - The transaction finished and the slave returned ACK (on all bytes)
 * NACK     - The transaction is aborted since the slave returned a NACK
 * SLA_NACK - The transaction is aborted since the slave returned a NACK on the SLA
 *            this can be intentional (e.g. an 24LC08 EEPROM states it is busy)
 *            or the slave is not available/accessible at all.
 * ARB_LOSS - Arbitration loss during any part of the transaction.
 *            This could only happen in a multi master system or could also
 *            identify a hardware problem in the system.
 */
 
 /** Used I2C device as slave definition */
#define USEDI2CDEV_M		0

#if (USEDI2CDEV_M == 0)
	#define I2CDEV_M 	LPC_I2C0
	#define irq_handler	I2C0_IRQHandler	
#elif (USEDI2CDEV_M == 1)
	#define I2CDEV_M 	LPC_I2C1
	#define irq_handler	I2C1_IRQHandler	
#elif (USEDI2CDEV_M == 2)
	#define I2CDEV_M 	LPC_I2C2
	#define irq_handler	I2C2_IRQHandler	
#else
	#error "Master I2C device not defined!"
#endif
 
#define I2CSTATE_IDLE     0x000
#define I2CSTATE_PENDING  0x001
#define I2CSTATE_ACK      0x101
#define I2CSTATE_NACK     0x102
#define I2CSTATE_SLA_NACK 0x103
#define I2CSTATE_ARB_LOSS 0x104

#define FAST_MODE_PLUS	0

#define EEPROM
// #define FRAM

#ifdef EEPROM
#define PAGE_BUFFER			16
#define PAGE_BUFFER_INT		16
#define PAGE_BUFFER_FLOAT	16.0
#else

// #ifdef FRAM
#define PAGE_BUFFER			32
#define PAGE_BUFFER_INT		32
#define PAGE_BUFFER_FLOAT	32.0
#endif

#define I2C_MASTER_BUFSIZE	PAGE_BUFFER+2
#define I2C_SLAVE_BUFSIZE	PAGE_BUFFER
#define MAX_TIMEOUT			0x00FFFFFF


#define I2CMASTER		0x01
#define I2CSLAVE		0x02

#define PCF8594_ADDR	0xA0
#define READ_WRITE		0x01

#define RD_BIT			0x01

#define I2CONSET_I2EN		0x00000040  /* I2C Control Set Register */
#define I2CONSET_AA			0x00000004
#define I2CONSET_SI			0x00000008
#define I2CONSET_STO		0x00000010
#define I2CONSET_STA		0x00000020

#define I2CONCLR_AAC		0x00000004  /* I2C Control clear Register */
#define I2CONCLR_SIC		0x00000008
#define I2CONCLR_STAC		0x00000020
#define I2CONCLR_I2ENC		0x00000040

#define I2DAT_I2C			0x00000000  /* I2C Data Reg */
#define I2ADR_I2C			0x00000000  /* I2C Slave Address Reg */
#define I2SCLH_SCLH			58  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL			57  /* I2C SCL Duty Cycle Low Reg */
#define I2SCLH_HS_SCLH		0x00000020  /* Fast Plus I2C SCL Duty Cycle High Reg */
#define I2SCLL_HS_SCLL		0x00000020  /* Fast Plus I2C SCL Duty Cycle Low Reg */


extern volatile uint8_t I2CMasterBuffer[I2C_MASTER_BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[I2C_SLAVE_BUFSIZE];
extern volatile uint32_t I2CReadLength, I2CWriteLength;

extern uint32_t I2CInit( uint32_t I2cMode );
extern uint32_t I2CEngine( void );

extern int I2CDeinit( void );

#endif /* end __I2C_H */
/****************************************************************************
**                            End Of File
*****************************************************************************/
