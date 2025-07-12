/*****************************************************************************
 *   uart.h:  Header file for NXP LPC17xx Family Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.05.27  ver 1.00    Prelimnary version, first Release
 *
 *		..\Sample_ARM\from_NXP\LPC1700 CANActivity Wakup\CAN
******************************************************************************/
#ifndef __UART_H 
#define __UART_H

#include "lpc17xx.h"
#include "modbus.h"

#define IER_RBR		0x01
#define IER_THRE	0x02
#define IER_RLS		0x04

#define IIR_PEND	0x01
#define IIR_RLS		0x03
#define IIR_RDA		0x02
#define IIR_CTI		0x06
#define IIR_THRE	0x01

#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80

#define BUFSIZE		0x40

/**
***************Defined in ring.h********************
#define SIZE_COMBUF_S	64
#define SIZE_COMBUF_B	128
#define SIZE_COMBUF_M	256
#define SIZE_COMBUF_H	512
****************************************************
**/

#define UART_0_SIZE 	SIZE_COMBUF_B
#define UART_1_SIZE 	SIZE_COMBUF_M
#define UART_2_SIZE 	SIZE_COMBUF_M
#define UART_3_SIZE 	SIZE_COMBUF_M
#define USB_SIZE 		SIZE_COMBUF_M

/* 	for Monitor		*/
uint16_t UART_Init2( void );			// Initialize UART
void Uart_PutCharBuf2(uint8_t);			// Output one character
void Uart_PutStrBuf2(uint8_t *);		// Send strings to TX
// uint8_t Uart_GetC2( void );				// Get one character
uint16_t Uart_ChkRcv2(void);			// Check receiving buffer

/* for printf...*/
uint16_t UART_Init3( void );			// Initialize UART
void Uart_PutCharBuf3(uint8_t);			// Output one character
void Uart_PutStrBuf3(uint8_t *);		// Send strings to TX
// uint8_t Uart_GetC3( void );				// Get one character
uint16_t Uart_ChkRcv3(void);			// Check receiving buffer

/* for printf...*/
uint16_t UART_Init0( void );			// Initialize UART
void Uart_PutCharBuf0(uint8_t);			// Output one character
void Uart_PutStrBuf0(uint8_t *);		// Send strings to TX
// uint8_t Uart_GetC3( void );				// Get one character
uint16_t Uart_ChkRcv0(void);			// Check receiving buffer

uint16_t UART_Init1(uint32_t);			// Initialize UART at given baudrate
int Uart_PutCharBuf1(uint8_t);			// Output one character
uint8_t Uart_GetC1( void );				// Get one character
uint16_t Uart_ChkRcv1(void);			// Check receiving buffer
int Uart_StartSending1(void);
int Uart_SendCharArray1(uint8_t*,int);
void uart1TxFlush(void);
int uart1TxRxEnable(void);

/* for printf*/
uint16_t UARTInit( uint32_t portNum, uint32_t baudrate ,uint8_t uartLCR);

#ifdef MOD_UART0
extern modbus_t	modbus0;
#endif

#ifdef MOD_UART1
extern modbus_t	modbus1;
#endif

#ifdef MOD_UART2
extern modbus_t	modbus2; //used to update modbus data in multisteppers
#endif

#ifdef MOD_UART3
extern modbus_t	modbus3;
#endif
extern modbus_t	modbusUSB;

#endif /* end __UART_H */

