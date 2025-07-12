/*****************************************************************************
 *   uart.c:  UART API file for NXP LPC17xx Family Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *		..\Sample_ARM\from_NXP\LPC1700 CANActivity Wakup\CAN
******************************************************************************/
/*
 *  Modified for mbed NXP LPC1768 board
 *  Use UART2 & 3 modules
 *  By Kenji Arai / JH1PJL on April 24th,2010
 *  	September 27th,2010
 */

#include "lpc17xx.h"
#include "lpc_types.h"
#include "uart.h"
#include "ring.h"
#include "lpc17xx_uart.h"
#include "multi-steppers.h"

int uart_enable_accumulator[4] = {0,0,0,0}; //for rs485
int preset_enable[4] = {0,0,0,0};

#ifdef USE_GPIO_ENABLE_UART1
#define UART1_ENABLE_TIMEOUT_MULTIPLIER	MODBUS_TIMER_TIME /*Move this somewhere else*/
#endif

uint32_t UART2Status, UART1Status,UART3Status,UART0Status;
// uint8_t	 UART2TxEmpty = 1, UART3TxEmpty = 1;
// uint8_t	 uart2_errno = 0, uart3_errno = 0;
ring_t	 ring_rx0;
ring_t	 ring_rx1;
ring_t	 ring_rx2;
ring_t	 ring_rx3;
ring_t	 ring_tx0;
ring_t	 ring_tx1;
ring_t	 ring_tx2;
ring_t	 ring_tx3;
#define USE_AHB_RAM 1
#if (USE_AHB_RAM == 1)
uint8_t	 buff_rx0[UART_0_SIZE]	__attribute__((section(".Ram1")));
uint8_t	 buff_tx1[UART_1_SIZE]	__attribute__((section(".Ram1")));
uint8_t	 buff_rx1[UART_1_SIZE]	__attribute__((section(".Ram1")));
uint8_t	 buff_rx2[UART_2_SIZE]	__attribute__((section(".Ram1")));
uint8_t	 buff_tx2[UART_2_SIZE]	__attribute__((section(".Ram1")));
uint8_t	 buff_rx3[UART_3_SIZE]	__attribute__((section(".Ram1")));
uint8_t	 buff_tx3[UART_3_SIZE]	__attribute__((section(".Ram1")));
uint8_t	 buff_rxusb[USB_SIZE]	__attribute__((section(".Ram1")));
uint8_t	 buff_rxu[SIZE_COMBUF_B]	__attribute__((section(".Ram1")));
uint8_t	 buff_txu[SIZE_COMBUF_H]	__attribute__((section(".Ram1")));

#else
uint8_t	 buff_rx0[UART_0_SIZE];
uint8_t	 buff_rx1[UART_1_SIZE];
uint8_t	 buff_tx1[UART_1_SIZE];
uint8_t	 buff_rx2[UART_2_SIZE];
uint8_t	 buff_tx2[UART_2_SIZE];
uint8_t	 buff_rx3[UART_3_SIZE];
uint8_t	 buff_tx3[UART_3_SIZE];
uint8_t	 buff_rxusb[USB_SIZE];
uint8_t	 buff_rxu[SIZE_COMBUF_B];
uint8_t	 buff_txu[SIZE_COMBUF_H];

#endif

#ifdef MOD_UART0
modbus_t	modbus0;
#endif
#ifdef MOD_UART1
modbus_t	modbus1;
#endif
#ifdef MOD_UART2
modbus_t	modbus2;
#endif
#ifdef MOD_UART3
modbus_t	modbus3;
#endif

extern uint32_t SystemFrequency;	/*!< System Clock Frequency (Core Clock)  */

/*****************************************************************************
** Function name:		UART0Handler
*****************************************************************************/
void UART0_IRQHandler (void){
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;
	uint8_t c;

	IIRValue = LPC_UART0->IIR;
	IIRValue >>= 1;					/* skip pending bit in IIR */
	IIRValue &= 0x07;				/* check bit 1~3, interrupt identification */
	if ( IIRValue == IIR_RLS ){		/* Receive Line Status */
		LSRValue = LPC_UART0->LSR;
		/* Receive Line Status */
		if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) ){
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART0Status = LSRValue;
			Dummy = LPC_UART0->RBR;	/* Dummy read on RX to clear interrupt, then bail out */
			return;
		}
		if ( LSRValue & LSR_RDR ){	/* Receive Data Ready */
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			c = LPC_UART0->RBR;
			ring_putc(&ring_rx0, c);
		}
	} else if ( IIRValue == IIR_RDA ){	/* Receive Data Available */
		/* Receive Data Available */
		c = LPC_UART0->RBR;
		ring_putc(&ring_rx0, c);
	} else if ( IIRValue == IIR_CTI ){	/* Character timeout indicator */
		/* Character Time-out indicator */
		UART0Status |= 0x100;		/* Bit 9 as the CTI error */
	}
}

/*****************************************************************************
** Function name:		UART1Handler
*****************************************************************************/
void UART1_IRQHandler (void){
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;
	uint8_t c;

	IIRValue = LPC_UART1->IIR;
	IIRValue >>= 1;					/* skip pending bit in IIR */
	IIRValue &= 0x07;				/* check bit 1~3, interrupt identification */
	if ( IIRValue == IIR_RLS ){		/* Receive Line Status */
		LSRValue = LPC_UART1->LSR;
		/* Receive Line Status */
		if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) ){
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART1Status = LSRValue;
			Dummy = (uint8_t)LPC_UART1->RBR;	/* Dummy read on RX to clear interrupt, then bail out */
			return;
		}
		if ( LSRValue & LSR_RDR ){	/* Receive Data Ready */
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			c = LPC_UART1->RBR;
			#ifdef MOD_UART1
			checkModbusData(&modbus1,c);
			#else
			ring_putc(&ring_rx1, c);
			#endif
		}
	} else if ( IIRValue == IIR_RDA ){	/* Receive Data Available */
		/* Receive Data Available */
		c = LPC_UART1->RBR;
		#ifdef MOD_UART1
		checkModbusData(&modbus1,c);
		#else
		ring_putc(&ring_rx1, c);
		#endif
	} else if ( IIRValue == IIR_CTI ){	/* Character timeout indicator */
		/* Character Time-out indicator */
		UART1Status |= 0x100;		/* Bit 9 as the CTI error */
	} else if( IIRValue == IIR_THRE) {
		// if txfifo is not empty
		// copy next set of data to TX FIFO
		int count = 0;
		// if(ring_is_empty(&ring_tx1)){
		// 	// LPC_GPIO0->FIOSET |= (1<<19);

		//  	uart_enable_accumulator[1] = 0;
		// }
		while((count<16) && (!ring_is_empty(&ring_tx1))){ //TX FIFO not full yet and ring_tx2 is not empty
			// get next (1) charcter and drop it into TX FIFO
			ring_getc(&ring_tx1);
			LPC_UART1->THR = (uint16_t)ring_tx1.dt_got & 0xFF;
			count += 1;
		}
	}
}		
/*****************************************************************************
** Function name:		UART2Handler
*****************************************************************************/
void UART2_IRQHandler (void){
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;
	uint8_t c;

	IIRValue = LPC_UART2->IIR;
	IIRValue >>= 1;					/* skip pending bit in IIR */
	IIRValue &= 0x07;				/* check bit 1~3, interrupt identification */
	if ( IIRValue == IIR_RLS ){		/* Receive Line Status */
		LSRValue = LPC_UART2->LSR;
		/* Receive Line Status */
		if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) ){
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART2Status = LSRValue;
			Dummy = LPC_UART2->RBR;	/* Dummy read on RX to clear interrupt, then bail out */
			return;
		}
		if ( LSRValue & LSR_RDR ){	/* Receive Data Ready */
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			c = LPC_UART2->RBR;
			ring_putc(&ring_rx2, c);
		}
	} else if ( IIRValue == IIR_RDA ){	/* Receive Data Available */
		/* Receive Data Available */
		c = LPC_UART2->RBR;
		ring_putc(&ring_rx2, c);
	} else if ( IIRValue == IIR_CTI ){	/* Character timeout indicator */
		/* Character Time-out indicator */
		UART2Status |= 0x100;		/* Bit 9 as the CTI error */
	}
}

/*****************************************************************************
** Function name:		UART3Handler
*****************************************************************************/
void UART3_IRQHandler (void){
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;
	uint8_t c;

	IIRValue = LPC_UART3->IIR;
	IIRValue >>= 1;					/* skip pending bit in IIR */
	IIRValue &= 0x07;				/* check bit 1~3, interrupt identification */
	if ( IIRValue == IIR_RLS ){		/* Receive Line Status */
		LSRValue = LPC_UART3->LSR;
		/* Receive Line Status */
		if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) ){
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART3Status = LSRValue;
			Dummy = (uint8_t)LPC_UART3->RBR;	/* Dummy read on RX to clear interrupt, then bail out */
			return;
		}
		if ( LSRValue & LSR_RDR ){	/* Receive Data Ready */
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			c = (uint8_t)LPC_UART3->RBR;
			ring_putc(&ring_rx3, c);
			// DEBUG only
			//LPC_UART2->THR = (uint16_t)c & 0xFF;
		}
	} else if ( IIRValue == IIR_RDA ){	/* Receive Data Available */
		/* Receive Data Available */
		c = (uint8_t)LPC_UART3->RBR;
		ring_putc(&ring_rx3, c);
		// DEBUG only
		//LPC_UART2->THR = (uint16_t)c & 0xFF;
	} else if ( IIRValue == IIR_CTI ){	/* Character timeout indicator */
		/* Character Time-out indicator */
		UART3Status |= 0x100;		/* Bit 9 as the CTI error */
	}
}


/*****************************************************************************
** Function name:		UARTInit
*****************************************************************************/
uint16_t UARTInit( uint32_t portNum, uint32_t baudrate ,uint8_t uartLCR){
	
	if ( portNum == 0 ){
		// LPC_PINCON->PINSEL0 &= ~0xC0000000;
		// LPC_PINCON->PINSEL0 |= 0x0000000C;	/* Enable TxD0 P0.2 and RxD P0.3 */
		LPC_SC->PCONP |= (1UL << 3UL );//eanble power to the UART0
		LPC_PINCON->PINSEL0 |= ( 1UL << 6UL )|( 1UL << 4UL );/* Enable TxD0 P0.2 and RxD P0.3 */
		/* By default, the PCLKSELx value is zero, thus, the PCLK for
		all the peripherals is 1/4 of the SystemCoreClock. */
		/* Bit 8,9 are for UART0 *//*
		pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
		switch ( pclkdiv )
		{
		  case 0x00:
		  default:
			pclk = SystemCoreClock/4;
			break;
		  case 0x01:
			pclk = SystemCoreClock;
			break; 
		  case 0x02:
			pclk = SystemCoreClock/2;
			break; 
		  case 0x03:
			pclk = SystemCoreClock/8;
			break;
		}
	*/
		LPC_UART0->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
		
		switch (baudrate){
			case 115200 :
				LPC_UART0->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART0->DLL = 8;
				LPC_UART0->FDR = 50;	//DIVADDVAL=1, MULVAL=2
				break;
			case 19200 :
				LPC_UART0->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART0->DLL = 53;
				LPC_UART0->FDR = 248;   //DIVADDVAL=1, MULVAL=2
				break;
			case 9600 :
				LPC_UART0->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART0->DLL = 107;
				LPC_UART0->FDR = 33;	//DIVADDVAL=1, MULVAL=2
				break;
			case 4800 :
				LPC_UART0->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART0->DLL = 214;
				LPC_UART0->FDR = 33;	//DIVADDVAL=1, MULVAL=2
				break;
			default :					// 9600 bps
				LPC_UART0->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART0->DLL = 107;
				LPC_UART0->FDR = 33;	//DIVADDVAL=1, MULVAL=2
				break;
		}
		
		//Fdiv = ( pclk / 16 ) / baudrate ;	/*baud rate */
		//LPC_UART0->DLM = Fdiv / 256;							
		//LPC_UART0->DLL = Fdiv % 256;
		LPC_UART0->LCR = 0x03;		/* DLAB = 0 */
		LPC_UART0->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */
		ring_init( &ring_rx0, buff_rx0, SIZE_COMBUF_B );
		
		NVIC_EnableIRQ(UART0_IRQn);

		//LPC_UART1->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART1 interrupt */
		LPC_UART0->IER = IER_RBR | IER_RLS;	/* Enable UART1 interrupt */
		return 0;
	}else if ( portNum == 1 ){
		
		LPC_PINCON->PINSEL0 |= (1UL<<30);	/* Enable TxD1 P0.15 */
		LPC_PINCON->PINSEL1 |= (1UL<<0);	/* Enable RxD1 P0.16 */
		LPC_PINCON->PINSEL1 |= (1UL<<8);	/* Enable DTR1 P0.20 */

		/* By default, the PCLKSELx value is zero, thus, the PCLK for
		all the peripherals is 1/4 of the SystemCoreClock. */
		/* Bit 8,9 are for UART1 *//*
		pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
		switch ( pclkdiv )
		{
		  case 0x00:
		  default:
			pclk = SystemCoreClock/4;
			break;
		  case 0x01:
			pclk = SystemCoreClock;
			break; 
		  case 0x02:
			pclk = SystemCoreClock/2;
			break; 
		  case 0x03:
			pclk = SystemCoreClock/8;
			break;
		}
	*/
		LPC_UART1->LCR = uartLCR;		/* 8 bits, no Parity, 1 Stop bit */
		uart_set_divisors(LPC_UART1,baudrate);

		LPC_UART1->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

		/*RS485 initialization*/
		/*
		Auto direction control enabled.
		DTR used for Enable.
		OINV Set. Enable pin inverted.
		*/
		// LPC_UART1->RS485CTRL = 24;
		LPC_UART1->RS485CTRL = 56;
		// LPC_UART1->RS485DLY = 2; // approximately 100us per count
		LPC_UART1->RS485DLY = 0; // approximately 100us per count

		ring_init( &ring_rx1, buff_rx1, UART_1_SIZE );
		ring_init( &ring_tx1, buff_tx1, UART_1_SIZE );

		NVIC_EnableIRQ(UART1_IRQn);

		LPC_UART1->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART1 interrupt */
		// LPC_UART1->IER = IER_RBR | IER_RLS;	/* Enable UART1 interrupt */

		return 0;
		
	}else if ( portNum == 2 ){
		LPC_SC->PCONP |= (1UL << 24UL );		// Power on for UART2
		LPC_PINCON->PINSEL0 |= ( 1UL << 20UL )|( 1UL << 22UL );/* RxD2 is P0.11 and TxD2 is P0.10 */
		//LPC_PINCON->PINSEL0 |= 0x00500000;	/* RxD2 is P0.11 and TxD2 is P0.10 */

		LPC_UART2->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
		/* By default, the PCLKSELx value is zero, thus, the PCLK for all the peripherals is 1/4 of the SystemFrequency. */
		switch (baudrate){
			case 115200 :
				LPC_UART2->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART2->DLL = 8;
				LPC_UART2->FDR = 50;	//DIVADDVAL=1, MULVAL=2
				break;
			case 19200 :
				LPC_UART2->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART2->DLL = 53;
				LPC_UART2->FDR = 248;   //DIVADDVAL=1, MULVAL=2
				break;
			case 9600 :
				LPC_UART2->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART2->DLL = 107;
				LPC_UART2->FDR = 33;	//DIVADDVAL=1, MULVAL=2
				break;
			case 4800 :
				LPC_UART2->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART2->DLL = 214;
				LPC_UART2->FDR = 33;	//DIVADDVAL=1, MULVAL=2
				break;
			default :			// 9600 bps
				LPC_UART2->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART2->DLL = 107;
				LPC_UART2->FDR = 33;	//DIVADDVAL=1, MULVAL=2
				break;
		}
		LPC_UART2->LCR = 0x03;		/* DLAB = 0 */
		LPC_UART2->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */
		ring_init( &ring_rx2, buff_rx2, SIZE_COMBUF_B );

		NVIC_EnableIRQ(UART2_IRQn);

		LPC_UART2->IER = IER_RBR | IER_RLS;	/* Enable UART2 interrupt */
		return 0;
	} else if ( portNum == 3 ){
		LPC_SC->PCONP |= (1UL << 25UL );			// Power on for UART3
		LPC_PINCON->PINSEL0 |= ( 2UL << 0UL )|( 2UL << 2UL );	/* RxD3 is P0.1 and TxD3 is P0.0 */

		LPC_UART3->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
		/* By default, the PCLKSELx value is zero, thus, the PCLK for all the peripherals is 1/4 of the SystemFrequency. */
		switch (baudrate){
			case 115200 :
				LPC_UART3->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART3->DLL = 8;
				LPC_UART3->FDR = 50;	//DIVADDVAL=1, MULVAL=2
				break;
			case 19200 :
				LPC_UART3->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART3->DLL = 53;
				LPC_UART3->FDR = 248;   //DIVADDVAL=1, MULVAL=2
				break;
			case 9600 :
				LPC_UART3->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART3->DLL = 107;
				LPC_UART3->FDR = 33;	//DIVADDVAL=1, MULVAL=2
				break;
			case 4800 :
				LPC_UART3->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART3->DLL = 214;
				LPC_UART3->FDR = 33;	//DIVADDVAL=1, MULVAL=2
				break;
			default :			// 9600 bps
				LPC_UART3->DLM = 0;		//Fpclk=24.75MHz->DLest=161.13->NOT INTEGER->FRest=1.5->DLest=107->FRest=1.50
				LPC_UART3->DLL = 107;
				LPC_UART3->FDR = 33;	//DIVADDVAL=1, MULVAL=2
				break;
		}
		LPC_UART3->LCR = 0x03;		/* DLAB = 0 */
		LPC_UART3->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

		ring_init( &ring_rx3, buff_rx3, SIZE_COMBUF_B );

		NVIC_EnableIRQ(UART3_IRQn);

		LPC_UART3->IER = IER_RBR | IER_RLS;	/* Enable UART2 interrupt */
		return 0;
	}
	return 1;
}

/*****************************************************************************
 * UART2 Interface
*****************************************************************************/
// Initialize UART
uint16_t UART_Init2( void ){
	return (UARTInit( 2, 115200, 0x03));
}

// Output one character
void Uart_PutCharBuf2(uint8_t c) {
	/* THRE status, contain valid data */
	while ( !(( LPC_UART2->LSR ) & LSR_THRE ));	// Polling for write action
	LPC_UART2->THR = (uint16_t)c & 0xFF;
}

// Send strings to TX
void Uart_PutStrBuf2(uint8_t *dat){
	uint8_t c;

	while ( (c = *dat++) != 0) {
		Uart_PutCharBuf2(c);
	}
}

// Get one character
uint8_t Uart_GetC2(void){
	while(ring_getc (&ring_rx2)){
		// vTaskDelay(1/portTICK_RATE_MS );		// Wait 1mS
	}
	return ring_rx2.dt_got;
}

// Check key input
uint16_t Uart_ChkRcv2(void){
	return ring_is_empty (&ring_rx2);
}

/*****************************************************************************
 * UART3 Interface
*****************************************************************************/
// Initialize UART
uint16_t UART_Init3( void ){
	return (UARTInit( 3, 115200 ,0x03));
}

// Output one character
void Uart_PutCharBuf3(uint8_t c) {
	/* THRE status, contain valid data */
	while ( !(( LPC_UART3->LSR ) & LSR_THRE ));	// Polling for write action
	LPC_UART3->THR = (uint16_t)c & 0xFF;
}

// Send strings to TX
void Uart_PutStrBuf3(uint8_t *dat){
	uint8_t c;

	while ( (c = *dat++) != 0) {
		Uart_PutCharBuf3(c);
	}
}
// Get one character
uint8_t Uart_GetC3(){
	while(ring_getc (&ring_rx3)){
		// vTaskDelay(1/portTICK_RATE_MS );		// Wait 1mS
	}
	return ring_rx3.dt_got;
}

// Check key input
uint16_t Uart_ChkRcv3(void){
	return ring_is_empty (&ring_rx3);
}

/*****************************************************************************
 * UART0 Interface
*****************************************************************************/
// Initialize UART
uint16_t UART_Init0( void ){
	return (UARTInit( 0, 115200 ,0));
}

// Output one character
void Uart_PutCharBuf0(uint8_t c) {
	/* THRE status, contain valid data */
	while ( !(( LPC_UART0->LSR ) & LSR_THRE ));	// Polling for write action
	LPC_UART0->THR = (uint16_t)c & 0xFF;
}

// Send strings to TX
void Uart_PutStrBuf0(uint8_t *dat){
	uint8_t c;

	while ( (c = *dat++) != 0) {
		Uart_PutCharBuf0(c);
	}
}

// Check key input
uint16_t Uart_ChkRcv0(void){
	return ring_is_empty (&ring_rx0);
}

/*****************************************************************************
 * UART1 Interface
*****************************************************************************/
// Initialize UART
uint16_t UART_Init1( uint32_t baud ){
	#ifdef MOD_UART1
		modInit(&modbus1,&ring_rx1,MOD_UART1);
	#endif
	return (UARTInit( 1, baud, 0x03 ));
}

int Uart_PutCharBuf1(uint8_t c) {
	/*This function only inserts data into TX fifo. Does not actually send data.
	 For sending data one must put at least one char into the HW TX-FIFO.
	 This will genereate an interrupt after that char is transmitted.
	 Then in the IRQ we check if the tx-ring has anymore characters; 
	 if yes then put a bunch of those char into the HW TX-FIFO and the cycle continues till the ring is empty.
	*/
	if(!ring_is_full(&ring_tx1)){
		// printf("Value of c is=%d",c);
		ring_putc(&ring_tx1,c); //insert char to tx fifo
	}else{
		return 0;
	}
	return 1;
}

int Uart_StartSending1(void){
	if(!ring_is_empty(&ring_tx1)){ // ring is not empty
		ring_getc(&ring_tx1);
		LPC_UART1->THR = ring_tx1.dt_got; //write to tx fifo
		return 1;
	}
	return 0;
}

int Uart_SendCharArray1(uint8_t* c, int size){

	// copy everything to ring_tx1
	for(int i = 0;i<size;i++){
		if(!Uart_PutCharBuf1(c[i])){
			break; //ring is full
		}
	}
	
	// send first char
	Uart_StartSending1();
	return 1;
}

// Get one character
uint8_t Uart_GetC1(void){
	while(ring_getc (&ring_rx1)){
		return (uint8_t)-1;
	}
	return ring_rx1.dt_got;
}

// Check key input
uint16_t Uart_ChkRcv1(void){
	return ring_is_empty (&ring_rx1);
}

void uart1TxFlush(void)
{ //do this before starting to write new data to fifo

	// ring_clear(&ring_rx1);
	ring_clear(&ring_tx1);
	LPC_UART1->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */
	return;
}

int uart1TxRxEnable(void)
{
	LPC_UART1->FCR = 0x01;		/* Enable TX and RX FIFO. */
	return 1;
}
