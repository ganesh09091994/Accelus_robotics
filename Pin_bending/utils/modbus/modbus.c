#include "modbus.h"
#include "multi-steppers.h"
#include "usbSerial.h"

uint8_t fixedData = 9; // slave address(1), function code(1), start address(2), Qty(2),Byte Count(1), CRC(2)

unsigned char getUartdata(modbus_t* mbus){ //read only modbusPacketSize number of bytes
	unsigned char buffer = 1; //first char (slaveID) already removed from ring during interrupt
	// mbus->frame[0] = ring_slaveID;
	switch(mbus->uart_port){
		case MOD_UART1: //uart 1
			while (!Uart_ChkRcv1()){	//
				// The maximum number of bytes is limited to the serial buffer size of 128 bytes
				// If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the 
				// serial buffer will be red untill all the data is cleared from the receive buffer.
				if (mbus->ovrflw) 
					Uart_GetC1();
				else{
					if (buffer == BUFFER_SIZE)
						mbus->ovrflw = 1;
					mbus->frame[buffer] = Uart_GetC1();
					buffer++;
				}
			}
			break;

		case MOD_USB:
			buffer = 1;
			while(!VCOM_ChkRcv()){
				if (mbus->ovrflw){
					VCOM_GetCh();
				}
				else{
					if (buffer == BUFFER_SIZE){
						mbus->ovrflw = 1;
						// printf("\nOverflow1");
					}
					mbus->frame[buffer] = VCOM_GetCh();
					// printf("\n%d  %d",mbus->frame[buf],buf);
					buffer++;
				}
			}
		break;
			
	}
	return buffer;
}

int checkModbusData(modbus_t* modbus, uint8_t c){
	// MODBUS_PACKET_TIMER_RESET;
	// MODBUS_PACKET_TIMER; //start timer
	modbus->modbusTimerCount = 0;
	ring_putc(modbus->ring_address,c);
	return 1;
}

int modInit(modbus_t* modbus, ring_t* ring, uint8_t num){
	modbus->ring_address = ring; //ring_address contains the address of the pointer.
	modbus->uart_port = num;
	// modbus->modbus_timeout = 3200; //default for 9600 baud
	modbus->modbusTimerCount = 0;
	modbus->modbus_data_complete = 0;
	ring_clear (modbus->ring_address);
	return 1;
}
