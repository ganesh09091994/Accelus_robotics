#ifndef _MODBUS_H_
#define _MODBUS_H_

#include "inttypes.h"
#include "ring.h"

#define BUFFER_SIZE 128
// #define BUFFER_SIZE 512
// #define BUFFER_SIZE 256

// #define MOD_UART0		0
#define MOD_UART1		1
// #define MOD_UART2		2
// #define MOD_UART3		3
#define MOD_USB		4

typedef struct{

	int modbus_data_complete;
	unsigned int modbusTimerCount;
	ring_t* ring_address; //stores pointer to current modbus ring.
	uint8_t uart_port; //uart 0,2,3
	unsigned int modbus_timeout;
	uint8_t processed;
	char ring_slaveID; // slaveID of received packet
	
	//data originally from simplemodbusslave
	unsigned char frame[BUFFER_SIZE];
	unsigned int holdingRegsSize; // size of the register array
	unsigned char broadcastFlag;
	unsigned int * regs;
	unsigned int slaveID;
	unsigned char function;
	unsigned int errorCount;
	unsigned char ovrflw;
	unsigned char buff;
	
}modbus_t;

unsigned char getUartdata(modbus_t*);
int checkModbusData(modbus_t*, uint8_t);
int modInit(modbus_t*,ring_t*,uint8_t);

#endif
