#ifndef SIMPLE_MODBUS_SLAVE_H
#define SIMPLE_MODBUS_SLAVE_H

/*
 SimpleModbusSlave allows you to communicate
 to any slave using the Modbus RTU protocol.
 
 This implementation DOES NOT fully comply with the Modbus specifications.
 
 Specifically the frame time out have not been implemented according
 to Modbus standards. The code does however combine the check for
 inter character time out and frame time out by incorporating a maximum
 time out allowable when reading from the message stream.
 
 SimpleModbusSlave implements an unsigned int return value on a call to modbus_update().
 This value is the total error count since the slave started. It's useful for fault finding.
 
 This code is for a Modbus slave implementing functions 3 and 16
 function 3: Reads the binary contents of holding registers (4X references)
 function 16: Presets values into a sequence of holding registers (4X references)
 
 All the functions share the same register array.
 
 Note:  
 The Arduino serial ring buffer is 128 bytes or 64 registers.
 Most of the time you will connect the arduino to a master via serial
 using a MAX485 or similar.
 
 In a function 3 request the master will attempt to read from your
 slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
 and two BYTES CRC the master can only request 122 bytes or 61 registers.
 
 In a function 16 request the master will attempt to write to your 
 slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
 NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
 118 bytes or 59 registers.
 
 Using a USB to Serial converter the maximum bytes you can send is 
 limited to its internal buffer which differs between manufactures. 
 
 The functions included here have been derived from the 
 Modbus Specifications and Implementation Guides
 
 http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
 http://www.modbus.org/docs/PI_MBUS_300.pdf
*/

// #include "LPC214x.h"
// #include "irq.h"
// #include "RingBuf.h"
#include <limits.h>
#include <stdio.h>
#include "global.h"
#include "config.h"
#include "delay_timer.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif
	#include "uart.h"
	unsigned int modbus_update(modbus_t*);
#ifdef __cplusplus
}
#endif

#define MODBUS_REGISTER_ADDRESS_BASE	4000

// function definitions
// void modbus_configure(	modbusCom_t*, int _slaveID);	
void modbus_configure(	modbus_t* mbus,
						long baud,						
						int _slaveID, 						
						unsigned int _holdingRegsSize,
						unsigned int* _regs);						

enum MODBUS_ERROR_TYPES {
	MB_NO_ERROR=0,
	MB_OVERFLOW,
	MB_DATA_WRITTEN,
	MB_CORRUPT_PACKET,
	MB_CHECKSUM_FAILED,
	MB_INCORRECT_ID,
	MB_DATA_READ
};
						
#endif
