#include "SimpleModbusSlave_1769.h"
#include "hmi.h"

extern "C"{
	#include "usbSerial.h"

}

#define BUFFER_SIZE 128
int reg_count;
/////
char temp;
// function definitions
void exceptionResponse(modbus_t* mbus,unsigned char exception);
unsigned int calculateCRC(modbus_t* mbus,unsigned char bufferSize); 
void sendPacket(modbus_t* mbus,unsigned char bufferSize);
int set_holding_registers_values(int modbus_cnt);
int apply_modbus_values(int modbus_cnt);
int setRegister(modbus_t* mbus,int i, unsigned int value);
int getRegister(modbus_t* mbus,int i);

void modbus_configure(	modbus_t* mbus,
						long baud,
						int _slaveID,
						unsigned int _holdingRegsSize,
						unsigned int* _regs)
{
	
	mbus->slaveID = _slaveID;
	mbus->holdingRegsSize = _holdingRegsSize; 
	mbus->regs = _regs;	
	mbus->errorCount = MB_NO_ERROR; // initialize errorCount
	mbus->modbusTimerCount = 0;
	/***
	int T1_5, T3_5;
	if(baud>19200){
		T1_5 = 750us;
		T3_5 = 1750us;
	}else{
		T1_5 = 15000000/baud;
		T3_5 = 35000000/baud;
	}	
	***/

	if(baud>19200){
		mbus->modbus_timeout = 1750; //keep the timeout 100us less than actual
	}else{
		mbus->modbus_timeout = ((int)(35000000/baud)); //keep the timeout 100us less than actual
	}
	init_timer0_freq(mbus->modbus_timeout); //set timer value
	// printf("\n mbus time out %d",mbus->modbus_timeout);
}


unsigned int modbus_update(modbus_t* mbus){
	mbus->errorCount=MB_NO_ERROR;
	// printf("\n calling");
	if (mbus->modbus_data_complete){ //last frame complete

		mbus->buff = getUartdata(mbus); // get data from ring buffer
		mbus->frame[0] = mbus->ring_slaveID; //was initially removed from ring
		
		mbus->modbusTimerCount = 0;
		mbus->modbus_data_complete = 0;
		
		// If an overflow occurred increment the errorCount
		// variable and return to the main sketch without 
		// responding to the request i.e. force a timeout
		if (mbus->ovrflw){
			mbus->errorCount = MB_OVERFLOW;
			return MB_OVERFLOW;
		}
		
		mbus->broadcastFlag = 0;
		// The minimum request packet is 8 bytes for function 3 & 16
		// printf("\nbuffer=%d",buffer);
		if (mbus->buff > 7){
			unsigned char id = mbus->frame[0];
			// printf("\nID %d",id);
			mbus->broadcastFlag = 0;
		
			if (id == 0){
				mbus->broadcastFlag = 1;
				// printf("\nBRDCST1");
			}
			if (id == mbus->slaveID || mbus->broadcastFlag){ // if the recieved ID matches the slaveID or broadcasting id (0), continue				
				unsigned int crc = ((mbus->frame[mbus->buff - 2] << 8) | mbus->frame[mbus->buff - 1]); // combine the crc Low & High bytes
				if (calculateCRC(mbus,(mbus->buff - 2)) == crc){ // if the calculated crc matches the recieved crc continue			
					mbus->function = mbus->frame[1];
					// printf("\nfunction %d",mbus->function);
					unsigned int startingAddress = ((mbus->frame[2] << 8) | mbus->frame[3]); // combine the starting address bytes
					// printf("\nstarting addr %d",startingAddress);
					unsigned int no_of_registers = ((mbus->frame[4] << 8) | mbus->frame[5]); // combine the number of register bytes	
					// printf("\nno_of_registers %d",no_of_registers);
					unsigned int maxData = startingAddress + no_of_registers;
					// printf("\nmaxData %d",maxData);
					
					unsigned int index;
					unsigned int address;
					unsigned int crc16;
					
					// broadcasting is not supported for function 3 
					if (!mbus->broadcastFlag && (mbus->function == 3)){
						if (startingAddress < mbus->holdingRegsSize){ // check exception 2 ILLEGAL DATA ADDRESS
							if (maxData <= mbus->holdingRegsSize){ // check exception 3 ILLEGAL DATA VALUE						  
								unsigned char noOfBytes = no_of_registers * 2; 
								// ID, function, noOfBytes, (dataLo + dataHi)*number of registers,
								//  crcLo, crcHi
								unsigned char responseFrameSize = 5 + noOfBytes; 
								// printf("\nF3");
								mbus->frame[0] = mbus->slaveID;
								mbus->frame[1] = mbus->function;
								mbus->frame[2] = noOfBytes;
								address = 3; // PDU starts at the 4th byte
								unsigned int temp;
								
								for (index = startingAddress; index < maxData; index++){
									temp = getRegister(mbus,index);
									mbus->frame[address] = temp >> 8; // split the register into 2 bytes
									address++;
									mbus->frame[address] = temp & 0xFF;
									address++;
								}	
								
								crc16 = calculateCRC(mbus,(responseFrameSize - 2));
								mbus->frame[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
								mbus->frame[responseFrameSize - 1] = crc16 & 0xFF;								
								sendPacket(mbus,responseFrameSize);
								mbus->errorCount = MB_DATA_READ;
							}else	
								exceptionResponse(mbus,3); // exception 3 ILLEGAL DATA VALUE
						}else
							exceptionResponse(mbus,2); // exception 2 ILLEGAL DATA ADDRESS
					}
					else if (mbus->function == 16){
							// Check if the recieved number of bytes matches the calculated bytes 
							// minus the request bytes.
							// id + function + (2 * address bytes) + (2 * no of register bytes) + 
							// byte count + (2 * CRC bytes) = 9 bytes
							if(mbus->frame[6] == (mbus->buff - 9)){						
								if (startingAddress < mbus->holdingRegsSize){ // check exception 2 ILLEGAL DATA ADDRESS						  
									if (maxData <= mbus->holdingRegsSize){ // check exception 3 ILLEGAL DATA VALUE							  
										address = 7; // start at the 8th byte in the frame
										for (index = startingAddress; index < maxData; index++){
											unsigned int temp = ((mbus->frame[address] << 8) | mbus->frame[address + 1]);
											setRegister(mbus,index,temp);
											address += 2;
										}	
										// printf("\nF16");
										// only the first 6 bytes are used for CRC calculation
										crc16 = calculateCRC(mbus,6); 
										mbus->frame[6] = crc16 >> 8; // split crc into 2 bytes
										mbus->frame[7] = crc16 & 0xFF;
									
										// a function 16 response is an echo of the first 6 bytes from 
										// the request + 2 crc bytes
										if (!mbus->broadcastFlag){ // don't respond if it's a broadcast message
											sendPacket(mbus,8);
											mbus->errorCount = MB_DATA_WRITTEN;
										}	
									}else	
										exceptionResponse(mbus,3); // exception 3 ILLEGAL DATA VALUE
								}else
									exceptionResponse(mbus,2); // exception 2 ILLEGAL DATA ADDRESS
							}else 
								mbus->errorCount=MB_CORRUPT_PACKET; // corrupted packet
								// errorCount++; // corrupted packet
						}
						else if(mbus->function == 6){
								// printf("\n%d",mbus->function);
								// Check if the recieved number of bytes matches the calculated bytes 
								// minus the request bytes.
								// id + function + (2 * address bytes) + (2 * no of register bytes) + 
								// byte count + (2 * CRC bytes) = 9 bytes
								// if(frame[6] == (buffer - 9)){
									// printf("\n%d",frame[6]);
									if(startingAddress < mbus->holdingRegsSize){ // check exception 2 ILLEGAL DATA ADDRESS
										maxData = startingAddress+1; //1 is added bcoz function6 dosent provide how many register needs to read
										// printf("\n%d",maxData);
										if (maxData <= mbus->holdingRegsSize){ // check exception 3 ILLEGAL DATA VALUE
											address = 4; // start at the 6th byte in the frame
											// printf("\n%d",address);
											for (index = startingAddress; index < maxData; index++){
												unsigned int temp = ((mbus->frame[address] << 8) | mbus->frame[address + 1]);
												setRegister(mbus,index,temp);
												// mbus->regs[index] = ((mbus->frame[address] << 8) | mbus->frame[address + 1]);
												address += 2;
											}
										
											// only the first 6 bytes are used for CRC calculation
											crc16 = calculateCRC(mbus,6);
											mbus->frame[6] = crc16 >> 8; // split crc into 2 bytes
											mbus->frame[7] = crc16 & 0xFF;
										
											// a function 16 response is an echo of the first 6 bytes from 
											// the request + 2 crc bytes
											if (!mbus->broadcastFlag){ // don't respond if it's a broadcast message
												sendPacket(mbus,8);
												mbus->errorCount = MB_DATA_WRITTEN;
											}
										}else
											exceptionResponse(mbus,3); // exception 3 ILLEGAL DATA VALUE
									}else
										exceptionResponse(mbus,2); // exception 2 ILLEGAL DATA ADDRESS
								// }else 
									// errorCount=MB_CORRUPT_PACKET; // corrupted packet
								// errorCount++; // corrupted packet
							}else
								exceptionResponse(mbus,1); // exception 1 ILLEGAL FUNCTION
						}else // checksum failed
							mbus->errorCount=MB_CHECKSUM_FAILED;
							// errorCount++;
					}else // incorrect id
						mbus->errorCount=MB_INCORRECT_ID;
				}else if (mbus->buff > 0 && mbus->buff < 8)
					mbus->errorCount=MB_CORRUPT_PACKET; // corrupted packet
				// errorCount++; // corrupted packet
	}	
	return mbus->errorCount;
}

void exceptionResponse(modbus_t* mbus,unsigned char exception){
  // each call to exceptionResponse() will increment the mbus->errorCount
	// mbus->errorCount++;
	if (!mbus->broadcastFlag){ // don't respond if its a broadcast message
		mbus->frame[0] = mbus->slaveID;
		mbus->frame[1] = (mbus->function | 0x80); // set MSB bit high, informs the master of an exception
		mbus->frame[2] = exception;
		unsigned int crc16 = calculateCRC(mbus,3); // ID, mbus->function|0x80, exception code
		mbus->frame[3] = crc16 >> 8;
		mbus->frame[4] = crc16 & 0xFF;
		// exception response is always 5 bytes 
		// ID, mbus->function + 0x80, exception code, 2 bytes crc
		sendPacket(mbus,5); 
	}
	printf("\nException%d %d",exception,mbus->function);
}

unsigned int calculateCRC(modbus_t* mbus,unsigned char bufferSize) {
	unsigned int temp, temp2, flag;
	temp = 0xFFFF;
	for (unsigned char i = 0; i < bufferSize; i++){
		temp = temp ^ mbus->frame[i];
		for (unsigned char j = 1; j <= 8; j++){
			flag = temp & 0x0001;
			temp >>= 1;
			if (flag)
				temp ^= 0xA001;
		}
	}
	// Reverse byte order. 
	temp2 = temp >> 8;
	temp = (temp << 8) | temp2;
	temp &= 0xFFFF; 
	// the returned value is already swapped
	// crcLo byte is first & crcHi byte is last
	return temp; 
}


void sendPacket(modbus_t* mbus,unsigned char bufferSize){
  // digitalWrite(TxEnablePin, HIGH);
	switch(mbus->uart_port){
		case MOD_UART1: //uart 2.. actual (buffersize+1).. uartputchar is not sending the last frame. so send one additional frame to the function.
			Uart_SendCharArray1(mbus->frame,bufferSize);

			uart1TxRxEnable();
			break;
		case 4: //usb... (buffersize - 1) if bufferSize passed is (actual+1)
			for (unsigned char i = 0; i < (bufferSize); i++){
				VCOM_putchar(mbus->frame[i]); //
			}
			break;
	}
	// allow a mbus->frame delay to indicate end of transmission
	// my_delay(T3_5); 
	
  // digitalWrite(TxEnablePin, LOW);
}


int getRegister(modbus_t* mbus,int i){
	
	set_holding_registers_values(i);
	
	return (mbus->regs[i]);
}

int setRegister(modbus_t* mbus,int i, unsigned int value){
	
	mbus->regs[i] = value;//write new value to holding register	
	if(mbus->function == 16){
		if(reg_count){
			apply_modbus_values(i-1);
			reg_count = -1;
		}
		reg_count++;
	}
	else{
		// printf("\nhi how_are_you?");
		apply_modbus_values(i);
	}

	holdingRegs_prev[i] = value;
	return 1;
}
