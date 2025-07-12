//#define POLLED_USBSERIAL True


/*
	LPCUSB, an USB device driver for LPC microcontrollers	
	Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright
	   notice, this list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright
	   notice, this list of conditions and the following disclaimer in the
	   documentation and/or other materials provided with the distribution.
	3. The name of the author may not be used to endorse or promote products
	   derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
	OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
	IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
	THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
	Minimal implementation of a USB serial port, using the CDC class.
	This example application simply echoes everything it receives right back
	to the host.

	Windows:
	Extract the usbser.sys file from .cab file in C:\WINDOWS\Driver Cache\i386
	and store it somewhere (C:\temp is a good place) along with the usbser.inf
	file. Then plug in the LPC176x and direct windows to the usbser driver.
	Windows then creates an extra COMx port that you can open in a terminal
	program, like hyperterminal.

	Linux:
	The device should be recognised automatically by the cdc_acm driver,
	which creates a /dev/ttyACMx device file that acts just like a regular
	serial port.

*/

/* Modified by Sagar G V, Feb 2011
Used the USB CDC example to create a library. Added the following functions

void VCOM_puts(const char* str); //writes a null terminated string. 
void VCOM_putc(char c); // writes a character.
void VCOM_putHex(uint8_t hex); // writes 0x.. hex value on the terminal.
char VCOM_getc(); // returns character entered in the terminal. blocking function
void VCOM_gets(char* str); // returns a string. '\r' or '\n' will terminate character collection.
char VCOM_getc_echo(); // returns character entered and echoes the same back.
void VCOM_gets_echo(char *str); // gets string terminated in '\r' or '\n' and echoes back the same.

*/


#include "usbSerial.h"
#include "modbus.h"
#include "uart.h"

uint8_t	 buff_rxUSB[USB_SIZE];
ring_t	 ring_rxUSB;
#ifdef MOD_USB
modbus_t modbusUSB;
#endif

// data structure for GET_LINE_CODING / SET_LINE_CODING class requests
typedef struct {
	U32		dwDTERate;
	uint8_t		bCharFormat;
	uint8_t		bParityType;
	uint8_t		bDataBits;
} TLineCoding;

static TLineCoding LineCoding = {115200, 0, 0, 8};
static uint8_t abBulkBuf[64];
static uint8_t abClassReqData[8];

static uint8_t txdata[VCOM_FIFO_SIZE];
static uint8_t rxdata[VCOM_FIFO_SIZE];

static fifo_t txfifo;
static fifo_t rxfifo;

// forward declaration of interrupt handler
void USBIntHandler(void);

static const uint8_t abDescriptors[] = {

// device descriptor
	0x12,
	DESC_DEVICE,
	LE_WORD(0x0101),			// bcdUSB
	0x02,						// bDeviceClass
	0x00,						// bDeviceSubClass
	0x00,						// bDeviceProtocol
	MAX_PACKET_SIZE0,			// bMaxPacketSize
	LE_WORD(0xFFFF),			// idVendor
	LE_WORD(0x0006),			// idProduct
	LE_WORD(0x0100),			// bcdDevice
	0x01,						// iManufacturer
	0x02,						// iProduct
	0x03,						// iSerialNumber
	0x01,						// bNumConfigurations

// configuration descriptor
	0x09,
	DESC_CONFIGURATION,
	LE_WORD(67),				// wTotalLength
	0x02,						// bNumInterfaces
	0x01,						// bConfigurationValue
	0x00,						// iConfiguration
	0xC0,						// bmAttributes
	0x32,						// bMaxPower
// control class interface
	0x09,
	DESC_INTERFACE,
	0x00,						// bInterfaceNumber
	0x00,						// bAlternateSetting
	0x01,						// bNumEndPoints
	0x02,						// bInterfaceClass
	0x02,						// bInterfaceSubClass
	0x01,						// bInterfaceProtocol, linux requires value of 1 for the cdc_acm module
	0x00,						// iInterface
// header functional descriptor
	0x05,
	CS_INTERFACE,
	0x00,
	LE_WORD(0x0110),
// call management functional descriptor
	0x05,
	CS_INTERFACE,
	0x01,
	0x01,						// bmCapabilities = device handles call management
	0x01,						// bDataInterface
// ACM functional descriptor
	0x04,
	CS_INTERFACE,
	0x02,
	0x02,						// bmCapabilities
// union functional descriptor
	0x05,
	CS_INTERFACE,
	0x06,
	0x00,						// bMasterInterface
	0x01,						// bSlaveInterface0
// notification EP
	0x07,
	DESC_ENDPOINT,
	INT_IN_EP,					// bEndpointAddress
	0x03,						// bmAttributes = intr
	LE_WORD(8),					// wMaxPacketSize
	0x0A,						// bInterval
// data class interface descriptor
	0x09,
	DESC_INTERFACE,
	0x01,						// bInterfaceNumber
	0x00,						// bAlternateSetting
	0x02,						// bNumEndPoints
	0x0A,						// bInterfaceClass = data
	0x00,						// bInterfaceSubClass
	0x00,						// bInterfaceProtocol
	0x00,						// iInterface
// data EP OUT
	0x07,
	DESC_ENDPOINT,
	BULK_OUT_EP,				// bEndpointAddress
	0x02,						// bmAttributes = bulk
	LE_WORD(MAX_PACKET_SIZE),	// wMaxPacketSize
	0x00,						// bInterval
// data EP in
	0x07,
	DESC_ENDPOINT,
	BULK_IN_EP,					// bEndpointAddress
	0x02,						// bmAttributes = bulk
	LE_WORD(MAX_PACKET_SIZE),	// wMaxPacketSize
	0x00,						// bInterval
	
	// string descriptors
	0x04,
	DESC_STRING,
	LE_WORD(0x0409),

	0x0E,
	DESC_STRING,
	'C', 0, 'N', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

	0x14,
	DESC_STRING,
	'C', 0, 'N', 0, 'C', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0,

	0x12,
	DESC_STRING,
	'C', 0, 'N', 0, 'C', 0, '0', 0, 'C', 0, '0', 0, 'D', 0, 'E', 0,

// terminating zero
	0
};


/**
	Local function to handle incoming bulk data
		
	@param [in] bEP
	@param [in] bEPStatus
 */
static void BulkOut(uint8_t bEP, uint8_t bEPStatus)
{
	int i, iLen;
	bEPStatus = bEPStatus;
	if (fifo_free(&rxfifo) < MAX_PACKET_SIZE) {
		// may not fit into fifo
		return;
	}
	// get data from USB into intermediate buffer
	iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));
	for (i = 0; i < iLen; i++) {
		// put into FIFO
		// if (!fifo_put(&rxfifo, abBulkBuf[i])) {
		// 	// overflow... :(
		// 	ASSERT(False);
		// 	break;
		// }
		
		#ifdef MOD_USB
		checkModbusData(&modbusUSB, abBulkBuf[i]);
		#else
		ring_putc(&ring_rxUSB, abBulkBuf[i]);
		#endif
	}
}


/**
	Local function to handle outgoing bulk data
		
	@param [in] bEP
	@param [in] bEPStatus
 */
static void BulkIn(uint8_t bEP, uint8_t bEPStatus)
{
	int i, iLen;
	bEPStatus = bEPStatus;
	if (fifo_avail(&txfifo) == 0) {
		// no more data, disable further NAK interrupts until next USB frame
		USBHwNakIntEnable(0);
		return;
	}

	// get bytes from transmit FIFO into intermediate buffer
	for (i = 0; i < MAX_PACKET_SIZE; i++) {
		if (!fifo_get(&txfifo, &abBulkBuf[i])) {
			break;
		}
	}
	iLen = i;
	
	// send over USB
	if (iLen > 0) {
		USBHwEPWrite(bEP, abBulkBuf, iLen);
	}
}


/**
	Local function to handle the USB-CDC class requests
		
	@param [in] pSetup
	@param [out] piLen
	@param [out] ppbData
 */
static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, uint8_t **ppbData)
{
	int i;
	switch (pSetup->bRequest) {

	// set line coding
	case SET_LINE_CODING:

		//memcpy((uint8_t *)&LineCoding, *ppbData, 7);
		*piLen = 7;
		for(i=0;i<7;i++)
			((uint8_t *)&LineCoding)[i] = (*ppbData)[i];

		break;

	// get line coding
	case GET_LINE_CODING:

		*ppbData = (uint8_t *)&LineCoding;
		*piLen = 7;
		break;

	// set control line state
	case SET_CONTROL_LINE_STATE:
		// bit0 = DTR, bit = RTS

		break;

	default:
		return False;
	}
	return True;
}


/**
	Initialises the VCOM port.
	Call this function before using VCOM_puttchar or VCOM_getchar
 */
void VCOM_init(void)
{
	fifo_init(&txfifo, txdata);
	fifo_init(&rxfifo, rxdata);
}

 void VCOM_TxBuff_init(void)
{
	fifo_init(&txfifo, txdata);
}
 

void VCOM_RxBuff_init(void)
{
	fifo_init(&rxfifo, rxdata);
}

/**
	Writes one character to VCOM port
	
	@param [in] c character to write
	@returns character written, or EOF if character could not be written
 */
int VCOM_putchar(int c)
{
	return fifo_put(&txfifo, c) ? c : EOF;
}


/**
	Reads one character from VCOM port
	
	@returns character read, or EOF if character could not be read
 */
int VCOM_getchar(void)
{
	uint8_t c;
	
	return fifo_get(&rxfifo, &c) ? c : EOF;
}


int VCOM_getchar1(uint8_t *t) //data stored in pointer and returns 1 or 0
{
	uint8_t c;
	
	if(fifo_get(&rxfifo, &c)){
		*t = c;
		return 1;
	}
	else{
		return 0;
	}
	
	return 0;
}

int VCOM_getcharN(int num, uint8_t *t) //data stored into array
{
	uint8_t c;
	int i = 0;
	int count = 0;
	if(num > VCOM_FIFO_SIZE){ //invalid size
		return 0;
	}
	
	for(i = 0 ; i < num ; i++){
		while(!fifo_get(&rxfifo, &c)){ //wait for data
			if(count >= USB_TIMEOUT_COUNT){ //timeout
				return 0;
			}
			count++;
		}
		*(t+i) = c;
		count = 0;
	}
	
	return 1;
}

uint16_t VCOM_ChkRcv(void){
	// printf("\n vcom");
	return ring_is_empty (&ring_rxUSB);
}


uint8_t VCOM_GetCh(void){
	while(ring_getc (&ring_rxUSB)){
		return (uint8_t)-1;
	}
	return ring_rxUSB.dt_got;
}


/**
	Interrupt handler
	
	Simply calls the USB ISR
 */
//void USBIntHandler(void)
void USB_IRQHandler(void)
{
	// printf("\n1");
	USBHwISR();
}


static void USBFrameHandler(U16 wFrame)
{
	wFrame = wFrame;
	if (fifo_avail(&txfifo) > 0) {
		// data available, enable NAK interrupt on bulk in
		USBHwNakIntEnable(INACK_BI);
	}
}

void enable_USB_interrupts(void);


/*************************************************************************
	main
	====
**************************************************************************/
int usbSerialInit()
{
	// initialise stack
	USBInit();

	// register descriptors
	USBRegisterDescriptors(abDescriptors);

	// register class request handler
	USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

	// register endpoint handlers
	USBHwRegisterEPIntHandler(INT_IN_EP, NULL);
	USBHwRegisterEPIntHandler(BULK_IN_EP, BulkIn);
	USBHwRegisterEPIntHandler(BULK_OUT_EP, BulkOut);
	
	// register frame handler
	USBHwRegisterFrameHandler(USBFrameHandler);

	// enable bulk-in interrupts on NAKs
	USBHwNakIntEnable(INACK_BI);

	// initialise VCOM
	VCOM_init();
	
#ifdef MOD_USB
	modUSB_init();
#endif

/* CodeRed - comment out original interrupt setup code	
	// set up USB interrupt
	VICIntSelect &= ~(1<<22);               // select IRQ for USB
	VICIntEnable |= (1<<22);

	(*(&VICVectCntl0+INT_VECT_NUM)) = 0x20 | 22; // choose highest priority ISR slot 	
	(*(&VICVectAddr0+INT_VECT_NUM)) = (int)USBIntHandler;
	
	enableIRQ();
*/	

// CodeRed - add in interrupt setup code for RDB1768

#ifndef POLLED_USBSERIAL
	//enable_USB_interrupts();
	NVIC_EnableIRQ(USB_IRQn); 
	
#endif
		
	// connect to bus
		

	USBHwConnect(True);


	return 0;
}

void VCOM_puts(const char* str)
{
	while(*str != '\0')
	{
		VCOM_putchar(*str++);
	}
}
void VCOM_putc(char c)
{
	while(VCOM_putchar(c) == EOF);
}

int VCOM_getc()
{
	int c;
	c = EOF;
    c = VCOM_getchar();

	if(c != EOF)
	{	
		// VCOM_printf("\nreceived ");
		// VCOM_putc(c);
	}
	return c;
}

void VCOM_putHex(uint8_t hex)
{
	uint8_t temp;
	VCOM_puts("0x");
	
	temp = ((hex >> 4) & 0x0F) + 0x30;// add 0x30 to get ASCII value
	if(temp > 0x39)
		temp += 7; // alphabet, not numeral
	VCOM_putc((char)temp); 
	
	temp = ((hex) & 0x0F) + 0x30;
	if(temp > 0x39)
		temp += 7;
		
	VCOM_putc((char)temp);
}
void VCOM_gets(char* str)
{
	char c;
	c = VCOM_getc();
	while((c != '\n') && (c != '\r'))
	{
		*str++ = c;
		c = VCOM_getc();
	}
	*str = '\0';
}
char VCOM_getc_echo()
{
	char c;
	c = VCOM_getc();
	VCOM_putc(c);
	return c;
}
void VCOM_gets_echo(char *str)
{
	char c;
	c = VCOM_getc_echo();
	while((c != '\n') && (c != '\r'))
	{
		*str++ = c;
		c = VCOM_getc_echo();
	}
	*str = '\0';
}

/* Original code by ELM_ChaN. Modified by Martin Thomas */
int xatoi (char **str, long *res)
{
	uint32_t val;
	uint8_t c, radix, s = 0;


	while ((c = **str) == ' ') (*str)++;
	if (c == '-') {
		s = 1;
		c = *(++(*str));
	}
	if (c == '0') {
		c = *(++(*str));
		if (c <= ' ') {
			*res = 0; return 1;
		}
		if (c == 'x') {
			radix = 16;
			c = *(++(*str));
		} else {
			if (c == 'b') {
				radix = 2;
				c = *(++(*str));
			} else {
				if ((c >= '0')&&(c <= '9'))
					radix = 8;
				else
					return 0;
			}
		}
	} else {
		if ((c < '1')||(c > '9'))
			return 0;
		radix = 10;
	}
	val = 0;
	while (c > ' ') {
		if (c >= 'a') c -= 0x20;
		c -= '0';
		if (c >= 17) {
			c -= 7;
			if (c <= 9) return 0;
		}
		if (c >= radix) return 0;
		val = val * radix + c;
		c = *(++(*str));
	}
	if (s) val = -val;
	*res = val;
	return 1;
}



void xitoa (long val, int radix, int len)
{
	uint8_t c, r, sgn = 0, pad = ' ';
	uint8_t s[20], i = 0;
	uint32_t v;


	if (radix < 0) {
		radix = -radix;
		if (val < 0) {
			val = -val;
			sgn = '-';
		}
	}
	v = val;
	r = radix;
	if (len < 0) {
		len = -len;
		pad = '0';
	}
	if (len > 20) return;
	do {
		c = (uint8_t)(v % r);
		if (c >= 10) c += 7;
		c += '0';
		s[i++] = c;
		v /= r;
	} while (v);
	if (sgn) s[i++] = sgn;
	while (i < len)
		s[i++] = pad;
	do
		VCOM_putc(s[--i]);
	while (i);
}

void VCOM_printf (const char* str, ...)
{
	printf("\nto USB %s\n",str);
	// printf();
	va_list arp;
	int d, r, w, s, l;


	va_start(arp, str);

	while ((d = *str++) != 0) {
		if (d != '%') {
			VCOM_putc(d); continue;
		}
		d = *str++; w = r = s = l = 0;
		if (d == '0') {
			d = *str++; s = 1;
		}
		while ((d >= '0')&&(d <= '9')) {
			w += w * 10 + (d - '0');
			d = *str++;
		}
		if (s) w = -w;
		if (d == 'l') {
			l = 1;
			d = *str++;
		}
		if (!d) break;
		if (d == 's') {
			VCOM_puts(va_arg(arp, char*));
			continue;
		}
		if (d == 'c') {
			VCOM_putc((char)va_arg(arp, int));
			continue;
		}
		if (d == 'u') r = 10;
		if (d == 'd') r = -10;
		if (d == 'X' || d == 'x') r = 16; // 'x' added by mthomas in increase compatibility
		if (d == 'b') r = 2;
		if (!r) break;
		if (l) {
			xitoa((long)va_arg(arp, long), r, w);
		} else {
			if (r > 0)
				xitoa((unsigned long)va_arg(arp, int), r, w);
			else
				xitoa((long)va_arg(arp, int), r, w);
		}
	}

	va_end(arp);
}


int modUSB_init(){
#ifdef MOD_USB
	modInit(&modbusUSB, &ring_rxUSB, MOD_USB);
#endif
	ring_init( &ring_rxUSB, buff_rxUSB, USB_SIZE );
	return 1;
}
