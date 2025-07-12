/*
	IAP, an IAP (in application programming) driver for LPC2148 microcontroller	
	Copyright (C) 2008 Andreas Weschenfelder (A_Weschenfelder@web.de)

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


#include "LPC214x.h" 
#include "iap_fkt.h"
#include "print_dbg.h"

#define putstring_serial0 print_string
#define put_int2str print_ulong

#define IAP_LOCATION 0x7ffffff1

static unsigned long iap_cmd[5];			// IAP command table
static unsigned long iap_res[3];			//     result  table

//__attribute ((aligned(32))) char my_bl_data[512];

typedef void (*IAP)(unsigned int [],unsigned int[]);
IAP iap_entry;

/*
 * Call IAP function.
 *
 * Note: Direct XTAL mode is required
 * for IAP functions (PLL turned off)!
 *
 * Return status.
 */
static long iap(long code, long p1, long p2, long p3, long p4)
{

	iap_cmd[0] = code;				// set command code
	iap_cmd[1] = p1;				//     1st param
	iap_cmd[2] = p2;				//     2nd param
	iap_cmd[3] = p3;				//     3rd param
	iap_cmd[4] = p4;				//     4th param

	//_pll_off();					// disconnect PLL
	((void (*)())IAP_LOCATION)(iap_cmd, iap_res);	// IAP entry point
	//_pll_on();					// re-connect PLL

	return *iap_res;				// return status

}



/*Read part ID.

Input:			Command:	code: 54
Parameters: 	None
Return:			Code CMD_SUCCESS |
Result:			Result0: Part Identification Number (see Table 21–302 “LPC214x Part Identification numbers” on page 307 for details)
Description:	This command is used to read the part identification number.
*/
unsigned long read_part_id(void)
{

	if (iap(READ_PART_ID, 0, 0, 0, 0)) 
		{
		putstring_serial0("\n\r-- Error - Reading part ID --\n\r");
		return 0;
		}

	if (iap_res[1]==PART_ID_LPC2148)
		{putstring_serial0("\n\rPart ID: LPC2148\n\r");}
		else
		{
		putstring_serial0("\n\rPart ID (no LPC2148): ");
		put_int2str(iap_res[1]);
		putstring_serial0("\n\r");
		}
	return iap_res[1]; 
}



/*Read boot code version.

Input:			Command:	code: 55
Parameters: 	None
Return:			Code CMD_SUCCESS |
Result:			Result0: 2 bytes of boot code version number in ASCII format. It is to be interpreted as <byte1(Major)>.<byte0(Minor)>
Description:	This command is used to read the boot code version number.
*/
long read_boot_code_version(void)
{
	return iap(READ_BOOT_CODE_VERSION, 0, 0, 0, 0);
}


unsigned int boot_code_version(void)
{
	if (read_boot_code_version()) 
		{
		putstring_serial0("-- Error - Reading boot code version --");
		return 0;
		}
	putstring_serial0("\n\rBoot code version: ");
	put_int2str((unsigned char)(iap_res[1]>>8));
	putstring_serial0(".");
	put_int2str((unsigned char)iap_res[1]);
	putstring_serial0("\n\r");
	return  (unsigned int)(iap_res[1]&0xFFFF); 
}

#if 0

/*
 * Write sector.
 */
void s_write(void)
{
char err;
static char data[512] = {'T','E','S','T'};			// data buffer
long buffer_vic;
//Interrupt Enable Speichern
buffer_vic = VICIntEnable;
//clear vic
VICIntEnClr = 0xffffffff;

long addr = 0x0007C000; // start of sector 26...

// If not blank, dump sector data.
//if (blank_check_sector( get_sec_num(0x30000), get_sec_num(0x30000) ) == SECTOR_NOT_BLANK) {
	if (blank_check_sector( get_sec_num(addr), get_sec_num(addr) ) == SECTOR_NOT_BLANK) {
		putstring_serial0("\nSector not blank!\n");
		VICIntEnable = buffer_vic;		//interrupts zurueckschreiben
		return;
	}

	//putstring_serial0("\n\rEnter data OR single RETURN to abort:");


	//if (prepare_sector( get_sec_num(0x30000), get_sec_num(0x30000)))
	if (prepare_sector( get_sec_num(addr), get_sec_num(addr))) {
		putstring_serial0("\n\r-- Error: PREPARE_SECTOR_FOR_WRITE_OPERATION: ");
	}else{
		err = copy_ram_to_flash( addr, (long)data, sizeof(data));
		if (err){
			putstring_serial0("\n\r-- Error: COPY_RAM_TO_FLASH: ");
			put_int2str(err);
			putstring_serial0(" --\n\r");
		}else{
			if (compare(addr,(long)data, sizeof(data))){
				putstring_serial0("\n\r-- Error: COMPARE --\n\r");
			}else{
				putstring_serial0("\n\rData successfully written.\n\r");
			}
		}
	}

	VICIntEnable = buffer_vic;		//interrupts zurueckschreiben
}


int lpc_write(unsigned long addr)
{
char err;
//static char data[512] = {'T','E','S','T'};			// data buffer
long buffer_vic;
//Interrupt Enable Speichern
buffer_vic = VICIntEnable;
//clear vic
VICIntEnClr = 0xffffffff;


// If not blank, dump sector data.
if (blank_check_sector( get_sec_num(addr), get_sec_num(addr) ) == SECTOR_NOT_BLANK) {
	putstring_serial0("\nWarning: Sector not blank!\n");
	//VICIntEnable = buffer_vic;		//interrupts zurueckschreiben
	//return 1;
}


if (prepare_sector( get_sec_num(addr), get_sec_num(addr))){
	putstring_serial0("\n\r-- Error: PREPARE_SECTOR_FOR_WRITE_OPERATION: ");
	return 2;
}else{
	err = copy_ram_to_flash( addr, (unsigned int)my_bl_data, sizeof(my_bl_data));
	if (err)
		{
		putstring_serial0("\n\r-- Error: COPY_RAM_TO_FLASH: ");
		put_int2str(err);
		putstring_serial0(" --\n\r");
		return 3;
		}
		else
		{
		if (compare(addr,(long)my_bl_data, sizeof(my_bl_data)))	
			{
			putstring_serial0("\n\r-- Error: COMPARE --\n\r");
			return 4;
			}
			else
			{
			putstring_serial0("\n\rData successfully written.\n\r");
			return 0;
			}
		}
	}

VICIntEnable = buffer_vic;		//interrupts zurueckschreiben
}

/*
int config_write_iap(unsigned long flash_addr, uint8_t * data, int len)
{
int ret_val=0;
char err;
long buffer_vic;
char local_data[512]; // assume config data will never exceed 512 bytes...
int i;
//Interrupt Enable copy
buffer_vic = VICIntEnable;
//clear vic
VICIntEnClr = 0xffffffff;

// copy data onto local_data
	for(i=0;i<len;i++, data++)
		local_data[i] = *data;

	// If not blank, dump sector data.
	if (blank_check_sector( get_sec_num(flash_addr), get_sec_num(flash_addr) ) == SECTOR_NOT_BLANK) {
		putstring_serial0("\nWarning: Sector not blank going to overwrite!\n");
		//VICIntEnable = buffer_vic;		//interrupts zurueckschreiben
		//return 1;
	}


	if (prepare_sector( get_sec_num(flash_addr), get_sec_num(flash_addr))){
		putstring_serial0("\n\r-- Error: PREPARE_SECTOR_FOR_WRITE_OPERATION: ");
		ret_val = 2;
	}else{
		err = copy_ram_to_flash( flash_addr, (long)local_data, sizeof(local_data));
		if (err){
			putstring_serial0("\n\r-- Error: COPY_RAM_TO_FLASH: ");
			put_int2str(err);
			putstring_serial0(" --\n\r");
			ret_val = 3;
		}else{
			if (compare(flash_addr,(long)local_data, sizeof(local_data))){
				putstring_serial0("\n\r-- Error: COMPARE --\n\r");
				ret_val = 4;
			}else{
				putstring_serial0("\n\rData successfully written.\n\r");
				ret_val = 0;
			}
		}
	}

	VICIntEnable = buffer_vic;		//interrupts zurueckschreiben
	return ret_val;
}
*/

#endif

/*Copy Ram to Flash

Input:		 	Command: 	code: 51
				Param0:		(DST): Destination Flash address where data bytes are to be written. This address should be a 256 byte boundary.
				Param1:		(SRC): Source RAM address from which data bytes are to be read. This address should be a word boundary.
				Param2: 	Number of bytes to be written. Should be 256 | 512 | 1024 | 4096.
				Param3: 	System Clock Frequency (CCLK) in kHz.
Return:			Code CMD_SUCCESS |
				SRC_ADDR_ERROR (Address not a word boundary) |
				DST_ADDR_ERROR (Address not on correct boundary) |
				SRC_ADDR_NOT_MAPPED |
				DST_ADDR_NOT_MAPPED |
				COUNT_ERROR (Byte count is not 256 | 512 | 1024 | 4096) |
				SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION |
				BUSY |
Result:			None
Description:	This command is used to program the flash memory. The affected sectors should
				be prepared first by calling "Prepare Sector for Write Operation" command. The
				affected sectors are automatically protected again once the copy command is
				successfully executed. The boot sector can not be written by this command.
*/
long copy_ram_to_flash( long tmp_adr_dst,long tmp_adr_src, long tmp_size )
{
return iap(COPY_RAM_TO_FLASH, tmp_adr_dst, tmp_adr_src, tmp_size, _XTAL);
}



/*Compare

Input:			Command:	code: 56
				Param0:		(DST): Starting Flash or RAM address of data bytes to be compared. This address should be a word boundary.
				Param1:		(SRC): Starting Flash or RAM address of data bytes to be compared. This address should be a word boundary.
				Param2:		Number of bytes to be compared; should be a multiple of 4.
Return:			Code CMD_SUCCESS |
				COMPARE_ERROR |
				COUNT_ERROR (Byte count is not a multiple of 4) |
				ADDR_ERROR |
				ADDR_NOT_MAPPED
Result:			Result0: Offset of the first mismatch if the Status Code is COMPARE_ERROR.
Description:	This command is used to compare the memory contents at two locations.
				The result may not be correct when the source or destination includes any
				of the first 64 bytes starting from address zero. The first 64 bytes can be
				re-mapped to RAM.
*/
long compare(long tmp_adr_dst,long tmp_adr_src, long tmp_size)
{
return iap(COMPARE, tmp_adr_dst, tmp_adr_src, tmp_size, 0);
} 


#if 0
// Erase sector.
void s_erase(void)
{
// If blank, skip erase.
if (!blank_check_sector( get_sec_num(0x30000), get_sec_num(0x30000) ))
	{
	putstring_serial0("\n\rSector already blank!\n\r");
	return;
	}

if (prepare_sector( get_sec_num(0x30000), get_sec_num(0x30000) ))
	{putstring_serial0("\n\r-- Error: PREPARE_SECTOR_FOR_WRITE_OPERATION --\n\r");
	return;}
if (erase( get_sec_num(0x30000), get_sec_num(0x30000)  ))
	{putstring_serial0("\n\r-- Error: ERASE_SECTOR --\n\r");
	return;}
if (blank_check_sector( get_sec_num(0x30000), get_sec_num(0x30000) )) 
	{putstring_serial0("\n\r-- Error: BLANK_CHECK_SECTOR --\n\r");
	return;}

putstring_serial0("\n\rSector successfully erased.\n\r");
}

unsigned int lpc_erase(unsigned long tmp_adr)
{
// If blank, skip erase.
if (!blank_check_sector( get_sec_num(tmp_adr), get_sec_num(tmp_adr) ))
	{
	putstring_serial0("\n\rSector already blank!\n\r");
	return 1;
	}

if (prepare_sector( get_sec_num(tmp_adr), get_sec_num(tmp_adr) ))
	{putstring_serial0("\n\r-- Error: PREPARE_SECTOR_FOR_WRITE_OPERATION --\n\r");
	return 2;}
if (erase( get_sec_num(tmp_adr), get_sec_num(tmp_adr)  ))
	{putstring_serial0("\n\r-- Error: ERASE_SECTOR --\n\r");
	return 3;}
if (blank_check_sector( get_sec_num(tmp_adr), get_sec_num(tmp_adr) )) 
	{putstring_serial0("\n\r-- Error: BLANK_CHECK_SECTOR --\n\r");
	return 4;}

putstring_serial0("\n\rSector successfully erased.\n\r");
return 0;
}

unsigned int lpc_erase_sector(char sec)
{
// If blank, skip erase.
if (!blank_check_sector( sec, sec) )
	{
	putstring_serial0("\n\rSector already blank!\n\r");
	return 1;
	}

if (prepare_sector( sec, sec ))
	{putstring_serial0("\n\r-- Error: PREPARE_SECTOR_FOR_WRITE_OPERATION --\n\r");
	return 2;}
if (erase( sec, sec ) )
	{putstring_serial0("\n\r-- Error: ERASE_SECTOR --\n\r");
	return 3;}
if (blank_check_sector( sec, sec )) 
	{putstring_serial0("\n\r-- Error: BLANK_CHECK_SECTOR --\n\r");
	return 4;}

putstring_serial0("\n\rSector successfully erased.\n\r");
return 0;
}

#endif

/*Prepare sector(s) for write operation

Input:			Command:	code: 50
				Param0:		Start Sector Number
				Param1:		End Sector Number (should be greater than or equal to start sector number).
Return 			Code CMD_SUCCESS |
				BUSY |
				INVALID_SECTOR
Result 			None
Description: 	This command must be executed before executing "Copy RAM to Flash" or
				"Erase Sector(s)" command. Successful execution of the "Copy RAM to Flash" or
				"Erase Sector(s)" command causes relevant sectors to be protected again. The
				boot sector can not be prepared by this command.
*/
long prepare_sector( long tmp_sect1, long tmp_sect2 )
{
return iap(PREPARE_SECTOR_FOR_WRITE_OPERATION, tmp_sect1, tmp_sect2, 0, 0);
}



/*erase

Input:				Command:	 code: 52
					Param0:		Start Sector Number
					Param1: 	End Sector Number (should be greater than or equal to start sector number).
					Param2: 	System Clock Frequency (CCLK) in kHz.
Return:				Code CMD_SUCCESS |
					BUSY |
					SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION |
					INVALID_SECTOR
Result:				None
Description:		This command is used to erase a sector or multiple sectors of on-chip Flash
					memory. The boot sector can not be erased by this command.
*/
long erase(long tmp_sect1, long tmp_sect2 )
{
return iap(ERASE_SECTOR, tmp_sect1, tmp_sect2, _XTAL, 0);
}


/*Blank Check Sector

Input:				Command:	code: 53
					Param0: 	Start Sector Number
					Param1: 	End Sector Number (should be greater than or equal to start sector number).
Return:				Code CMD_SUCCESS |
					BUSY |
					SECTOR_NOT_BLANK |
					INVALID_SECTOR
Result:				Result0: 	Offset of the first non blank word location if the Status Code is SECTOR_NOT_BLANK.
					Result1: 	Contents of non blank word location.
Description:		This command is used to blank check a sector or multiple sectors of on-chip Flash
					memory. To blank check a single sector use the same "Start" and "End" sector numbers.
*/
long blank_check_sector(long tmp_sect1, long tmp_sect2)
{
return iap(BLANK_CHECK_SECTOR, tmp_sect1, tmp_sect2, 0, 0);
}


#define PLLCON_PLLE     0x01               // PLL Enable
#define PLLCON_PLLD     0x00               // PLL Disable
#define PLLCON_PLLC     0x03               // PLL Connect
#define PLLSTAT_PLOCK   0x0400             // PLL Lock Status




/*
 * Switch CPU to standard XTAL
 */
void _pll_off(void) 
{
#if 0
/*
  PLL48CFG = (1-1)|((8-1)<<5); // M=1 und P=8 (multiplikatoren von 0 sind nicht erlaubt)
  
  
  //Nochmal nach rechnen:
  //CPU TAKT = M * FOSC = 1 * 12000000 Hz = 120000000 Hz
  //CCO TAKT = 2 * P * M * FOSC = 2 * 8 * 1 * 12000000 Hz = 192000000 Hz
  
  //F_CCO muss zwischen 156 MHz und 320 MHz liegen
 
  //Mit diesen Werten ist alles innerhalb der Spezifikationen aus dem Datenblatt.  
 
  //Der CCO (Current Controlled Oscillator) ist ein Bestandteil der PLL.
  
 
  PLL48CON = 0x01; // PLL aktivieren
 
  PLL48FEED = 0xAA;  //PLL Feed-Sequenz
  PLL48FEED = 0x55;
  while ( !( PLL48STAT & PLOCK ) ); // Darauf warten, dass die ÿnderungen übernommen werden 
 
  
  //Mit MAMTIM werden die Waitstates beim flash speicherzugriff eingestellt, das Datenblatt empfiehlt folgende Werte:
  //1 - bei unter 20 Mhz
  //2 - bei 20-40 Mhz 
  //3 - bei über 40 Mhz
  
    
  MAMTIM = 3; 
 
  PLL48CON = 0x03; 		// PLL aktivieren und mit dem internen Taktgeber verbinden
  PLL48FEED = 0xAA;  	// PLL Feed Sequence
  PLL48FEED = 0x55;
*/
PLLCON = PLLCON_PLLD;
feed();
#endif
}

/*
 * Switch to Original CPU-XTAL
 */
void _pll_on(void) 
{
#if 0
/*
//TODO
	// Enabling the PLL 	
	PLLCON = 0x1;
	feed();
  
	// Wait for the PLL to lock to set frequency
	while(!(PLLSTAT & 0x400)) ;
  
	// Connect the PLL as the clock source
	PLLCON = 0x3;
	feed();
*/	
PLLCON = PLLCON_PLLE;
feed();
while ((PLLSTAT & PLLSTAT_PLOCK) == 0);
PLLCON = PLLCON_PLLC;
feed();
#endif
}
/*
static flashSectorToAddress_t flashSectorToAddress [] = 
{
  { 0x00000000, 4096 },   // 0
  { 0x00001000, 4096 },   // 1
  { 0x00002000, 4096 },   // 2
  { 0x00003000, 4096 },   // 3
  { 0x00004000, 4096 },   // 4
  { 0x00005000, 4096 },   // 5
  { 0x00006000, 4096 },   // 6
  { 0x00007000, 4096 },   // 7
  { 0x00008000, 32768 },  // 8
  { 0x00010000, 32768 },  // 9
  { 0x00018000, 32768 },  // 10
  { 0x00020000, 32768 },  // 11
  { 0x00028000, 32768 },  // 12
  { 0x00030000, 32768 },  // 13
  { 0x00038000, 32768 },  // 14
  { 0x00040000, 32768 },  // 15
  { 0x00048000, 32768 },  // 16
  { 0x00050000, 32768 },  // 17
  { 0x00058000, 32768 },  // 18
  { 0x00060000, 32768 },  // 19
  { 0x00068000, 32768 },  // 20
  { 0x00070000, 32768 },  // 21
  { 0x00078000, 4096 },   // 22
  { 0x00079000, 4096 },   // 23
  { 0x0007a000, 4096 },   // 24
  { 0x0007b000, 4096 },   // 25
  { 0x0007c000, 4096 },   // 26
};
*/	
/*
 * Convert 'addr' to sector number
 */

/*
Sector-table
Sec		Size(kb)		Range
0 		4 			0X00000000 - 0X00000FFF
1 		4 			0X00001000 - 0X00001FFF
2 		4 			0X00002000 - 0X00002FFF
3 		4 			0X00003000 - 0X00003FFF
4 		4 			0X00004000 - 0X00004FFF
5 		4 			0X00005000 - 0X00005FFF
6 		4 			0X00006000 - 0X00006FFF
7 		4 			0X00007000 - 0X00007FFF
8 		32 			0x00008000 - 0X0000FFFF
9 		32 			0x00010000 - 0X00017FFF
10		32 			0x00018000 - 0X0001FFFF
11 		32 			0x00020000 - 0X00027FFF
12		32 			0x00028000 - 0X0002FFFF
13		32 			0x00030000 - 0X00037FFF
14 		32 			0x00038000 - 0X0003FFFF
15 		32 			0x00040000 - 0X00047FFF
16		32 			0x00048000 - 0X0004FFFF
17		32 			0x00050000 - 0X00057FFF
18		32 			0x00058000 - 0X0005FFFF
19 		32 			0x00060000 - 0X00067FFF
20 		32 			0x00068000 - 0X0006FFFF
21 		32 			0x00070000 - 0X00077FFF
22		4 			0x00078000 - 0X00078FFF
23 		4 			0x00079000 - 0X00079FFF
24		4 			0x0007A000 - 0X0007AFFF
25 		4 			0x0007B000 - 0X0007BFFF
26 		4 			0x0007C000 - 0X0007CFFF
*/
unsigned int get_sec_num(unsigned long addr)
{
if ((addr>= 0x00000000)&&(addr<=0x00000FFF))
	{return 0;}
if ((addr>= 0x00001000)&&(addr<=0x00001FFF))
	{return 1;}
if ((addr>= 0x00002000)&&(addr<=0x00002FFF))
	{return 2;}
if ((addr>= 0x00003000)&&(addr<=0x00003FFF))
	{return 3;}
if ((addr>= 0x00004000)&&(addr<=0x00004FFF))
	{return 4;}
if ((addr>= 0x00005000)&&(addr<=0x00005FFF))
	{return 5;}
if ((addr>= 0x00006000)&&(addr<=0x00006FFF))
	{return 6;}
if ((addr>= 0x00007000)&&(addr<=0x00007FFF))
	{return 7;}
if ((addr>= 0x00008000)&&(addr<=0x0000FFFF))
	{return 8;}
if ((addr>= 0x00010000)&&(addr<=0x00017FFF))
	{return 9;}
if ((addr>= 0x00018000)&&(addr<=0x0001FFFF))
	{return 10;}
if ((addr>= 0x00020000)&&(addr<=0x00027FFF))
	{return 11;}
if ((addr>= 0x00028000)&&(addr<=0x0002FFFF))
	{return 12;}
if ((addr>= 0x00030000)&&(addr<=0x00037FFF))
	{return 13;}
if ((addr>= 0x00038000)&&(addr<=0x0003FFFF))
	{return 14;}
if ((addr>= 0x00040000)&&(addr<=0x00047FFF))
	{return 15;}
if ((addr>= 0x00048000)&&(addr<=0x0004FFFF))
	{return 16;}
if ((addr>= 0x00050000)&&(addr<=0x00057FFF))
	{return 17;}
if ((addr>= 0x00058000)&&(addr<=0x0005FFFF))
	{return 18;}
if ((addr>= 0x00060000)&&(addr<=0x00067FFF))
	{return 19;}
if ((addr>= 0x00068000)&&(addr<=0x0006FFFF))
	{return 20;}
if ((addr>= 0x00070000)&&(addr<=0x00077FFF))
	{return 21;}
if ((addr>= 0x00078000)&&(addr<=0x00078FFF))
	{return 22;}
if ((addr>= 0x00079000)&&(addr<=0x00079FFF))
	{return 23;}
if ((addr>= 0x0007A000)&&(addr<=0x0007AFFF))
	{return 24;}
if ((addr>= 0x0007B000)&&(addr<=0x0007BFFF))
	{return 25;}
if ((addr>= 0x0007C000)&&(addr<=0x0007CFFF))
	{return 26;}
	
return 0xFF;
}



