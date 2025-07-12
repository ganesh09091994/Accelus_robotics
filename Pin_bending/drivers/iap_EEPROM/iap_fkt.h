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


#ifndef __IAP_FKT_H__
#define __IAP_FKT_H__


#include <stdio.h>
#include <stdint.h>

#define _XTAL 60000


//Adressen
#define FLASH_APPI_START	0x3000
#define RAM_START			0x40000000
#define MAP_TO_RAM			0x02


//IAP-Commands
#define	PREPARE_SECTOR_FOR_WRITE_OPERATION	50
#define	COPY_RAM_TO_FLASH					51
#define	ERASE_SECTOR						52
#define	BLANK_CHECK_SECTOR					53
#define READ_PART_ID						54
#define	READ_BOOT_CODE_VERSION				55
#define COMPARE								56	

//ISP_READ_MEMORY_CMD
//ISP_Go <address> <mode>

//PartIDs
#define PART_ID_LPC2141 0x0402FF01
#define PART_ID_LPC2142 0x0402FF11 
#define PART_ID_LPC2144 0x0402FF12 
#define PART_ID_LPC2146 0x0402FF23 
#define PART_ID_LPC2148 0x0402FF25 


//IAP-Status-Codes

#define CMD_SUCCESS			0 	//Command is executed successfully.
#define INVALID_COMMAND 	1	//Invalid command.
#define SRC_ADDR_ERROR 		2	//Source address is not on a word boundary.
#define DST_ADDR_ERROR 		3	//Destination address is not on a correct boundary.
#define SRC_ADDR_NOT_MAPPED 4	//Source address is not mapped in the memory map. Count value is taken in to consideration where applicable.
#define DST_ADDR_NOT_MAPPED 5	//Destination address is not mapped in the memory map. Count value is taken in to consideration where applicable.
#define COUNT_ERROR 		6	//Byte count is not multiple of 4 or is not a permitted value.
#define INVALID_SECTOR 		7	//Sector number is invalid
#define SECTOR_NOT_BLANK 	8	//Sector is not blank.
#define SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION 9	//Command to prepare sector for write operation was not executed.
#define COMPARE_ERROR 		10	//Source and destination data is not same.
#define BUSY 				11	//Flash programming hardware interface is busy







//static long iap(long code, long p1, long p2, long p3, long p4);
unsigned long read_part_id(void);
unsigned int boot_code_version(void);
void s_write(void);
int lpc_write(unsigned long addr);
void s_erase(void);
unsigned int lpc_erase(unsigned long tmp_adr);
unsigned int lpc_erase_sector(char sec);
//int config_write_iap(unsigned long flash_addr, uint8_t * data, int len);


extern __attribute ((aligned(32))) char my_bl_data[512];


long blank_check_sector(long tmp_sect1, long tmp_sect2);
long compare(long tmp_adr_dst,long tmp_adr_src, long tmp_size);
long erase(long tmp_sect1, long tmp_sect2 );
long copy_ram_to_flash( long tmp_adr_dst,long tmp_adr_src, long tmp_size );
long prepare_sector( long tmp_sect1, long tmp_sect2 );
long read_boot_code_version(void);

unsigned int get_sec_num(unsigned long addr);

void _pll_off(void);
void _pll_on(void) ;

#endif
