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


/**
	@file
	primitive types used in the USB stack
 */

/*
 *  Modified for mbed NXP LPC1768 & LPCXpresso board
 *  original	LPC214x USB stack beta by bertrik
 *  			target-20070727.tar.gr (119.5KB)
 *  			http://sourceforge.net/projects/lpcusb/
 *  By Kenji Arai / JH1PJL on September 25th, 2010
 *  	October 3rd, 2010
 */

#ifndef _TYPE_H_
#define _TYPE_H_

typedef unsigned char		U8;		/**< unsigned 8-bit */
typedef unsigned short int	U16;	/**< unsigned 16-bit */
typedef unsigned int		U32;	/**< unsigned 32-bit */

typedef int					BOOL;	/**< #TRUE or #FALSE */

#define	TRUE	1					/**< TRUE */
#define FALSE	0					/**< FALSE */

#ifndef NULL
#define NULL	((void*)0)			/**< NULL pointer */
#endif

/* some other useful macros */
// #define MIN(x,y)	((x)<(y)?(x):(y))	/**< MIN */
// #define MAX(x,y)	((x)>(y)?(x):(y))	/**< MAX */

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

// added #ifndef   by Kenji Arai / JH1PJL on March 21st,2010
#ifndef _INTEGER
typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned long  DWORD;
//typedef unsigned int   BOOL;
#endif

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

/* Pointer to Function returning Void (any number of parameters) */
typedef void (*PFV)();

#endif /* _TYPE_H_ */


/*****************************************************************************
 *   type.h:  Type definition Header file for NXP LPC17xx Family 
 *   Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.08.21  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#if 0
#ifndef __TYPE_H__
#define __TYPE_H__

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

// added #ifndef   by Kenji Arai / JH1PJL on March 21st,2010
#ifndef _INTEGER
typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned long  DWORD;
typedef unsigned int   BOOL;
#endif

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

/* Pointer to Function returning Void (any number of parameters) */
typedef void (*PFV)();

#endif  /* __TYPE_H__ */
#endif
