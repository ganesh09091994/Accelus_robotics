/* File : barebones/ee_printf.c
	This file contains an implementation of ee_printf that only requires a method to output a char to a UART without pulling in library code.

This code is based on a file that contains the following:
 Copyright (C) 2002 Michael Ringgaard. All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
 
 1. Redistributions of source code must retain the above copyright 
    notice, this list of conditions and the following disclaimer.  
 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.  
 3. Neither the name of the project nor the names of its contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission. 
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 SUCH DAMAGE.

*/

// #include <coremark.h>
#include <stdarg.h>
#include <stddef.h>
// #include <string.h>
#include "debug_frmwrk.h"
#include "ee_printf-arg.h"

#define ZEROPAD  	(1<<0)	/* Pad with zero */
#define SIGN    	(1<<1)	/* Unsigned/signed long */
#define PLUS    	(1<<2)	/* Show plus */
#define SPACE   	(1<<3)	/* Spacer */
#define LEFT    	(1<<4)	/* Left justified */
#define HEX_PREP 	(1<<5)	/* 0x */
#define UPPERCASE   (1<<6)	/* 'ABCDEF' */

#define is_digit(c) ((c) >= '0' && (c) <= '9')

#define putsnonl(c) UARTPuts(DEBUG_UART_PORT,c)

static char *lower_digits = "0123456789abcdefghijklmnopqrstuvwxyz";
static char *upper_digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static size_t ee_strnlen(const char *s, size_t count);

static size_t ee_strnlen(const char *s, size_t count){
	const char *sc;
	for (sc = s; *sc != '\0' && count--; ++sc);
	return sc - s;
}

static int ee_skip_atoi(const char **s){
	int i = 0;
	while (is_digit(**s)) i = i*10 + *((*s)++) - '0';
	return i;
}

static char *ee_number(char *str, long num, int base, int size, int precision, int type){
	char c, sign, tmp[66];
	char *dig = lower_digits;
	int i;

	if (type & UPPERCASE)  dig = upper_digits;
	if (type & LEFT) type &= ~ZEROPAD;
	if (base < 2 || base > 36) return 0;

	c = (type & ZEROPAD) ? '0' : ' ';
	sign = 0;
	if (type & SIGN){
		if (num < 0){
			sign = '-';
			num = -num;
			size--;
		}else if (type & PLUS){
			sign = '+';
			size--;
		}else if (type & SPACE){
			sign = ' ';
			size--;
		}
	}

	if (type & HEX_PREP){
		if (base == 16)
			size -= 2;
		else if (base == 8)
			size--;
	}

	i = 0;

	if (num == 0)
		tmp[i++] = '0';
	else{
		while (num != 0){
			tmp[i++] = dig[((unsigned long) num) % (unsigned) base];
			num = ((unsigned long) num) / (unsigned) base;
		}
	}

	if (i > precision) precision = i;
	size -= precision;
	if (!(type & (ZEROPAD | LEFT))) while (size-- > 0) *str++ = ' ';
	if (sign) *str++ = sign;

	if (type & HEX_PREP){
		if (base == 8)
			*str++ = '0';
		else if (base == 16){
			*str++ = '0';
			*str++ = lower_digits[33];
		}
	}

	if (!(type & LEFT)) while (size-- > 0) *str++ = c;
	while (i < precision--) *str++ = '0';
	while (i-- > 0) *str++ = tmp[i];
	while (size-- > 0) *str++ = ' ';

	return str;
}

static char *eaddr(char *str, unsigned char *addr, int size, int precision, int type){
	char tmp[24];
	char *dig = lower_digits;
	int i, len;

	if (type & UPPERCASE)  dig = upper_digits;
	len = 0;
	for (i = 0; i < 6; i++){
		if (i != 0) tmp[len++] = ':';
		tmp[len++] = dig[addr[i] >> 4];
		tmp[len++] = dig[addr[i] & 0x0F];
	}

	if (!(type & LEFT)) while (len < size--) *str++ = ' ';
	for (i = 0; i < len; ++i) *str++ = tmp[i];
	while (len < size--) *str++ = ' ';

	return str;
}

static char *iaddr(char *str, unsigned char *addr, int size, int precision, int type){
	char tmp[24];
	int i, n, len;

	len = 0;
	for (i = 0; i < 4; i++){
		if (i != 0) tmp[len++] = '.';
		n = addr[i];

		if (n == 0)
			tmp[len++] = lower_digits[0];
		else{
			if (n >= 100){
				tmp[len++] = lower_digits[n / 100];
				n = n % 100;
				tmp[len++] = lower_digits[n / 10];
				n = n % 10;
			}else if (n >= 10){
				tmp[len++] = lower_digits[n / 10];
				n = n % 10;
			}
			tmp[len++] = lower_digits[n];
		}
	}

	if (!(type & LEFT)) while (len < size--) *str++ = ' ';
	for (i = 0; i < len; ++i) *str++ = tmp[i];
	while (len < size--) *str++ = ' ';

	return str;
}

#ifdef HAS_FLOAT
char *ecvtbuf(double arg, int ndigits, int *decpt, int *sign, char *buf);
char *fcvtbuf(double arg, int ndigits, int *decpt, int *sign, char *buf);
static void ee_bufcpy(char *d, char *s, int count); 

void ee_bufcpy(char *pd, char *ps, int count) {
	char *pe=ps+count;
	while (ps!=pe)
		*pd++=*ps++;
}

static void parse_float(double value, char *buffer, char fmt, int precision){
	int decpt, sign, exp, pos;
	char *fdigits = NULL;
	char cvtbuf[80];
	int capexp = 0;
	int magnitude;

	if (fmt == 'G' || fmt == 'E'){
		capexp = 1;
		fmt += 'a' - 'A';
	}

	if (fmt == 'g'){
		fdigits = ecvtbuf(value, precision, &decpt, &sign, cvtbuf);
		magnitude = decpt - 1;
		if (magnitude < -4  ||  magnitude > precision - 1){
			fmt = 'e';
			precision -= 1;
		}else{
			fmt = 'f';
			precision -= decpt;
		}
	}

	if (fmt == 'e'){
		fdigits = ecvtbuf(value, precision + 1, &decpt, &sign, cvtbuf);

		if (sign) *buffer++ = '-';
		*buffer++ = *fdigits;
		if (precision > 0) *buffer++ = '.';
		ee_bufcpy(buffer, fdigits + 1, precision);
		buffer += precision;
		*buffer++ = capexp ? 'E' : 'e';

		if (decpt == 0){
			if (value == 0.0)
				exp = 0;
			else
				exp = -1;
		}else
			exp = decpt - 1;

		if (exp < 0){
			*buffer++ = '-';
			exp = -exp;
		}else
			*buffer++ = '+';

		buffer[2] = (exp % 10) + '0';
		exp = exp / 10;
		buffer[1] = (exp % 10) + '0';
		exp = exp / 10;
		buffer[0] = (exp % 10) + '0';
		buffer += 3;
	}else if (fmt == 'f'){
		fdigits = fcvtbuf(value, precision, &decpt, &sign, cvtbuf);
		if (sign) *buffer++ = '-';
		if (*fdigits){
			if (decpt <= 0){
				*buffer++ = '0';
				*buffer++ = '.';
				for (pos = 0; pos < -decpt; pos++) *buffer++ = '0';
				while (*fdigits) *buffer++ = *fdigits++;
			}else{
				pos = 0;
				while (*fdigits){
					if (pos++ == decpt) *buffer++ = '.';
					*buffer++ = *fdigits++;
				}
			}
		}else{
			*buffer++ = '0';
			if (precision > 0){
				*buffer++ = '.';
				for (pos = 0; pos < precision; pos++) *buffer++ = '0';
			}
		}
	}
	*buffer = '\0';
}

static void decimal_point(char *buffer){
	while (*buffer){
		if (*buffer == '.') return;
		if (*buffer == 'e' || *buffer == 'E') break;
		buffer++;
	}

	if (*buffer){
		int n = ee_strnlen(buffer,256);
		while (n > 0){
			buffer[n + 1] = buffer[n];
			n--;
		}
		*buffer = '.';
	}else{
		*buffer++ = '.';
		*buffer = '\0';
	}
}

static void cropzeros(char *buffer){
	char *stop;
	while (*buffer && *buffer != '.') buffer++;
	if (*buffer++){
		while (*buffer && *buffer != 'e' && *buffer != 'E') buffer++;
		stop = buffer--;
		while (*buffer == '0') buffer--;
		if (*buffer == '.') buffer--;
		while (buffer!=stop)
		*++buffer=0;
	}
}

static char *flt(char *str, double num, int size, int precision, char fmt, int flags)
{
	char tmp[80];
	char c, sign;
	int n, i;

	// Left align means no zero padding
	if (flags & LEFT) flags &= ~ZEROPAD;

	// Determine padding and sign char
	c = (flags & ZEROPAD) ? '0' : ' ';
	sign = 0;
	if (flags & SIGN){
		if (num < 0.0){
		  sign = '-';
		  num = -num;
		  size--;
		}else if (flags & PLUS){
			sign = '+';
			size--;
		}else if (flags & SPACE){
			sign = ' ';
			size--;
		}
	}

	// Compute the precision value
	if (precision < 0)
	precision = 6; // Default precision: 6

	// Convert floating point number to text
	parse_float(num, tmp, fmt, precision);

	if ((flags & HEX_PREP) && precision == 0) decimal_point(tmp);
	if (fmt == 'g' && !(flags & HEX_PREP)) cropzeros(tmp);

	n = ee_strnlen(tmp,256);

	// Output number with alignment and padding
	size -= n;
	if (!(flags & (ZEROPAD | LEFT))) while (size-- > 0) *str++ = ' ';
	if (sign) *str++ = sign;
	if (!(flags & LEFT)) while (size-- > 0) *str++ = c;
	for (i = 0; i < n; i++) *str++ = tmp[i];
	while (size-- > 0) *str++ = ' ';

	return str;
}

#endif

static int ee_vsprintf(char *buf, const char *fmt, va_list args)
{
	int len;
	unsigned long num;
	int i, base;
	char *str;
	char *s;

	int flags;            // Flags to number()

	int field_width;      // Width of output field
	int precision;        // Min. # of digits for integers; max number of chars for from string
	int qualifier;        // 'h', 'l', or 'L' for integer fields

	for (str = buf; *fmt; fmt++){
		if (*fmt != '%'){
		  *str++ = *fmt;
		  continue;
		}
                  
		// Process flags
		flags = 0;
	repeat:
		fmt++; // This also skips first '%'
		switch (*fmt){
			case '-': flags |= LEFT; goto repeat;
			case '+': flags |= PLUS; goto repeat;
			case ' ': flags |= SPACE; goto repeat;
			case '#': flags |= HEX_PREP; goto repeat;
			case '0': flags |= ZEROPAD; goto repeat;
		}
          
		// Get field width
		field_width = -1;
		if (is_digit(*fmt))
			field_width = ee_skip_atoi(&fmt);
		else if (*fmt == '*'){
			fmt++;
			field_width = va_arg(args, int);
			if (field_width < 0){
				field_width = -field_width;
				flags |= LEFT;
			}
		}

		// Get the precision
		precision = -1;
		if (*fmt == '.'){
			++fmt;    
			if (is_digit(*fmt))
				precision = ee_skip_atoi(&fmt);
			else if (*fmt == '*'){
				++fmt;
				precision = va_arg(args, int);
			}
			if (precision < 0) precision = 0;
		}

		// Get the conversion qualifier
		qualifier = -1;
		if (*fmt == 'l' || *fmt == 'L'){
			qualifier = *fmt;
			fmt++;
		}

		// Default base
		base = 10;

		switch (*fmt){
			case 'c':
				if (!(flags & LEFT)) while (--field_width > 0) *str++ = ' ';
				*str++ = (unsigned char) va_arg(args, int);
				while (--field_width > 0) *str++ = ' ';
			continue;

			case 's':
				s = va_arg(args, char *);
				if (!s) s = "<NULL>";
					len = ee_strnlen(s, precision);
				if (!(flags & LEFT)) while (len < field_width--) *str++ = ' ';
				for (i = 0; i < len; ++i) *str++ = *s++;
				while (len < field_width--) *str++ = ' ';
			continue;

			case 'p':
			if (field_width == -1){
				field_width = 2 * sizeof(void *);
				flags |= ZEROPAD;
			}
			str = ee_number(str, (unsigned long) va_arg(args, void *), 16, field_width, precision, flags);
			continue;

			case 'A':
			flags |= UPPERCASE;

			case 'a':
			if (qualifier == 'l')
				str = eaddr(str, va_arg(args, unsigned char *), field_width, precision, flags);
			else
				str = iaddr(str, va_arg(args, unsigned char *), field_width, precision, flags);
			continue;

			// Integer number formats - set up the flags and "break"
			case 'o':
				base = 8;
			break;

			case 'X':
				flags |= UPPERCASE;

			case 'x':
				base = 16;
			break;

			case 'd':
			case 'i':
				flags |= SIGN;

			case 'u':
			break;

			#ifdef HAS_FLOAT

			case 'f':
				str = flt(str, va_arg(args, double), field_width, precision, *fmt, flags | SIGN);
			continue;

			#endif

			default:
			if (*fmt != '%') *str++ = '%';		
			if (*fmt)
				*str++ = *fmt;
			else
				--fmt;
			continue;
		}

		if (qualifier == 'l')
			num = va_arg(args, unsigned long);
		else if (flags & SIGN)
			num = va_arg(args, int);
		else
			num = va_arg(args, unsigned int);

		str = ee_number(str, num, base, field_width, precision, flags);
		}

	*str = '\0';
	return str - buf;
}

void uart_send_char(char c) {
	char str[2];
	str[0] = c;
	str[1] = '\0';
	putsnonl(str);
}

int sprintf(char *buf, const char *fmt, ...) {	
	va_list args;
	int n;

	va_start(args, fmt);
	n = vsprintf(buf, fmt, args);
	va_end(args);
	
	return n;
}

int printf(const char *fmt, ...){
	char buf[15*80],*p;
	va_list args;
	int n=0;

	va_start(args, fmt);
	ee_vsprintf(buf, fmt, args);
	va_end(args);
	p=buf;
	while (*p) {
		uart_send_char(*p);
		n++;
		p++;
	}

	return n;
}
