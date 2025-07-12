/******************************************************************************
 *
 * $RCSfile: $
 * $Revision: $
 *
 * This module defines some regularly used typedefs
 *
 *****************************************************************************/

#ifndef INC_TYPES_H
#define INC_TYPES_H

/*  typedefs are here  */
typedef unsigned char       uint8_t;
typedef   signed char        int8_t;
typedef unsigned short     uint16_t;
typedef   signed short      int16_t;
typedef unsigned long      uint32_t;
typedef   signed long       int32_t;
typedef unsigned long long uint64_t;
typedef   signed long long  int64_t;

typedef unsigned short int	U16;	/**< unsigned 16-bit */
typedef unsigned int		U32;	/**< unsigned 32-bit */

typedef int					BOOL;	/**< #TRUE or #FALSE */

typedef enum {False, True}  boolean;

typedef float SINGLE;
typedef long double DOUBLE;

/* This struct is for return value from keypad...*/
typedef struct {
	unsigned char code;
	unsigned char updown;
	unsigned int count;
} key_code_count;

#define MIN(x,y)	((x)<(y)?(x):(y))	/**< MIN */
#define MAX(x,y)	((x)>(y)?(x):(y))	/**< MAX */

#define Key_Code_Count	key_code_count

#endif
