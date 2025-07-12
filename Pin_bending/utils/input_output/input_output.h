#ifndef _SENSOR_STATE_UPDATE_H_
#define _SENSOR_STATE_UPDATE_H_

#ifdef __cplusplus
// extern "C"{
#endif

//#include "typedefs.h"
#include <stdint.h>
// #include "fio.h"
#include "board.h"
// #include "global.h"
#include "config.h"

#define MAX_INPUT_COUNT 	INPUT_NUM
#define MAX_OUTPUT_COUNT	OUTPUT_NUM

/* configure the inputs used...*/
#define IO_INPUT0 			INPUT0
#define IO_INPUT1 			INPUT1
#define IO_INPUT2 			INPUT2
#define IO_INPUT3 			INPUT3
#define IO_INPUT4 			INPUT4
#define IO_INPUT5 			INPUT5
#define IO_INPUT6 			INPUT6
#define IO_INPUT7 			INPUT7
#define IO_INPUT8 			INPUT8
#define IO_INPUT9 			INPUT9
#define IO_INPUT10 			INPUT10
#define IO_INPUT11 			INPUT11
#define IO_INPUT12 			INPUT12
#define IO_INPUT13 			INPUT13

/* configure the outputs used...*/
#define IO_OUTPUT0 			OUTPUT0
#define IO_OUTPUT1 			OUTPUT1
#define IO_OUTPUT2 			OUTPUT2
#define IO_OUTPUT3 			OUTPUT3
#define IO_OUTPUT4 			OUTPUT4
#define IO_OUTPUT5 			OUTPUT5
#define IO_OUTPUT6 			OUTPUT6
#define IO_OUTPUT7 			OUTPUT7
#define IO_OUTPUT8 			OUTPUT8
#define IO_OUTPUT9 			OUTPUT9
#define IO_OUTPUT10			OUTPUT10
#define IO_OUTPUT11 		OUTPUT11
#define IO_OUTPUT12 		OUTPUT12
#define IO_OUTPUT13 		OUTPUT13
#define IO_OUTPUT14 		OUTPUT14
#define IO_OUTPUT15 		OUTPUT15

// Input Group1 
#define INPUT_0_RISE_FLAG	input_rise_flag[0]
#define INPUT_1_RISE_FLAG	input_rise_flag[1]
#define INPUT_2_RISE_FLAG	input_rise_flag[2]
#define INPUT_3_RISE_FLAG	input_rise_flag[3]
#define INPUT_4_RISE_FLAG	input_rise_flag[4]
#define INPUT_5_RISE_FLAG	input_rise_flag[5]
#define INPUT_6_RISE_FLAG	input_rise_flag[6]
#define INPUT_7_RISE_FLAG	input_rise_flag[7]
#define INPUT_8_RISE_FLAG	input_rise_flag[8]
#define INPUT_9_RISE_FLAG	input_rise_flag[9]
#define INPUT_10_RISE_FLAG	input_rise_flag[10]
#define INPUT_11_RISE_FLAG	input_rise_flag[11]
#define INPUT_12_RISE_FLAG	input_rise_flag[12]
#define INPUT_13_RISE_FLAG	input_rise_flag[13]

#define INPUT_0_FALL_FLAG	input_fall_flag[0]
#define INPUT_1_FALL_FLAG	input_fall_flag[1]
#define INPUT_2_FALL_FLAG	input_fall_flag[2]
#define INPUT_3_FALL_FLAG	input_fall_flag[3]
#define INPUT_4_FALL_FLAG	input_fall_flag[4]
#define INPUT_5_FALL_FLAG	input_fall_flag[5]
#define INPUT_6_FALL_FLAG	input_fall_flag[6]
#define INPUT_7_FALL_FLAG	input_fall_flag[7]
#define INPUT_8_FALL_FLAG	input_fall_flag[8]
#define INPUT_9_FALL_FLAG	input_fall_flag[9]
#define INPUT_10_FALL_FLAG	input_fall_flag[10]
#define INPUT_11_FALL_FLAG	input_fall_flag[11]
#define INPUT_12_FALL_FLAG	input_fall_flag[12]
#define INPUT_13_FALL_FLAG	input_fall_flag[13]


#define INPUT_LEVEL_0		input_state[0]
#define INPUT_LEVEL_1		input_state[1]
#define INPUT_LEVEL_2		input_state[2]
#define INPUT_LEVEL_3		input_state[3]
#define INPUT_LEVEL_4		input_state[4]
#define INPUT_LEVEL_5		input_state[5]
#define INPUT_LEVEL_6		input_state[6]
#define INPUT_LEVEL_7		input_state[7]
#define INPUT_LEVEL_8		input_state[8]
#define INPUT_LEVEL_9		input_state[9]
#define INPUT_LEVEL_10		input_state[10]
#define INPUT_LEVEL_11		input_state[11]
#define INPUT_LEVEL_12		input_state[12]
#define INPUT_LEVEL_13		input_state[13]

//#define INPUT_PORT_NUM 0
//#define INPUT_BIT_START 8
#define INPUT_ON 1
#define INPUT_OFF 0

// #define INPUT_RISE_DEBOUNCE_TIME 	2
// #define INPUT_FALL_DEBOUNCE_TIME	2

#define input_rise_debounce_time 	2
#define input_fall_debounce_time	2

//extern static unsigned char switch_prev_state[MAX_INPUT_COUNT];
//extern static unsigned int switch_state_count[MAX_INPUT_COUNT];
extern unsigned char input_name[MAX_INPUT_COUNT];
extern unsigned char input_rise_flag[MAX_INPUT_COUNT];
extern unsigned char input_fall_flag[MAX_INPUT_COUNT];
//extern unsigned char input_value[MAX_INPUT_COUNT];
extern unsigned char input_state[MAX_INPUT_COUNT];

// extern int input_rise_debounce_time;
// extern int input_fall_debounce_time;

extern GPIOPin io_input_pins[];
extern GPIOPin io_output_pins[];

void io_init(void);
void io_update(void);
char io_read_input( int i);
char io_configure(GPIOPin *list, unsigned int size, char in_out);
void io_set_output(int i, char value);

int GPIO_Configure(GPIOPin *pin, char io);
char GPIO_Read(GPIOPin *pin);
void GPIO_Set(GPIOPin *pin);
void GPIO_Clear(GPIOPin *pin);
//void test_switches(void);
//void retrigger_all_sensors(void);
// #ifdef __cplusplus
// }
// #endif
#endif
