

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include "input_output.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_gpio.h"
#include "debug_frmwrk.h"

static unsigned char switch_prev_state[MAX_INPUT_COUNT];
static unsigned int switch_state_count[MAX_INPUT_COUNT];    //number of 
unsigned char input_name[MAX_INPUT_COUNT];
unsigned int switch_state_max_rise_count[MAX_INPUT_COUNT];
unsigned int switch_state_max_fall_count[MAX_INPUT_COUNT];
unsigned char input_rise_flag[MAX_INPUT_COUNT];
unsigned char input_fall_flag[MAX_INPUT_COUNT];

unsigned char input_value[MAX_INPUT_COUNT];
unsigned char input_state[MAX_INPUT_COUNT];

GPIOPin io_input_pins[MAX_INPUT_COUNT] = {  
#if MAX_INPUT_COUNT > 0
IO_INPUT0 
#endif
#if MAX_INPUT_COUNT > 1
,IO_INPUT1 
#endif
#if MAX_INPUT_COUNT >2
,IO_INPUT2 
#endif
#if MAX_INPUT_COUNT >3
,IO_INPUT3 
#endif
#if MAX_INPUT_COUNT >4
,IO_INPUT4 
#endif
#if MAX_INPUT_COUNT >5
,IO_INPUT5 
#endif
#if MAX_INPUT_COUNT >6
,IO_INPUT6 
#endif
#if MAX_INPUT_COUNT >7
,IO_INPUT7 
#endif
#if MAX_INPUT_COUNT >8
,IO_INPUT8 
#endif
#if MAX_INPUT_COUNT >9
,IO_INPUT9 
#endif
#if MAX_INPUT_COUNT >10
,IO_INPUT10
#endif
#if MAX_INPUT_COUNT >11
,IO_INPUT11 
#endif
#if MAX_INPUT_COUNT >12
,IO_INPUT12 
#endif
#if MAX_INPUT_COUNT >13
,IO_INPUT13
#endif
// #if MAX_INPUT_COUNT >14
// ,IO_INPUT14
// #endif
// #if MAX_INPUT_COUNT >15
// ,IO_INPUT15
// #endif
};

GPIOPin io_output_pins[MAX_OUTPUT_COUNT] = {
#if MAX_OUTPUT_COUNT > 0
IO_OUTPUT0
#endif
#if MAX_OUTPUT_COUNT > 1
,IO_OUTPUT1
#endif
#if MAX_OUTPUT_COUNT > 2
,IO_OUTPUT2
#endif
#if MAX_OUTPUT_COUNT > 3
,IO_OUTPUT3
#endif
#if MAX_OUTPUT_COUNT > 4
,IO_OUTPUT4
#endif
#if MAX_OUTPUT_COUNT > 5
,IO_OUTPUT5
#endif
#if MAX_OUTPUT_COUNT > 6
,IO_OUTPUT6
#endif
#if MAX_OUTPUT_COUNT > 7
,IO_OUTPUT7
#endif
#if MAX_OUTPUT_COUNT > 8
,IO_OUTPUT8
#endif
#if MAX_OUTPUT_COUNT > 9
,IO_OUTPUT9
#endif
#if MAX_OUTPUT_COUNT > 10
,IO_OUTPUT10
#endif
#if MAX_OUTPUT_COUNT > 11
,IO_OUTPUT11
#endif
#if MAX_OUTPUT_COUNT > 12
,IO_OUTPUT12
#endif
#if MAX_OUTPUT_COUNT > 13
,IO_OUTPUT13
#endif
#if MAX_OUTPUT_COUNT > 14
,IO_OUTPUT14
#endif
#if MAX_OUTPUT_COUNT > 15
,IO_OUTPUT15
#endif
};

void io_update(void)
{
	static int i;
	static unsigned char input_val[MAX_INPUT_COUNT];   //refering to the value read by each sensor	
	for(i=0;i<MAX_INPUT_COUNT;i++){
		input_val[i] = io_read_input(i);
		
		if (input_val[i] != switch_prev_state[i]){
			switch_state_count[i] ++;  // switch_state_count[i] is incremented by one   
		}
		else{								 // if prev state is same, set state count to 0
			switch_state_count[i] = 0;       //switch state count reset to zero
		}
 	}
	
	for(i=0;i<MAX_INPUT_COUNT;i++){
 		if (switch_prev_state[i] == INPUT_ON){	// this is for rising edge...
			if (switch_state_count[i] > switch_state_max_rise_count[i]){  //if false no further action keep checking for other sensors and stop
				input_state[i] = switch_prev_state[i] = input_val[i];      //if yes update previous_switch_state
				// set the rise flag...
				input_rise_flag[i] = 1;
			}	
		}else{ // prev_switch_state = ON // this is for falling edge...
			if (switch_state_count[i] > switch_state_max_fall_count[i]){  //if flase no further action keep checking for other sensors and stop
				input_state[i] = switch_prev_state[i] = input_val[i];      //if yes update previous_switch_state
				input_fall_flag[i] = 1;	
			}	
		}
	}	
}


void io_init(void)
{
	int i;
	io_configure(io_input_pins,MAX_INPUT_COUNT, 0);
	for(i=0;i<MAX_INPUT_COUNT;i++){
		switch_state_count[i] = 0;
		switch_state_max_rise_count[i] = input_rise_debounce_time;//debounce check for rise
		switch_state_max_fall_count[i] = input_fall_debounce_time;//debounce check for fall
		input_rise_flag[i] = input_fall_flag[i] = input_state[i] = 0;
		input_state[i] = io_read_input(i);
		switch_prev_state[i] = input_state[i];

		// if(i == DRIVE_ERROR){ //the drive on being reset blinks the error pin once which is registered as an error in while(1)
		// 	switch_state_max_rise_count[i] = DRIVE_INPUT_RISE_DEBOUNCE;//debounce check for rise
		// 	switch_state_max_fall_count[i] = DRIVE_INPUT_FALL_DEBOUNCE;//debounce check for fall
		// }

	}

	io_configure(io_output_pins,MAX_OUTPUT_COUNT, 1);
	printf("io_init=%d\n",TRUE);
}

	
char io_read_input ( int i)
{
	return GPIO_Read(&io_input_pins[i]);
}


char io_configure(GPIOPin *list, unsigned int size, char in_out)
{
	unsigned int i;
	for(i=0;i<size;i++){
		// if(in_out == 1){
		// 	io_set_output(&list[i],1);

		// }
		GPIO_Configure(&list[i],in_out);
	}
	return 0;
}

void io_set_output(int i, char value)
{
	if (value){	// set the output
		GPIO_Set(&io_output_pins[i]);
	}
	else { // clear the output
		GPIO_Clear(&io_output_pins[i]);
	}
}

char GPIO_Read(GPIOPin *pin){
	uint32_t port_value, mask;
	port_value = GPIO_ReadValue(pin->port_num);
	mask = (port_value >> pin->pin_num) & 0x01;
	return mask;
}

int GPIO_Configure(GPIOPin *pin, char io){
	uint32_t pinMask = 1;
	if (pin->pin_num >= 32) {
		return 0;
	}
	pinMask = 1 << (pin->pin_num);
	if (io == 1){ //if output set to high before changing direction to out
		GPIO_Set(pin);
	}
	GPIO_SetDir(pin->port_num,pinMask,io);
	return 1;
}

void GPIO_Set(GPIOPin *pin){
	uint32_t pinMask = 0;
	pinMask = 1 << (pin->pin_num);
	GPIO_SetValue(pin->port_num,pinMask);
}

void GPIO_Clear(GPIOPin *pin){
	uint32_t pinMask = 0;
	pinMask = 1 << (pin->pin_num);
	GPIO_ClearValue(pin->port_num,pinMask);
}

