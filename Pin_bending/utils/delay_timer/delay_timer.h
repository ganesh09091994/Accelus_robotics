#ifndef DELAY_TIMER_h
#define DELAY_TIMER_h
#include "LPC17xx.h"		/* LPC17XX Peripheral Registers	*/
#include "lpc_types.h"
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#include "timer.h"

extern "C" {
	#include "lpc17xx_pinsel.h"
	#include "lpc17xx_libcfg.h"
	#include "lpc17xx_nvic.h"
}

#define MAX_NUM_DELAY DELAY_TOTAL_NUM

// #define timer_update	TIMER2_IRQHandler

void dt_init(void);
void dt_init(uint32_t sec_mul);
void my_delay(long d);
int dt_is_timeup(int i);
int dt_timeup_reset(int i);
int dt_reset(int i);
uint32_t dt_time_left(int i);
void dt_set_delay(int i, uint32_t d);
uint32_t dt_get_delay(int i);
// #define dt_start(i)	dt_reset(i)

int dt_is_timeup_with_enable(int i);
int dt_start(int i);
int dt_stop(int i);

typedef struct { 
	int hrs; 
	int min; 
	int sec;
	int msec;
} DT_HMS;

extern "C"{ 
extern uint32_t delay_timer_32;
}

/* sec_mul is the seconds multiplier..*/
DT_HMS dt_get_hms(uint32_t time_left, uint32_t sec_mul);
uint32_t dt_convert_hms(DT_HMS hms, uint32_t sec_mul);

#endif
