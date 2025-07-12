#include "delay_timer.h"
// #include "timer.h"

uint32_t delay_value[MAX_NUM_DELAY];
uint32_t delay_const[MAX_NUM_DELAY];
static char delay_enable[MAX_NUM_DELAY];

uint32_t accum_update_a;
uint32_t accum_update_b;

//-----------PWM DESCRIPTION-----------------------------
//PWMPR:prescalar  
//default timer working at 15Mhz
//after prescaling: 15Mhz/1000 =15 khz.
//		Timeperiod: 66.66usec
//		 max count: (2^32) * 66.66usec
//--------------------------------------------------------

void dt_init(void){
	delay_timer_32 = 0;
}

// extern "C" void timer_update (void){
	// delay_timer_32++;
	// TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
// }

/*
sec mul is the number of divisions of a second allowed...
i.e., for secmul = 15000 and VPB_DIV = 4 and CCLK = 60MHz, PWMPR = 999 


*/
void dt_init(uint32_t sec_mul){
	delay_timer_32 = 0;
	for (int i=0;i<MAX_NUM_DELAY;i++){
		delay_value[i] = 0;
		delay_enable[i] = 0;
	}
	LPC_SC -> PCONP |= 1 << 22; // Power up Timer 2	
    LPC_SC -> PCLKSEL1 |= 1 << 12; // Clock for timer = CCLK
	LPC_TIM2 -> PR = ((SystemCoreClock / 1000000) * 0.5) - 1; // prescalar value for 500us	
	LPC_TIM2 -> MR0 = sec_mul;
	LPC_TIM2 -> MCR |= (1 << 0) | (1 << 1); // all values zero for continuous operation
	LPC_TIM2 -> TCR |= 1 << 1; // Manually Reset Timer0 ( forced )
	LPC_TIM2 -> TCR &= ~(1 << 1); // stop resetting the timer.
	NVIC_EnableIRQ((IRQn_Type)TIMER2_IRQn);	
	NVIC_SetPriority((IRQn_Type)TIMER2_IRQn, 3);
	LPC_TIM2 -> TCR |= 1 << 0; // Start timer
	printf("Delay Timer \n");
	// init_timer2((void *)TIMER2_IRQn);
}

static uint32_t value_check(){
	do{
		accum_update_a = delay_timer_32;
		accum_update_b = delay_timer_32;
	}while(accum_update_a != accum_update_b);
	return accum_update_a;
}

static int is_timeup(uint32_t start_time, uint32_t time_delay){

	uint32_t l_delay_timer_32 = value_check();	
	int32_t time_diff = l_delay_timer_32 - start_time;

	if (0 > time_diff){
	// add 0xffffffff for -ve values for the 32 bit timer...
		time_diff += 0xFFFFFFFF;
	}
	// printf("\ndelay_timer32: %d",delay_timer_32);
	
	// printf("\nstart_time: %d",start_time);
	
	// printf("\ntime_diff: %d",time_diff);
	
	if ((uint32_t)time_diff > time_delay)
		return 1;
	else
		return 0;
}

static uint32_t time_left(uint32_t start_time, uint32_t time_delay){
	
	uint32_t l_delay_timer_32 = value_check();
	int32_t time_diff = l_delay_timer_32 - start_time;
	
	if (time_diff < 0){
		// add 0xffffffff for -ve values for the 32 bit timer...
		time_diff += 0xFFFFFFFF;
	}
	
	// printf("\ndelay_timer32: %ld",delay_timer_32);
	
	// printf("\nstart_time: %ld",start_time);
	
	// printf("\ntime_diff: %ld",time_diff);
	
	if ((uint32_t)time_diff > time_delay){
		return 0;
	} else{
		return (time_delay - time_diff);
	}
}

void my_delay(long d)
{
	long t_val;
	uint32_t l_delay_timer_32 = value_check();
	
	t_val = l_delay_timer_32;
		while (1){
			if (!is_timeup(t_val, d));
			//	update_32bit_counter_value();
			//	delay_timer_32 = PWMTC;
			else
				break;
	}
}

int dt_is_timeup(int i)
{
	// printf("\ndt_is_timeup");
	return is_timeup(delay_value[i], delay_const[i]);

}

// this function is not working... not sure why...
int dt_timeup_reset(int i)
{
	int retVal;
	retVal=is_timeup(delay_value[i], delay_const[i]);
	dt_reset(i);
	return retVal;
}

int dt_reset(int i)
{
	uint32_t l_delay_timer_32 = value_check();
	
	delay_value[i] = l_delay_timer_32;
	return 1;
}

uint32_t dt_time_left(int i)
{
	return time_left(delay_value[i],delay_const[i]);
}

void dt_set_delay(int i, uint32_t d)
{
	delay_const[i] = d;
}

uint32_t dt_get_delay(int i)
{
	return delay_const[i];
}

DT_HMS dt_get_hms(uint32_t time_left, uint32_t sec_mul)
{
	// char str [35];
	DT_HMS l_hms;
	int32_t total_secs_left ,temp;
	// total_msec_left is the total milisecs in time_left

	// print_string("\nsec_mul ");
	// print_long(SEC_MUL);
	// print_string("\ntime_left ");
	// print_long(time_left);
	// time_left = time_left / sec_mul;//thid gives the time in milliseconds
	total_secs_left = time_left / sec_mul;//this gives the time in seconds
	uint64_t t64 = (uint64_t) time_left * 1000;
	t64 /= sec_mul;
	uint32_t total_msec_left = (uint32_t) t64;
	
	// print_string("\ntotal_msec_left");
	// print_long(total_msec_left);
	
	
	l_hms.hrs =  ( total_secs_left / 3600 );//time left in hours
	total_secs_left %= 3600;//time left after conversion into hours
	l_hms.min = total_secs_left / 60;
	// total_msec_left = (total_msec_left % 3600);
	total_secs_left = total_secs_left % 60;
	l_hms.sec = total_secs_left;
	
	total_secs_left = ((l_hms.hrs*60)+l_hms.min)*60 + l_hms.sec;
	
	
	
	// print_string("\ntotal secs ");
	// print_long(total_msecs_left);
	
	temp = total_msec_left - total_secs_left*1000;
	// print_string("\ntemp");
	// print_long(temp);
	if (temp > 0) {
		l_hms.msec = (int)temp;
		// print_string("\n\nmsec ");
		// print_long(l_hms.msec);
	} else {
		l_hms.msec = 0;
		// print_string("\n\nmsec ");
		// print_long(l_hms.msec);
		// print_string("\n\nThere is a error in DTgethms");
	}
	
	return l_hms;
	
}

uint32_t dt_convert_hms(DT_HMS hms, uint32_t sec_mul)
{
	uint32_t cnt_time;
	
	cnt_time = (hms.hrs *3600 + hms.min*60 + hms.sec) * sec_mul;
	
	uint32_t msec_time = (hms.msec *sec_mul);
	msec_time /= 1000;
	cnt_time += msec_time;
	return cnt_time;
}

int dt_start(int i){
	
	uint32_t l_delay_timer_32 = value_check();
	delay_value[i] = l_delay_timer_32;
	delay_enable[i] = 1;
	
	return 1;
}

int dt_stop(int i){
	delay_enable[i] = 0;
	
	return 1;
}	
	
int dt_is_timeup_with_enable(int i){
	if (delay_enable[i] == 1){
		return is_timeup(delay_value[i], delay_const[i]);
	}
	else{
		return 0;
	}
}	

