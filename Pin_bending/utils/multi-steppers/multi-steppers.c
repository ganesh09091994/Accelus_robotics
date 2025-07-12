#include <stdlib.h>
#include "multi-steppers.h"
#include "timer.h"
#include "input_output.h"
#include "uart.h"
//variables
int32_t periodns;// = 1e9/MAKE_PULSES_FREQ;  // time in ns for freq of execution of makepulses
float periodfp;// = 1/MAKE_PULSES_FREQ;		// time in s
float freqscale;// = (1L <<PICKOFF) *periodfp;
float accelscale;// = freqscale * periodfp;
int32_t dtns;// = 1e9/FREQ_UPDATE_FREQ;								// period of update_freq in ns
float dt;// = 1/UPDATE_FREQ_FREQ;									// update freq period in secs.
float recip_dt;// = UPDATE_FREQ_FREQ;								// reciprocal of dt period

stepper_struct g_stepgen[NUM_AXES];
profiler_t Profiler[NUM_AXES];

static uint32_t ulceil(uint32_t value, uint32_t increment);

int saveDataOnBrownOut(void);

int trig_update_freq = 0;

// int modbus_data_complete = 0;
// char ring_slaveID = 0;


int update_next_motor_position_every_1ms();
int update_sm_operation_every_1ms();

#ifdef MULTI_STEPPERS_DIFFERENT_PORTS
int PULSE_PORT_NUM[4] = {
						PULSE_PORT1,
						PULSE_PORT2,
						PULSE_PORT3,
						PULSE_PORT4,					
					};

volatile uint32_t *PULSE_PORT_SET_NUM[4] = {
							&PULSE_PORT_SET1,
							&PULSE_PORT_SET2,
							&PULSE_PORT_SET3,
							&PULSE_PORT_SET4,
							};
							
volatile uint32_t *PULSE_PORT_CLEAR_NUM[4]={
							&PULSE_PORT_CLEAR1,	
							&PULSE_PORT_CLEAR2,	
							&PULSE_PORT_CLEAR3,	
							&PULSE_PORT_CLEAR4,	
							};

volatile uint32_t *PULSE_PORT_DIR_NUM[4]={
							&PULSE_PORT_DIR1,
							&PULSE_PORT_DIR2,
							&PULSE_PORT_DIR3,
							&PULSE_PORT_DIR4,
							};

int PULSE_PIN_OFFSET_NUM[4]={	
							PULSE_PIN_OFFSET1,
							PULSE_PIN_OFFSET2,
							PULSE_PIN_OFFSET3,
							PULSE_PIN_OFFSET4,
							};							
					
int DIR_PORT_NUM[4] = {
					DIR_PORT1,
					DIR_PORT2,
					DIR_PORT3,
					DIR_PORT4,
				};

volatile uint32_t *DIR_PORT_SET_NUM[4]={
						&DIR_PORT_SET1,
						&DIR_PORT_SET2,
						&DIR_PORT_SET3,
						&DIR_PORT_SET4,
						};

volatile uint32_t *DIR_PORT_CLEAR_NUM[4]={
							&DIR_PORT_CLEAR1,
							&DIR_PORT_CLEAR2,
							&DIR_PORT_CLEAR3,
							&DIR_PORT_CLEAR4,
							};

volatile uint32_t *DIR_PORT_DIR_NUM[4]={
						&DIR_PORT_DIR1,
						&DIR_PORT_DIR2,
						&DIR_PORT_DIR3,
						&DIR_PORT_DIR4,
						};

int DIR_PIN_OFFSET_NUM[4]={
						DIR_PIN_OFFSET1,
						DIR_PIN_OFFSET2,
						DIR_PIN_OFFSET3,
						DIR_PIN_OFFSET4,
						};
#else
static uint32_t CLEAR_ALL_MOTOR_PULSE_OUTPUTS =0;
static uint32_t fio_pulse_reg_set;
static uint32_t fio_pulse_reg_clr;
static uint32_t fio_dir_reg_set;
static uint32_t fio_dir_reg_clr;	

static uint32_t make_pulses_GPIO_Set (uint32_t *reg, GPIOpin_t *p);
static uint32_t make_pulses_GPIO_Clear (uint32_t *reg, GPIOpin_t *p);					
#endif

void Delay_Timer_ISR(void){	
	// update_next_motor_position_every_1ms();
	io_update(); 
	update_sm_operation_every_1ms();
	// io_set_output(4,1);	
	delay_timer_32++;	
	io_update(); //update inputs
	#if NUM_AXES > 0
		stepper_move_motor(MOTOR0);
	#if NUM_AXES > 1
		stepper_move_motor(MOTOR1);
	#if NUM_AXES > 2
		stepper_move_motor(MOTOR2);
	#if NUM_AXES > 3
		stepper_move_motor(MOTOR3);
	#endif
	#endif
	#endif
	#endif
	stepper_update_freq();		
	TIM_ClearIntPending(LPC_TIM3, TIM_MR0_INT);
	// io_set_output(4,0);	
}

void Make_Pulses_ISR(void){	
	// io_set_output(9,1);
	stepper_make_pulses();

	MULTI_STEPPER_PULSE_CLEAR_TIMER;

	// MULTI_STEPPER_PULSE_CLEAR_TIMER;

	TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);	
	// io_set_output(9,0);
}



void Pulse_Clear_ISR(void){
	// io_set_output(7,1);	
#ifdef MULTI_STEPPERS_DIFFERENT_PORTS	
	#if NUM_AXES > 0		
		PULSE_PORT_CLEAR1 = PULSE_PIN_SHIFT1;
	#if NUM_AXES > 1		
		PULSE_PORT_CLEAR2 = PULSE_PIN_SHIFT2;
	#if NUM_AXES > 2		
		PULSE_PORT_CLEAR3 = PULSE_PIN_SHIFT3;
	#if NUM_AXES > 3		
		PULSE_PORT_CLEAR4 = PULSE_PIN_SHIFT4;
	#endif
	#endif
	#endif
	#endif
	TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
#else
	PULSE_PORT_CLEAR = CLEAR_ALL_MOTOR_PULSE_OUTPUTS;
#endif

	// io_set_output(7,0);	
}

void EINT2_IRQHandler(void){ // brownOut
	saveDataOnBrownOut();
	LPC_SC->EXTINT = EINT2_CLEAR;
}

void Modbus_Timeout_ISR(void){
// printf("\n mdms tmr");
#ifdef MOD_UART1	
	// printf("\n uart = %d",Uart_ChkRcv1());
	// printf("\n data_comp = %d",(&modbus1)->modbus_data_complete);
	if( (Uart_ChkRcv1() == 0) && ((&modbus1)->modbus_data_complete == 0) ) { // some data available on Uart and last frame analysis complete
		(&modbus1)->modbusTimerCount++;
		
		// printf("\n slave_id_hmi= %d",(&modbus1)->ring_slaveID);
		// printf("\n slave_id_local= %d",(&modbus1)->slaveID);
		if( ( ( ((&modbus1)->modbusTimerCount)*(MODBUS_TIMER_TIME+1) ) ) > ((&modbus1)->modbus_timeout) ){
			(&modbus1)->ring_slaveID = Uart_GetC1(); // store slaveId here
			if((&modbus1)->ring_slaveID != (&modbus1)->slaveID){ //not my data...  clear ring
				ring_clear((&modbus1)->ring_address);
				// printf("\n else id wrong");
			}else{ //my data
				(&modbus1)->modbus_data_complete = 1;
				// printf("\n data done");

			}
		}
	}
	
#endif

#ifdef MOD_USB
	// printf("\n data avl=%d",(&modbusUSB)->modbus_data_complete);

	if( (VCOM_ChkRcv() == 0) && ((&modbusUSB)->modbus_data_complete == 0) ) { // some data available on Uart and last frame analysis complete
		(&modbusUSB)->modbusTimerCount++;

		// printf("\n arg1=%d", ( ( ((&modbusUSB)->modbusTimerCount)*(MODBUS_TIMER_TIME+1) ) ));
		// printf("\n arg2=%d", ((&modbusUSB)->modbus_timeout));
		if( ( ( ((&modbusUSB)->modbusTimerCount)*(MODBUS_TIMER_TIME+1) ) ) > ((&modbusUSB)->modbus_timeout) ){
			(&modbusUSB)->ring_slaveID = VCOM_GetCh(); //store slaveId here

			// printf();
			if((&modbusUSB)->ring_slaveID != (&modbusUSB)->slaveID){ //not my data...  clear ring
				ring_clear((&modbusUSB)->ring_address);
				// printf("\n slv id");
			}else{ //my data
				(&modbusUSB)->modbus_data_complete = 1;

			}
		}
	}
		
#endif

	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}


#ifdef MULTI_STEPPERS_DIFFERENT_PORTS	
	#if 0
	static uint32_t make_pulses_GPIO_Set_Port (uint32_t * reg, GPIOpin_t *p){
		*reg |= (1<< p->pin_num);
	}
	static uint32_t make_pulses_GPIO_Clear_Port (uint32_t * reg, GPIOpin_t *p){
		*reg |= (1<< p->pin_num);
	}
	#endif
#else	
	static uint32_t make_pulses_GPIO_Set (uint32_t * reg, GPIOpin_t *p){
		*reg |= (1<< p->pin_num);
		return *reg;
	}

	static uint32_t make_pulses_GPIO_Clear (uint32_t* reg, GPIOpin_t *p){
		// FIO0SET |= (1<<4);
		*reg |= (1<< p->pin_num);
		return *reg;
	}
#endif

/*** This func has no float or divide operations

***/
int stepper_make_pulses(void)
{
	register int i;
	register stepper_struct *stepgen;
	// this needs to be done elsewhere... and just used here..
	//uint32_t periodns = (1e9/MAKE_PULSES_FREQ);	
	
	register int32_t old_addval, target_addval, new_addval, step_now;

	for (i = 0; i < NUM_AXES; i++){	// for each axes...
		stepgen = &g_stepgen[i];
		
		old_addval = stepgen->addval;
		target_addval = stepgen->target_addval;
		
		// assume that new value can be reached in 1 period and no rate limiting is required
		new_addval = target_addval;

		// check for direction reversal..
		if (new_addval >0){
			stepgen->curr_dir = 1;
		} else if (new_addval <0){
			stepgen->curr_dir = -1;
		}
		
/*++++++++++++++++NEW ADDIDTION IS HERE++++++++++++++++*/
		if(stepgen->prev_dir != stepgen->curr_dir){
			stepgen->prev_dir = stepgen->curr_dir;
			//stepgen->hold bit is SET to skip calculation & STEP generation while DIRECTION CHANGE
			stepgen->hold = 1;
			
			//this part is removed from step_now loop
			if (stepgen->curr_dir < 0){
				*DIR_PORT_SET_NUM[i] = 1<<stepgen->pin_dir.pin_num;
				stepgen->raw_pos -= 1;
			}else{
				*DIR_PORT_CLEAR_NUM[i] = 1<<stepgen->pin_dir.pin_num;
				stepgen->raw_pos += 1;
			}
		}
		
		if(stepgen->hold < 1){
			stepgen->addval = new_addval;
			// save lower half of accum...
			step_now = stepgen->accum;
			// update the accumulator...
			stepgen->accum += stepgen->addval;
			// test for changes (only in lower half)...
			step_now ^= stepgen->accum;
			// we only care about the pickoff bit...
			step_now &= LONG_PICKOFF;
			// sprintf(str,"\nstep_nw %x",step_now);
			// print_string(str);
			stepgen->rawcount = stepgen->accum >> PICKOFF;
		}else{
			step_now = 0;
			stepgen->hold--;
		}
		
		// donot change direction if addval = 0;
		

		if(step_now){
			*PULSE_PORT_SET_NUM[i] = 1<<stepgen->pin_pulse.pin_num;
		}
		
	}// for i

		
	return 1;
}


int stepper_update_freq(void)
{
	int i;
	int64_t accum_a, accum_b;
	float pos_cmd;
	float vel_cmd;
	float curr_pos;
	float curr_vel;
	float match_ac;
	float match_time;
	float avg_v;
	float est_out;
	float est_cmd;
	float est_err;
	float new_vel;
	float dv;
	float dp;
	
	stepper_struct *stepgen;
	for (i = 0; i< NUM_AXES; i++){
		stepgen = &g_stepgen[i];
		if (!stepgen->enable){
			stepgen->freq = 0;
			stepgen->addval = 0;
			stepgen->target_addval = 0;
			continue; // go to the next stepgen processor..
		}
		   
		/* calculate position command in counts */
		//pos_cmd = stepgen->pos_cmd * stepgen->pos_scale;
		pos_cmd = stepgen->pos_cmd;
		/* calculate velocity command in counts/sec */
		vel_cmd = (pos_cmd - stepgen->old_pos_cmd) * recip_dt;
		stepgen->old_pos_cmd = pos_cmd;
		/* 'accum' is a long long, and its remotely possible that
		   make_pulses could change it half-way through a read.
		   So we have a crude atomic read routine */
		do{
			accum_a = stepgen->accum;
			accum_b = stepgen->accum;
		}while ( accum_a != accum_b );
		
		/* convert from fixed point to float, after subtracting
		   the one-half step offset */
		curr_pos = ((accum_a-MINUS_PICKOFF) * DIVIDE_PICKOFF);
		/* get velocity in counts/sec */
		curr_vel = stepgen->freq;
		
		/* At this point we have good values for pos_cmd, curr_pos,
			   vel_cmd, curr_vel, max_freq and max_ac, all in counts,
			   counts/sec, or counts/sec^2.  Now we just have to do
			   something useful with them. */
		/* determine which way we need to ramp to match velocity */
		if (vel_cmd > curr_vel) {
			match_ac = stepgen->max_ac;
		} else {
			match_ac = -stepgen->max_ac;
		}
		
		/* determine how long the match would take */
		match_time = (vel_cmd - curr_vel) / match_ac;
		/* calc output position at the end of the match */
		avg_v = (vel_cmd + curr_vel) * 0.5;
		est_out = curr_pos + avg_v * match_time;
		/* calculate the expected command position at that time */
		est_cmd = pos_cmd + vel_cmd * (match_time - 1.5 * dt);
		/* calculate error at that time */
		est_err = est_out - est_cmd;
		if (match_time < dt) {
			/* we can match velocity in one period */
			if (fabs(est_err) < 0.0001) {
				/* after match the position error will be acceptable */
				/* so we just do the velocity match */
				new_vel = vel_cmd;
			} else {
				/* try to correct position error */
				new_vel = vel_cmd - 0.5 * est_err * recip_dt;
				/* apply accel limits */
				if (new_vel > (curr_vel + stepgen->max_ac * dt)) {
					new_vel = curr_vel + stepgen->max_ac * dt;
				} else if (new_vel < (curr_vel - stepgen->max_ac * dt)) {
					new_vel = curr_vel - stepgen->max_ac * dt;
				}
			}
		} else {
			/* calculate change in final position if we ramp in the
			   opposite direction for one period */
			dv = -2.0 * match_ac * dt;
			dp = dv * match_time;
			/* decide which way to ramp */
			if (fabs(est_err + dp * 2.0) < fabs(est_err)) {
				match_ac = -match_ac;
			}
			/* and do it */
			new_vel = curr_vel + match_ac * dt;
		}
		/* apply frequency limit */
		if (new_vel > stepgen->max_freq) {
			new_vel = stepgen->max_freq;
		} else if (new_vel < (-stepgen->max_freq)) {
			new_vel = (-stepgen->max_freq);
		}
		stepgen->freq = new_vel;
		/* calculate new addval */
		stepgen->target_addval = stepgen->freq * freqscale;		
		if (fabs(stepgen->freq) < 0.001){
			stepgen->state = 0;			
			//motor stopped
		}
		else{
			stepgen->state = 1;
			//motor started..
		}
		/* calculate new deltalim */
		// stepgen->deltalim = stepgen->max_ac * accelscale;	
	} // for each stepgen
	return 1;
}

SINGLE stepper_stop_distance(int axis){
	profiler_t *p = &Profiler[axis];
	SINGLE d_stop = p->v_next * fabsf(p->v_next) * p->ainv2; // delta stop...stop offset	
	return d_stop;
}

int32_t stepper_next_position(profiler_t *p)
{
	SINGLE a, a_hard;
	
	//ainv2 = 0.5/p->a_max;	// move outside if amax known
	p->x_final=(int32_t)(p->jointpos + p->offset);
	SINGLE d_stop = p->v_next * fabsf(p->v_next) * p->ainv2; // delta stop...stop offset
	int32_t int_d_stop = (int32_t) d_stop;
	int32_t x_err = p->x_final - p->x_curr;
	
	// Stop now...
	if (labs(p->x_curr - p->x_final) <= p->eps_x) {
	/* 	printf("\tSTOP\t\t"); */
		//print_dbg("\n\rSTOP");
		p->x_next = (float) p->x_final;
		p->v_next = 0.0;
		//mode_trace	= MP_STOP;
		p->x_curr = (int32_t) p->x_next;
		p->state = MOTOR_STOP;
		return p->x_curr;
	}
	
	// ramp-down/projected overshoot
	if (labs((p->x_final - p->x_curr) - int_d_stop) <= p->eps_x) { // prevent overshoot
		//if (approx((p->x_final-x_curr), d_stop)) {				
		//p->v_next = p->v_next - sign(p->v_next)*p->a_max*T;
		p->v_next = p->v_next - sign(p->v_next)*p->a_max;
		//p->x_next = p->x_next	+ (p->v_next*T);			// pos_diff is for typecasting
		p->x_next = p->x_next	+ (p->v_next);			// pos_diff is for typecasting
		p->x_curr = (int32_t) p->x_next;
		p->state = MOTOR_RAMPDOWN;
		return p->x_curr;

	}
	
	if (((int_d_stop - p->eps_x > x_err) && (x_err > 0))
					|| ((int_d_stop + p->eps_x < x_err) && (x_err < 0))){	// detect overshoot and take action
		a_hard = 0.5*p->v_next*p->v_next/(SINGLE)x_err;
		p->v_next = p->v_next - a_hard*T;
		p->x_next = (SINGLE) p->x_next + (p->v_next*T);			// pos_diff is for typecasting
		p->x_curr = (int32_t) p->x_next;
		p->state = MOTOR_RAMPDOWN;
	
		return p->x_curr;
	}

	// overshoot
	if (((x_err > 0) && (p->v_next + p->a_max*T < 0))
			|| ((x_err < 0) && (p->v_next - p->a_max*T > 0))){
				//mode_trace = MP_OVERSHOOT;
			//	printf("OHS \t\t");
			//	print_dbg("\n\rOHS");
		p->v_next = 0.0;
		p->x_next = (SINGLE)p->x_curr;
		p->state = MOTOR_OVERSHOOT;
		return p->x_curr;
	}
	
	// ramp-up & coasting
	a = (SINGLE)sign(x_err);
	a*= p->a_max;
	//v_next = sign(v_curr + a*T) * min(fabsf(v_curr + a*T), p->v_max);
	p->v_next = (SINGLE) sign(p->v_next + a) * min(fabsf(p->v_next + a), p->v_max);
	//x_next = x_curr + (pos_diff)(v_next*T);
	p->x_next = p->x_next + (p->v_next);
	p->x_curr = (int32_t) p->x_next;
	p->state = MOTOR_RAMPCOAST;
	return p->x_curr;
}

int stepper_init_pulse_generator_single_axis(int i){

	stepper_struct *stepgen = &g_stepgen[i];
	float desired_freq;
	
	int32_t min_step_period;

	stepgen->enable = 1;

	stepgen->pos_scale = 1.0;
	stepgen->scale_recip = DIVIDE_PICKOFF / stepgen->pos_scale;
	// make the step_len = 1 period...

	stepgen->step_len = 1;
	stepgen->step_len = ulceil(stepgen->step_len, periodns); 
	stepgen->step_space = 1;
	stepgen->step_space = ulceil(stepgen->step_space, periodns);

	stepgen->maxvel = -1.0;
	stepgen->maxaccel = -1.0;
	stepgen->freq = 0.0;
	stepgen->pos_cmd = 0.0;
	stepgen->old_pos_cmd = 0.0;
	
#ifdef MULTI_STEPPERS_DIFFERENT_PORTS
		// stepgen->myport = &PULSE_PORT_SET1;
		// stepgen->mypin = 1<<PULSE_PIN_OFFSET1;
		
		stepgen->pin_pulse.port_num = PULSE_PORT_NUM[i];
		stepgen->pin_pulse.pin_num = PULSE_PIN_OFFSET_NUM[i];
		stepgen->pin_pulse.io = 0;
		
		stepgen->pin_dir.port_num = DIR_PORT_NUM[i];
		stepgen->pin_dir.pin_num = DIR_PIN_OFFSET_NUM[i];
		stepgen->pin_dir.io = 0;
#else		
		stepgen->pin_pulse.port_num = PULSE_PORT;
		stepgen->pin_pulse.pin_num = i+PULSE_PIN_OFFSET;
		stepgen->pin_pulse.io = 0;
		
		stepgen->pin_dir.port_num = DIR_PORT;
		stepgen->pin_dir.pin_num = i+DIR_PIN_OFFSET;
		stepgen->pin_dir.io = 0;
#endif

#ifdef MULTI_STEPPERS_DIFFERENT_PORTS
		*DIR_PORT_DIR_NUM[i] |= 1<<stepgen->pin_dir.pin_num;
		*PULSE_PORT_DIR_NUM[i] |= 1<<stepgen->pin_pulse.pin_num;
#else
		DIR_PORT_DIR |= 1<<stepgen->pin_dir.pin_num;
		PULSE_PORT_DIR |= 1<<stepgen->pin_pulse.pin_num;
#endif			
	/*put all the pulse pins in a register variable*/
	/*so that they all are cleared at the same time*/
#ifdef MULTI_STEPPERS_DIFFERENT_PORTS
#else
	CLEAR_ALL_MOTOR_PULSE_OUTPUTS |= (1<<stepgen->pin_pulse.pin_num);
#endif
	
	/* calculate frequency limit */
	min_step_period = stepgen->step_len;// + stepgen->step_space;
	stepgen->max_freq = 1.0 / (min_step_period * 0.000000001);
	
	// print_string("\nmax_freq");
	// print_ulong((long) (stepgen->max_freq *1000.0));
	// printf("\nmax_freq: %f",max_freq);

	/* maybe these should happen in init...*/
	if (stepgen->maxvel <= 0.0) {
		/* set to zero if negative */
		stepgen->maxvel = 0.0;
	} else {
		/* parameter is non-zero, compare to max_freq */
		desired_freq = stepgen->maxvel * fabs(stepgen->pos_scale);
		if (desired_freq > stepgen->max_freq) {
			/* parameter is too high, complain about it */
			
			// printf("STEPGEN: Channel %d: The requested maximum velocity of %d steps/sec is too high.\n",
				// i, (int)desired_freq);
			// printf("STEPGEN: The maximum possible frequency is %d steps/second\n",
				// (int)stepgen->max_freq);
			/* parameter is too high, limit it */
			stepgen->maxvel = stepgen->max_freq / fabs(stepgen->pos_scale);
		} else {
			/* lower max_freq to match parameter */
			stepgen->max_freq = stepgen->maxvel * fabs(stepgen->pos_scale);
		}
	}

	/* set internal accel limit to its absolute max, which is
	   zero to full speed in one thread period */
	stepgen->max_ac = stepgen->max_freq * recip_dt;
	//printf("\nmax_ac: %f",max_ac);

	/* check for user specified accel limit parameter */
	if (stepgen->maxaccel <= 0.0) {
		/* set to zero if negative */
		stepgen->maxaccel = 0.0;
	} else {
		/* parameter is non-zero, compare to max_ac */
		if ((stepgen->maxaccel * fabs(stepgen->pos_scale)) > stepgen->max_ac) {
			/* parameter is too high, lower it */
			stepgen->maxaccel = stepgen->max_ac / fabs(stepgen->pos_scale);
		} else {
			/* lower limit to match parameter */
			stepgen->max_ac = stepgen->maxaccel * fabs(stepgen->pos_scale);
		}
	}	/* at this point, all scaling, limits, and other parameter
	   changes have been handled - time for the main control */
	stepgen->raw_pos = 0;
	stepgen->timer1 = 0;
	stepgen->addval = 0;
	stepgen->accum = 0;
	stepgen->rawcount = 0;
	stepgen->curr_dir = 0;
	stepgen->prev_dir = 0;
	
	stepgen->target_addval = 0;
	stepgen->deltalim = 0;
	
	stepgen->hold = 0;
	return 1;

}

int stepper_init_pulse_generator(void)
{
	int i;
	// stepper_struct *stepgen;	
#ifdef MULTI_STEPPERS_DIFFERENT_PORTS	
	
#else	
	CLEAR_ALL_MOTOR_PULSE_OUTPUTS = 0;
#endif	
	periodns = 1e9/MAKE_PULSES_FREQ;  // time in ns for freq of execution of makepulses
	periodfp = periodns * 0.000000001;
	freqscale = LONG_PICKOFF * periodfp;
	accelscale = freqscale * periodfp;
	
	dtns = 1e9/FREQ_UPDATE_FREQ;								// period of update_freq in ns
	dt = 1.0/FREQ_UPDATE_FREQ;
	recip_dt = 1.0 / dt;
	
	for (i = 0; i<NUM_AXES; i++){
		// stepgen = &g_stepgen[i];
		stepper_init_pulse_generator_single_axis(i);
	}	
	return 1;
}

/* used to init a profiler for one axis*/
int stepper_init_profiler(profiler_t *p)
{
	// print_string("ProfilerInit\n");
	p->x_final = 0;
	p->a_max = 1.0;
	p->v_max = 20.0; // pulses/msec... = 20,000 pulses/sec
	p->x_curr = 0;
	p->x_next = 0.0;
	p->v_next = 0.0;
	p->eps_x = 2;
	p->ainv2 = 0.5/p->a_max;
	p->mp_mode = MPM_HARD_STOP;
	p->state = MOTOR_STOP;
	return 1;
}


/* initialises the profilers for all the axes*/
int stepper_init_all_profilers(void)
{
	int i;
	for (i = 0 ; i < NUM_AXES ; i++ )
	{
		stepper_init_profiler(&Profiler[i]);
	}
	return 1;
}

/* both the timers of the LPC2148 are used */
/* initialise each of them with their appropriate ISR */
/* and with their appropriate match compare values here*/
int stepper_init_timers(void)
{	
	// we want the timer0 to run at 50Khz
	init_timer1_freq(MAKE_PULSES_TIME);
		
	// we want to run timer1 at 200Khz
	init_timer2_freq(PULSE_CLEAR_TIME);	
	
	// we want to run PWM timer at 1Khz
	init_timer3_freq(FREQ_UPDATE_TIME);
	init_timer0_freq(MODBUS_TIMER_TIME);
	return 1;
}

/* stepper init initialises all the necesssary timers,pulse generators,profilers,etc*/
int stepper_init(void){
	stepper_init_timers();
	stepper_init_pulse_generator();
	stepper_init_all_profilers();
	return 1;
}
#if 0
int stepper_set_curr_pos_as_zero(int axis){
	disableIRQ();
	disable_timer(0);
	disable_timer(1);
	float orig_vel = stepper_get_max_vel(axis);
	float orig_acc = stepper_get_max_acc(axis);
	
	stepper_init_pulse_generator_single_axis(axis);
	stepper_init_profiler(&Profiler[axis]);
	stepper_set_new_profile(axis,orig_vel,orig_acc);
	stepper_update_freq();

	enableIRQ();
	enable_timer(0);
	
	
	return 1;
}
#endif
int print_profiler_data(profiler_t *p){
	// print_string("\nProfiler Data:\nX_final:");
	// print_ulong(p->x_final);
	// print_string("\nX_cur:");
	// print_ulong(p->x_curr);
	return 1;
}

/* this function is used to move motor to the next position given by  stepper_next_position */
int stepper_move_motor(int motor_num){
	g_stepgen[motor_num].pos_cmd = stepper_next_position(&Profiler[motor_num]);
	return 1;
}

/*this function is used to stop all motors
* The Next position for the particular motor is calculated..
*/
int stepper_stop_motor(int motor_num){
	SINGLE stop=0.0;
	stop = stepper_stop_distance(motor_num);
	stop += (SINGLE)stepper_get_cur_pos(motor_num);
	stepper_set_final_pos(motor_num,stop);	
	// printf("StopDist: %d\n",stop);
	return 1;
}

int stepper_stop_synchro_motor(int motor_num){
	int32_t stop=0;
	int32_t div_val=0;
	stop = stepper_stop_distance(motor_num);
	// printf("\nstop_dist1=%d",stop);
	stop += Profiler[motor_num].x_curr;
	div_val=(int32_t)floor(stop/200);
	stop = ((div_val*200)+200);
	// printf("\nStopDist: %d",stop);
	stepper_set_final_pos(motor_num,stop);
	return 1;
}



/* setup a new profile..*/
int stepper_set_new_profile(int axis, float v_max, float a_max, float d_max){
	// print_string("\nset_new_profile");
	profiler_t *p = &Profiler[axis];
	p->a_max = a_max;
	p->v_max = v_max;
	// p->ainv2 = 0.5/p->a_max;
	p->d_max = d_max;
	// p->dinv2 = p->d_max;
	p->dinv2 = 0.5/p->d_max;
	// p->ainv2 = p->d_max;
	// printf("\n  ainv:  %.f  dinv %.f ",p->ainv2,p->dinv2);
	return 1;
}

int stepper_set_new_profile_ainv(int axis, float v_max, float a_max, float a_inv2){
	profiler_t *p = &Profiler[axis];
	p->a_max = a_max;
	p->v_max = v_max;
	p->ainv2 = a_inv2;
	return 1;
}

int stepper_set_new_vmax(int axis, float v_max){
	profiler_t *p = &Profiler[axis];
	p->v_max = v_max;
	return 1;
}

int stepper_set_new_amax(int axis, float a_max){
	profiler_t *p = &Profiler[axis];
	p->a_max = a_max;
	p->ainv2 = 0.5/p->a_max;
	return 1;
}

/* this sets the final position of the motor "motor num "*/
int stepper_set_final_pos(int axis, float steps){
	Profiler[axis].jointpos = steps;
	return 1;
}

int32_t stepper_get_final_pos(int axis){
	return Profiler[axis].jointpos;
}

// int32_t stepper_set_final_pos_acc(int axis, int32_t pos){
	// profiler_t *p = &Profiler[axis];
	// p->x_final += pos;
	// return p->x_final;
// }
int stepper_set_new_de_max(int axis, float d_max){
	profiler_t *p = &Profiler[axis];
	p->d_max = d_max;
	p->dinv2 = 0.5/p->d_max;
	return 1;
}
//return 0 if motor not running else return 1
int stepper_get_running_motor(int axis){
	if (Profiler[axis].state == MOTOR_STOP){
		return 0;
	}
	else{
		return 1;
	}
}

int stepper_get_running_motors(void){
	int i;
	int retVal = 0;
	for(i=0;i<NUM_AXES;i++){
		retVal |= stepper_get_running_motor(i) << i;
	}
	return retVal;
}

SINGLE stepper_get_max_vel(int axis){
	return Profiler[axis].v_max;
}

SINGLE stepper_get_max_acc(int axis){
	return Profiler[axis].a_max;
}

int stepper_set_curr_pos(int axis, float steps){
	Profiler[axis].jointpos=steps; //Without motion...
	Profiler[axis].offset=(SINGLE)(Profiler[axis].x_final)-Profiler[axis].jointpos;
	return 1;
}

// int stepper_steps(int motor_num, float steps){
// 	int retval;
// 	float diff = 0.0;
// 	int32_t final_steps = 0;
// 	profiler_t *p; 
// 	p = &Profiler[motor_num];
// 	p->jointpos += steps;	
// 	return retval;
// }
// /*set the current position as the zero for all the axes*/
// int stepper_set_curr_pos_as_zero(void)
// {
	// int i;
	// for (i = 0 ; i < NUM_AXES ; i++ )
	// {
		// // Profiler[i].x_offset = Profiler[i].x_curr ;
	// }
	// return 1;
// }

int stepper_offset_inc(int axis, int offset){
	Profiler[axis].offset += offset;
	return 1;
}

int32_t stepper_get_cur_pos(int axis){	
	return (Profiler[axis].x_curr - Profiler[axis].offset);
}

/* helper function - computes integeral multiple of increment that is greater
   or equal to value */
static uint32_t ulceil(uint32_t value, uint32_t increment)
{
    if ( value == 0 ) {
		return 0;
    }
    return increment*(((value-1)/increment)+1);
}

int stepper_steps(int motor_num, float steps){
	int retval;
	float diff = 0.0;
	int32_t final_steps = 0;
	profiler_t *p; 
	p = &Profiler[motor_num];
	p->jointpos += steps;	
	return retval;
}


int stepper_reset_axis(int axis){
	// new function added 24/7/2013 to address issue of multistepper failing during motor check
	// ASK amit..
	// print_string("ProfilerInit\n");
	profiler_t *p; 
	p = &Profiler[axis];	
	p->x_final = 0;
	p->jointpos = 0.0;
	p->offset = 0.0;
	// p->x_final_offset = 0.0;
	p->x_curr = 0;
	p->x_next = 0.0;
	p->v_next = 0.0;
	// p->eps_x = 2;
	// p->ainv2 = 0.5/p->a_max;
	// p->mp_mode = MPM_HARD_STOP;
	p->state = MOTOR_STOP;
	stepper_init_pulse_generator_single_axis(axis);	
	return 1;	
}

