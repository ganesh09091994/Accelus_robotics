#ifndef _MULTI_STEPPER_H_
#define _MULTI_STEPPER_H_

#include <stdint.h>
#include <math.h>
#include "types.h"
#include "config.h"
#include "multi-stepper-ports.h"
#include "debug_frmwrk.h"

#define MAKE_PULSES_TIME	(49)		// (us) 20khz
#define MAKE_PULSES_FREQ	(20000)		// in Hz
// #define MAKE_PULSES_TIME	(19)		// (us) 50khz
// #define MAKE_PULSES_FREQ	(50000)		// in Hz
// #define MAKE_PULSES_TIME	(15)		// (us) 60khz
// #define MAKE_PULSES_FREQ	(60000)		// in Hz
// #define MAKE_PULSES_TIME	(12)		// (us) 75khz
// #define MAKE_PULSES_FREQ	(75000)		// in Hz
// #define MAKE_PULSES_TIME	(9)			// (us) 100khz
// #define MAKE_PULSES_FREQ	(100000)	// in Hz
// #define MAKE_PULSES_TIME	(4)			// (us) 75khz
// #define MAKE_PULSES_FREQ	(200000)	// in Hz

#define FREQ_UPDATE_TIME	(999)		// (us) 1khz
#define FREQ_UPDATE_FREQ	(1000)		// in Hz

#define PULSE_CLEAR_TIME	(1)			// (us) 200khz
#define PULSE_CLEAR_FREQ	(200000)	// in Hz

#define DELAY_TIMER_CLK_TIME	(999)	// (us)
#define DELAY_TIMER_CLK_FREQ	(1000)	// Hz

#define MODBUS_TIMER_TIME	(49)		//
#define MODBUS_TIMER_FREQ	(20000)		//

#define PICKOFF			(28)
#define LONG_PICKOFF	(1L << PICKOFF)
#define MINUS_PICKOFF	(1<< (PICKOFF-1))
#define DIVIDE_PICKOFF	(1.0 / LONG_PICKOFF)

#define T 1	// for documenting dimension: MUST BE UNITY
#define sign(x) (x == 0 ? 0 : (x > 0 ? 1 : -1))
#define min(x,y) (x < y ? x : y)

#define NUM_AXES 2
#define MAX_NUM_STEPPERS 2

#define EINT0_CLEAR						0x00000001
#define EINT1_CLEAR						0x00000002
#define EINT2_CLEAR						0x00000004
#define EINT3_CLEAR						0x00000008

#define	Modbus_Timeout_ISR		TIMER0_IRQHandler
#define	Make_Pulses_ISR			TIMER1_IRQHandler
#define	Pulse_Clear_ISR			TIMER2_IRQHandler
#define	Delay_Timer_ISR			TIMER3_IRQHandler

#define MULTI_STEPPER_MAKE_PULSES_TIMER		enable_timer(LPC_TIM1)		// 50khz timer start
#define MULTI_STEPPER_PULSE_CLEAR_TIMER		enable_timer(LPC_TIM2)		// 200khz timer start
#define MULTI_STEPPER_MOVE_MOTOR_TIMER		enable_timer(LPC_TIM3)		// 1khz timer start

#define START_MODBUS_TIMER					enable_timer(LPC_TIM0)		// 100kHz modbus timer start
#define START_DELAY_TIMER					enable_timer(LPC_TIM3)		// 1kHz delay timer start

#define STOP_MODBUS_TIMER					disable_timer(LPC_TIM0)		// 100kHz modbus timer stop
#define STOP_DELAY_TIMER					disable_timer(LPC_TIM3)		// 1kHz delay timer stop

#define RESET_MODBUS_TIMER					reset_timer(LPC_TIM0)		// reset modbus timer


#define GPIOpin_t GPIOPin
#define STOPPING_DIST_PADDING 10

/*IO_struct to define the various pin and port access constants*/
typedef struct{
	uint8_t 	pulse_port;//defines the port of the pulse pin
	uint32_t 	pulse_port_set;//defines the portset address
	uint32_t 	pulse_port_clear;//defines the port clear register address
	uint32_t 	pulse_port_dir;//defines the port dir register address
	uint8_t 	pulse_pin_offset;//defines the pulse pin address
	
	uint8_t 	dir_port;
	uint32_t 	dir_port_set;
	uint32_t 	dir_port_clear;
	uint32_t 	dir_port_dir;
	uint8_t 	dir_pin_offset;
} io_struct;

/* struct template definitions */
typedef struct {
	// variables that are RW by make_pulses
	uint32_t timer1;			// times out when step pulse should end..
	int32_t addval;				// actual frequency generator add value
	volatile int64_t accum;		// frequency generator accumulator...
	int32_t rawcount;			// position feedback in counts..
	int curr_dir;				// current direction...
	
	int prev_dir;				// previous direction...
	int hold;				// counter for direction change hold...
	
	int32_t raw_pos;			// motor position in counts..
	int8_t state;				// current state of the motor i.e. moving or stopped..
	
	// Stuff that is RO in make_pulses
	int enable;
	int32_t target_addval;		// desired frequency generator value
	int32_t deltalim;			// max allowed change per period.
	uint32_t step_len;			// this is the actual length of the step..
	uint32_t step_space;		// this is the space between the steps...
	GPIOpin_t pin_pulse;		// the pin for pulse output
	GPIOpin_t pin_dir;			// the pin for dir output
	
	// stuff that is not accessed by make_pulses..
	float max_freq;		// max freq that the steps can be generated at... based on step_len + step_space
	float max_ac;		// this is the max acceleration that can be achieved.. based on max_freq..
	float pos_scale;
	float scale_recip;		/* reciprocal value used for scaling */
	float maxvel;		/* param: max velocity, (pos units/sec) */
    float maxaccel;	/* param: max accel (pos units/sec^2) */
	float freq;		/* param: frequency command.. this is only for storing */
	float vel_cmd;	/* pin: velocity command (pos units/sec) */
    float pos_cmd;	/* pin: position command (position units) */
	float old_pos_cmd;	/* previous position command (counts) */
} stepper_struct;

typedef struct{
	int32_t x_final;
	SINGLE v_max;
	SINGLE a_max;
	int32_t x_curr;	// the desired position of the motor
	SINGLE x_next;	//next position which is returned in next_position;
	SINGLE v_next;
	SINGLE ainv2;
	SINGLE jointpos; // this is the joint position	
	SINGLE offset; // this motor_position = jointpos + motor_offset	
	uint8_t state;
	int32_t eps_x;
	int mp_mode;

	SINGLE d_max;
	SINGLE dinv2;
} profiler_t;

typedef enum {	
	MOTOR_STOP = 0,
	MOTOR_RAMPCOAST = 1,
	MOTOR_RAMPDOWN = 2,
	MOTOR_OVERSHOOT = 3
} MOTOR_MODES;

typedef enum {
	MPM_HARD_STOP = 0,
	MPM_RUBBER_BAND = 1,
} PROFILER_MODES;

//functions
int stepper_init_pulse_generator(void);
//int stepper_init_pulse_generator_single_axis(stepper_struct *stepgen, int i);
int stepper_init_pulse_generator_single_axis(int);
int stepper_make_pulses(void);
int stepper_update_freq(void);
int32_t stepper_next_position(profiler_t *);
int stepper_init_profiler(profiler_t *);
SINGLE stepper_stop_distance(int);
int stepper_init_timers(void);
int stepper_init(void);
int stepper_move_motor(int);
int stepper_stop_motor(int);
//int stepper_set_motor_final(int32_t final,int motor_num);
int stepper_init_all_profilers(void);
int stepper_set_new_profile(int, float, float, float);
int stepper_set_new_profile_ainv(int, float, float, float);
int stepper_set_new_vmax(int, float);
int stepper_set_new_amax(int, float);
int stepper_set_final_pos(int, float);
int32_t stepper_get_final_pos(int);

// int32_t stepper_set_final_pos_acc(int, int32_t);
int stepper_get_running_motor(int);
int stepper_get_running_motors(void);
int stepper_set_curr_pos(int, float);
SINGLE stepper_get_max_vel(int);
SINGLE stepper_get_max_acc(int);
int32_t stepper_get_cur_pos(int);

int stepper_steps(int, float);
int stepper_offset_inc(int, int);

int stepper_reset_axis(int);
int stepper_stop_synchro_motor(int);
int stepper_set_new_de_max(int, float);
extern int trig_update_freq;
extern uint32_t delay_timer_32;
//struct declaration
extern stepper_struct g_stepgen[NUM_AXES];
extern profiler_t Profiler[NUM_AXES];

extern uint8_t modbus_write;
#endif
