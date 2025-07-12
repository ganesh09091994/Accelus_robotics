#ifndef _SM_MANUAL_H_
#define _SM_MANUAL_H_

#include "state_machine.h"

extern "C"{ 
	#include "input_output.h"
}

enum SM_MANUAL_STATES{

	SM_MANUAL_RESET = 0,
	SM_MANUAL_WAIT_FOR_MOTOR_TO_STOP,
	SM_MANUAL_WAIT_FOR_CYCLE_START,
	SM_MANUAL_WAIT_FOR_BEND1_ON,
	SM_MANUAL_WAIT_FOR_BEND2_ON,
	SM_MANUAL_WAIT_FOR_LASER_ON,
	SM_MANUAL_WAIT_FOR_LASER_OFF,
	SM_MANUAL_WAIT_FOR_BC_OFF,
	SM_MANUAL_WAIT_FOR_EJECT_ON,
	SM_MANUAL_WAIT_FOR_EJECT_OFF,
	SM_MANUAL_INDEX_CONVEYOR_MAX_STATES
};

enum SM_MANUAL_RET{
	SMM_RESET = 0,
	SMM_STARTED = 1,
	SMM_DONE = 2,
	SMM_WAITING = 3,
	SMM_ERROR = -1,
};


class SM_MANUAL: public State_Machine { 
	public:

	float vmax;
	int start_input;
	float steps;
	int drive_error_input;
	int drive_error_output;
	uint8_t prev_drive_error_state;
	float dist;
	uint8_t curr_laser_count;
	SM_MANUAL(void);
	int update(void);	
	int stop_now(void);
	int retVal;
};

#endif
