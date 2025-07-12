#ifndef _SM_OPERATION_H_
#define _SM_OPERATION_H_

#include "state_machine.h"

extern "C"{ 
	#include "input_output.h"
}

enum SM_OPERATION_STATES{

	SM_OPERATION_RESET = 0,
	SM_WAIT_FOR_MOTOR_TO_STOP,
	SM_WAIT_FOR_CYCLE_START,
	SM_WAIT_FOR_BEND1_ON,
	SM_WAIT_FOR_BEND2_ON,
	SM_WAIT_FOR_LASER_ON,
	SM_WAIT_FOR_LASER_OFF,
	SM_WAIT_FOR_BC_OFF,
	SM_WAIT_FOR_EJECT_ON,
	SM_WAIT_FOR_EJECT_OFF,
	SM_INDEX_CONVEYOR_MAX_STATES
};

enum SM_OPERATION_RET{
	SMO_RESET = 0,
	SMO_STARTED = 1,
	SMO_DONE = 2,
	SMO_WAITING = 3,
	SMO_ERROR = -1,
};


class SM_OPERATION: public State_Machine { 
	public:

	float vmax;
	int start_input;
	float steps;
	int drive_error_input;
	int drive_error_output;
	uint8_t prev_drive_error_state;
	float dist;
	uint8_t curr_laser_count;
	// SM_OPERATION(void);
	SM_OPERATION(void);
	int update(void);	
	int stop_now(void);
	int retVal;
};

#endif
