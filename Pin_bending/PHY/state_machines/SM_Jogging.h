#ifndef _SM_JOGGING_H_
#define _SM_JOGGING_H_

#include "state_machine.h"

#define STOPPED 0
#define DIRECTION 1

enum SM_JOGGING_STATES{
	SM_JOGGING_RESET = 0,
	WAIT_TO_STOP_FOR_STEP_JOG = 1,
	WAIT_FOR_BUTTON_RELEASE = 2,
	WAIT_TO_START_MOTOR = 4,
};

enum SM_JOGGING_RET{
	SMJ_RESET = 0,
	SMJ_STARTED = 1,
	SMJ_DONE = 2,
	SMJ_ERROR = -1,
};

enum JOG_TYPES{
	JOG_CONT = 0,
	JOG_STEP = 1,
};

enum JOG_DIR{
	JOG_FWD = 0,
	JOG_REV = 1,
};

class SM_Jogging: public State_Machine { 
	public:
	int retVal;
	int motor_num;
	int direction;
	float amax;
	float vmax;
	float jog_distance;
	
	// uint8_t jog_rev;
	// uint8_t jog_fwd;
	volatile char forced_stop;	
	SM_Jogging();	
	SM_Jogging(int);
	char change_jog_type(void);	
		
	int update(void);
	int stop_now(void);
};
#endif
