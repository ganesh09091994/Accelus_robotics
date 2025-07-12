#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

#define SM_START 1
#define SM_START_0 1
#define SM_START_1 1

#define SM_STOP 0

#define SM_OPERATION_AUTO			0
#define SM_OPERATION_MANUAL			1

/*Return codes for state machines...*/
enum SM_RET{
	SM_RESET = 0,
	SM_STARTED = 1,
	SM_DONE = 2,
	SM_ERROR = -1,
	SM_ABORTED = 3,
};

#define SM_STATE_RESET	0

class State_Machine{
	public:
	volatile int state;
	volatile int flag_start;
	
	// volatile int pause_cur_state;
	// volatile int pause_next_state;
	volatile int manual_flag;
	volatile int paused;
	
	State_Machine(void);
	State_Machine (int s); // consructor which calls the state machine..
	int next_state(void);    	// incremen state by 1
	int next_state(int s);  // set nex state to value state..
	
	// virtual functions return -1
	virtual int update(void);
	
	// mealy machine with input...
	virtual int update(int);
	
	virtual int reset(void);
	
	// this function returns 1 if state machine is started...
	virtual int SM_started();
	
	virtual int SM_start();
	virtual int SM_stop();
	
	// This will pause the state machine if manual is enabled..
	// it will return if the machine has paused or not..
	//virtual int SM_pause(void);
	
	virtual int SM_pause(int input);
	
	// this should be called in the pause state of the state machine...
	// this will take an input and if input is 1 it will release from pause to the next state...
	//virtual int SM_pause_update(int input);
	
	
	virtual ~State_Machine(void){};
};

#endif
