#include <stdio.h>

#include "state_machine.h"

State_Machine::State_Machine(void){
	state = 0;
	flag_start = 0;
	manual_flag = SM_OPERATION_AUTO;
}

State_Machine::State_Machine(int s){
	state = s;
	flag_start = 0;
	manual_flag = SM_OPERATION_AUTO;
}

int State_Machine::next_state(void){
	return state++;
}

int State_Machine::next_state(int s){
	state = s;
	return state;
}

int State_Machine::update(void) { 
	return -1;
};

int State_Machine::update(int){
	return -1;
}

int State_Machine::reset(void) {
	state = 0;
	flag_start = 0;
	return -1;
};

int State_Machine::SM_started(){
	return flag_start;
}

int State_Machine::SM_start(){
	flag_start = SM_START;
	flag_start = SM_START_0;
	flag_start = SM_START_1;
	return flag_start;
}

int State_Machine::SM_stop(){
	state = 0;
	flag_start = SM_STOP;
	return flag_start;
}
/*
int State_Machine::SM_pause(int nextState){
	if (manual_flag == SM_OPERATION_MANUAL){
		pause_cur_state = state;
		pause_next_state = nextState;
		next_state(SM_STATE_PAUSE);
	}
	return manual_flag;
}
*/
/************************
this will pause the state machine..
************************/
int State_Machine::SM_pause(int input){
	if (!paused){
		if (manual_flag == SM_OPERATION_MANUAL){
			paused = 1;
		}
	}else { // wea re paused...
		if (input){ // get out of pause state
			paused = 0;
		}
	}
	return paused;
}
/*
int State_Machine::SM_pause_update(int input){
	if (input) {
		// release from pause...
		next_state(pause_cur_state);
		return 1;
	}else {
		return 0;
	}
}
*/
