#include "SM_Jogging.h"
// #include "board.h"
#include "config.h"
#include "global.h"
#include "state_machine.h"
#include "delay_timer.h"
#include "config_data.h"

extern "C"{ 
	#include "input_output.h"
	#include "multi-steppers.h"
}

SM_Jogging::SM_Jogging():State_Machine()
{
	SM_stop();
	state=SM_JOGGING_RESET;
	jog_type = JOG_STEP;
}

SM_Jogging::SM_Jogging(int motor):State_Machine()
{
	SM_stop();
	state = SM_JOGGING_RESET;
	motor_num = motor;
}

int SM_Jogging::update(void){
	retVal = SMJ_RESET;
	switch(state){
		case SM_JOGGING_RESET:
			if (SM_START == SM_started()){
				amax = jog_accel;
				if(motor_num == MOTOR0){

					vmax =(float)((float)((jog_speed_1*drive_PPR*gearratio_num_1)/(float)(1000*PI*motor_diameter_1*gearratio_den_1)));

				}else if(motor_num == MOTOR1){

					vmax =(float)((float)((jog_speed_2*drive_PPR*gearratio_num_2)/(float)(1000*PI*motor_diameter_2*gearratio_den_2)));
				}

				motor_ppr_per_mm_1 = (float)((drive_PPR*(gearratio_num_1/gearratio_den_1))/(PI*motor_diameter_1));
				motor_ppr_per_mm_2 = (float)((drive_PPR*(gearratio_num_2/gearratio_den_2))/(PI*motor_diameter_2));
				
				if(jog_type == JOG_CONT){
					jog_distance = float(0x7FFFFFFL);
				}else{
					if(motor_num == MOTOR0){
						jog_distance = jog_distance_1*motor_ppr_per_mm_1;
						// printf("\n jog_distance_1 =%f",jog_distance);
					}else if(motor_num == MOTOR1){
						jog_distance = jog_distance_2*motor_ppr_per_mm_2;
						// printf("\n jog_distance_2 =%f",jog_distance);
					}			
				}
				
				stepper_set_new_profile(motor_num,vmax,amax,amax);
				
				if(direction == JOG_FWD){
					stepper_steps(motor_num,jog_distance);
				
				}else{
					stepper_steps(motor_num,-jog_distance);
					// printf("\n rev");
				}

				if (jog_type == JOG_STEP){
					next_state(WAIT_TO_STOP_FOR_STEP_JOG);
				}else {
					forced_stop = 0;
					next_state(WAIT_FOR_BUTTON_RELEASE);
				}				
				retVal = SMJ_STARTED;
			}			
			break;
		
		case WAIT_TO_STOP_FOR_STEP_JOG: // wait for motor to stop.. on it's own...
			if(stepper_get_running_motor(motor_num) == 0){ 				
				retVal = SMJ_DONE;
				next_state(SM_JOGGING_RESET);
				SM_stop();
			}else {
				retVal = SMJ_STARTED;
			}
			break;
		
		case WAIT_FOR_BUTTON_RELEASE: // wait for forced stop...
			if (forced_stop){
				next_state();
				stepper_stop_motor(motor_num);
				SM_stop();
			}
			retVal = SMJ_STARTED;
			break;
		default:			
			retVal = SMJ_ERROR;
			next_state(SM_JOGGING_RESET);
			break;
	}
	return retVal;
}

int SM_Jogging::stop_now(){
		SM_stop();
		stepper_stop_motor(motor_num);
		next_state(SM_JOGGING_RESET);
		return 1;
}
