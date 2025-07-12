#include "SM_Manual.h"
#include "config.h"
#include "global.h"
#include "state_machine.h"
#include "delay_timer.h"
#include "config_data.h"

extern "C"{ 
	#include "input_output.h"
	#include "multi-steppers.h"
}



#define MOTOR_STOPPED   0


SM_MANUAL::SM_MANUAL():State_Machine()	//add all the parameters to use in state machine
{	
	SM_stop();
	curr_laser_count = 0;
	state = SM_MANUAL_RESET;
}

int SM_MANUAL::update(void){
	retVal = SMM_RESET;

	switch(state){

		case SM_MANUAL_RESET:

			if(SM_START == SM_started()){
				if(next_stage){
					next_stage = 0;
					curr_laser_count = 0;
					stepper_reset_axis(MOTOR0);
					stepper_reset_axis(MOTOR1);
					vmax =(float)((float)((motor_speed_1*drive_PPR*gearratio_num_1)/(float)(1000*PI*motor_diameter_1*gearratio_den_1)));
					stepper_set_new_profile(MOTOR0,vmax,motor_accel_1,motor_accel_1);
					vmax =(float)((float)((motor_speed_2*drive_PPR*gearratio_num_2)/(float)(1000*PI*motor_diameter_2*gearratio_den_2)));
					stepper_set_new_profile(MOTOR1,vmax,motor_accel_2,motor_accel_2);
					motor_ppr_per_mm_1 = (float)((drive_PPR*(gearratio_num_1/gearratio_den_1))/(PI*motor_diameter_1));
					motor_ppr_per_mm_2 = (float)((drive_PPR*(gearratio_num_2/gearratio_den_2))/(PI*motor_diameter_2));
					dist = play_distance_1*motor_ppr_per_mm_1;
					if(motor_dir_1){
						stepper_set_final_pos(MOTOR0,-dist);	
					}else{
						stepper_set_final_pos(MOTOR0,dist);
					}

					dist = play_distance_2*motor_ppr_per_mm_2;
					if(motor_dir_2){
						stepper_set_final_pos(MOTOR1,-dist);	
					}else{
						stepper_set_final_pos(MOTOR1,dist);
					}
					next_state(SM_MANUAL_WAIT_FOR_MOTOR_TO_STOP);
				}
			}else{

				retVal = SMM_WAITING;
			}
			
		break;

		case SM_MANUAL_WAIT_FOR_MOTOR_TO_STOP:
			if(stepper_get_running_motor(MOTOR0) == MOTOR_STOPPED &&  stepper_get_running_motor(MOTOR1) == MOTOR_STOPPED){
				next_state(SM_MANUAL_WAIT_FOR_CYCLE_START);
			}
		break;

		case SM_MANUAL_WAIT_FOR_CYCLE_START:
			if(stage_bypass[0]){
				next_state(SM_MANUAL_WAIT_FOR_BEND1_ON);
			}else{
				if(next_stage){
					next_stage = 0;
					CUT_ON;
					next_state(SM_MANUAL_WAIT_FOR_BEND1_ON);
				}
			}
		break;		

		case SM_MANUAL_WAIT_FOR_BEND1_ON:
			if(stage_bypass[1]){
				next_state(SM_MANUAL_WAIT_FOR_BEND2_ON);

			}else{
				
				if(bend_individually){
					if(next_stage){
						next_stage = 0;
						BEND1_ON;
						next_state(SM_MANUAL_WAIT_FOR_BEND2_ON);
					}
				}else{
					if(next_stage){
						next_stage = 0;
						BEND1_ON;
						BEND2_ON;
						next_state(SM_MANUAL_WAIT_FOR_BEND2_ON);
					}
				}
			}
		break;

		case SM_MANUAL_WAIT_FOR_BEND2_ON:
			if(stage_bypass[2]){
				next_state(SM_MANUAL_WAIT_FOR_LASER_ON);

			}else {
				if(bend_individually){
					if(next_stage){
						next_stage = 0;
						BEND2_ON;
						next_state(SM_MANUAL_WAIT_FOR_LASER_ON);
					}
				}else{
					next_state(SM_MANUAL_WAIT_FOR_LASER_ON);
				}
			}
		break;

		case SM_MANUAL_WAIT_FOR_LASER_ON:
			if(stage_bypass[3]){
				next_state(SM_MANUAL_WAIT_FOR_BC_OFF);

			}else{
				if(next_stage){
					next_stage = 0;
					LASER_ON;
					curr_laser_count++;
					next_state(SM_MANUAL_WAIT_FOR_LASER_OFF);
				}
			}
		break;

		case SM_MANUAL_WAIT_FOR_LASER_OFF:
			if(next_stage){
				next_stage = 0;
				LASER_OFF;
				if(curr_laser_count < no_of_laser){
					next_state(SM_MANUAL_WAIT_FOR_LASER_ON);						
				}else{
					next_state(SM_MANUAL_WAIT_FOR_BC_OFF);
					
				}
			}
			
		break;

		case SM_MANUAL_WAIT_FOR_BC_OFF:

			if(next_stage){
				next_stage = 0;
				CUT_OFF;
				BEND1_OFF;
				BEND2_OFF;
				next_state(SM_MANUAL_WAIT_FOR_EJECT_ON);
			}

		break;

		case SM_MANUAL_WAIT_FOR_EJECT_ON:
			if(stage_bypass[4]){
				next_state(SM_MANUAL_WAIT_FOR_EJECT_OFF);

			}else{
				if(next_stage){
					next_stage = 0;
					EJECT_ON;
					next_state(SM_MANUAL_WAIT_FOR_EJECT_OFF);
				}
			}
		break;

		case SM_MANUAL_WAIT_FOR_EJECT_OFF:
			if(next_stage){
				next_stage = 0;

				if(!stage_bypass[4]){
					EJECT_OFF;
				}
				product_count++;
				// if(batch_count){
				// 	if(product_count >= batch_count){
				// 		// stop_now();
				// 	}else{
				// 		next_state(SM_MANUAL_RESET);	
				// 	}
				// }else{
				// 	next_state(SM_MANUAL_RESET);
				// }				
				next_state(SM_MANUAL_RESET);
			}
		break;

	}
    return retVal;
}

int SM_MANUAL::stop_now(){
	if (SM_START == SM_started()) { // if state machine has started...
		stepper_stop_motor(MOTOR0);
		stepper_stop_motor(MOTOR1);
		CUT_OFF;
		BEND1_OFF;
		BEND2_OFF;
		LASER_OFF;
		EJECT_OFF;
		SM_stop();
		next_state(SM_MANUAL_RESET);
		return 1;
	}
	return 0;
}

