#include "hmi.h" 
#include <stdint.h>
#include <stdio.h>
#include "delay_timer.h"

uint32_t hmi_execute_reg;
uint32_t hmi_bypass_reg;
uint8_t error_reset = 0;
uint8_t op_toggle[5];
uint16_t led_status = 0;

int detect_val_change(unsigned int lower_reg, unsigned int higher_reg, int count, float value){
	unsigned int lw_z;
	unsigned int hw_z;
	a.f_val = value;

	lw_z = (a.u_val & 0x0000FFFF);
	hw_z = ((a.u_val >> 16) & 0x0000FFFF);
	if(lower_reg != lw_z || higher_reg != hw_z){
		return 1;
	}else{
		return 0;
	}
}

void get_holding_reg_vals(float para_val, int modbus_reg, int idx_offset,int para_offset){
	unsigned int lw_z;
	unsigned int hw_z;
	a.f_val = para_val;

	lw_z = (a.u_val & 0x0000FFFF);
	hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
	if(holdingRegs[(modbus_reg+(idx_offset*para_offset))] != lw_z || holdingRegs[(modbus_reg+(idx_offset*para_offset))+1] != hw_z){
		holdingRegs[(modbus_reg+(idx_offset*para_offset))]= lw_z;
		holdingRegs[(modbus_reg+(idx_offset*para_offset))+1]= hw_z;
	}
}

void get_holding_reg_vals_int(unsigned int para_val, int modbus_reg, int idx_offset,int para_offset){
	unsigned int lw_z;
	unsigned int hw_z;
	a.u_val = para_val;
	lw_z = (a.u_val & 0x0000FFFF);
	hw_z = ((a.u_val >> 16) & 0x0000FFFF);
	if(holdingRegs[(modbus_reg+(idx_offset*para_offset))] != lw_z || holdingRegs[(modbus_reg+(idx_offset*para_offset))+1] != hw_z){
		holdingRegs[(modbus_reg+(idx_offset*para_offset))]= lw_z;
		holdingRegs[(modbus_reg+(idx_offset*para_offset))+1]= hw_z;
	
	}
}

int set_holding_registers_values(int modbus_cnt){
	unsigned int temp_int=0;
	unsigned int lw_z= 0;
	unsigned int hw_z= 0;

	switch(modbus_cnt){
		
		case MB_MOTOR_SPD_1:
			a.f_val = motor_speed_1;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_MOTOR_SPD_1] != lw_z || holdingRegs[MB_MOTOR_SPD_1+1] != hw_z){
				holdingRegs[MB_MOTOR_SPD_1]= lw_z;
				holdingRegs[MB_MOTOR_SPD_1+1]= hw_z;
			}
			break;

		case MB_MOTOR_DIA_1:
			a.f_val = motor_diameter_1;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_MOTOR_DIA_1] != lw_z || holdingRegs[MB_MOTOR_DIA_1+1] != hw_z){
				holdingRegs[MB_MOTOR_DIA_1]= lw_z;
				holdingRegs[MB_MOTOR_DIA_1+1]= hw_z;
			}
			break;

		case MB_MOTOR_ACC_1:
			temp_int = (unsigned int)(motor_accel_1*1000);
			if(temp_int != holdingRegs[MB_MOTOR_ACC_1])
				holdingRegs[MB_MOTOR_ACC_1] = temp_int;
			break;

		case MB_PLAY_DIST_1:
			a.f_val = play_distance_1;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_PLAY_DIST_1] != lw_z || holdingRegs[MB_PLAY_DIST_1+1] != hw_z){
				holdingRegs[MB_PLAY_DIST_1]= lw_z;
				holdingRegs[MB_PLAY_DIST_1+1]= hw_z;
			}
			break;

		case MB_GEAR_NUM_1:
			temp_int = (unsigned int)(gearratio_num_1);
			if(temp_int != holdingRegs[MB_GEAR_NUM_1])
				holdingRegs[MB_GEAR_NUM_1] = temp_int;
			break;

		case MB_GEAR_DEN_1:
			temp_int = (unsigned int)(gearratio_den_1);
			if(temp_int != holdingRegs[MB_GEAR_DEN_1])
				holdingRegs[MB_GEAR_DEN_1] = temp_int;
			break;

		case MB_JOG_DIST_1:
			a.f_val = jog_distance_1;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_JOG_DIST_1] != lw_z || holdingRegs[MB_JOG_DIST_1+1] != hw_z){
				holdingRegs[MB_JOG_DIST_1]= lw_z;
				holdingRegs[MB_JOG_DIST_1+1]= hw_z;
			}
			break;


		case MB_JOG_SPEED_1:
			a.f_val = jog_speed_1;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_JOG_SPEED_1] != lw_z || holdingRegs[MB_JOG_SPEED_1+1] != hw_z){
				holdingRegs[MB_JOG_SPEED_1]= lw_z;
				holdingRegs[MB_JOG_SPEED_1+1]= hw_z;
			}
			break;

		case MB_MOTOR_SPD_2:
			a.f_val = motor_speed_2;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_MOTOR_SPD_2] != lw_z || holdingRegs[MB_MOTOR_SPD_2+1] != hw_z){
				holdingRegs[MB_MOTOR_SPD_2]= lw_z;
				holdingRegs[MB_MOTOR_SPD_2+1]= hw_z;
			}
			break;

		case MB_MOTOR_DIA_2:
			a.f_val = motor_diameter_2;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_MOTOR_DIA_2] != lw_z || holdingRegs[MB_MOTOR_DIA_2+1] != hw_z){
				holdingRegs[MB_MOTOR_DIA_2]= lw_z;
				holdingRegs[MB_MOTOR_DIA_2+1]= hw_z;
			}
			break;

		case MB_MOTOR_ACC_2:
			temp_int = (unsigned int)(motor_accel_2*1000);
			if(temp_int != holdingRegs[MB_MOTOR_ACC_2]){
				holdingRegs[MB_MOTOR_ACC_2] = temp_int;
			}
			break;

		case MB_PLAY_DIST_2:
			a.f_val = play_distance_2;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_PLAY_DIST_2] != lw_z || holdingRegs[MB_PLAY_DIST_2+1] != hw_z){
				holdingRegs[MB_PLAY_DIST_2]= lw_z;
				holdingRegs[MB_PLAY_DIST_2+1]= hw_z;
			}
			break;

		case MB_GEAR_NUM_2:
			temp_int = (unsigned int)(gearratio_num_2);
			if(temp_int != holdingRegs[MB_GEAR_NUM_2]){
				holdingRegs[MB_GEAR_NUM_2] = temp_int;
			}
			break;

		case MB_GEAR_DEN_2:
			temp_int = (unsigned int)(gearratio_den_2);
			if(temp_int != holdingRegs[MB_GEAR_DEN_2]){
				holdingRegs[MB_GEAR_DEN_2] = temp_int;
			}
			break;


		case MB_JOG_DIST_2:
			a.f_val = jog_distance_2;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_JOG_DIST_2] != lw_z || holdingRegs[MB_JOG_DIST_2+1] != hw_z){
				holdingRegs[MB_JOG_DIST_2]= lw_z;
				holdingRegs[MB_JOG_DIST_2+1]= hw_z;
			}
			break;


		case MB_JOG_SPEED_2:
			a.f_val = jog_speed_2;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);	
			if(holdingRegs[MB_JOG_SPEED_2] != lw_z || holdingRegs[MB_JOG_SPEED_2+1] != hw_z){
				holdingRegs[MB_JOG_SPEED_2]= lw_z;
				holdingRegs[MB_JOG_SPEED_2+1]= hw_z;
			}
			break;


		case MB_JOG_ACC:
			temp_int = (unsigned int)(jog_accel*1000);
			if(temp_int != holdingRegs[MB_JOG_ACC]){
				holdingRegs[MB_JOG_ACC] = temp_int;
			}
			break;

		case MB_CYCLE_START_DELAY:
			temp_int = (unsigned int)(cycle_start_delay);
			if(temp_int != holdingRegs[MB_CYCLE_START_DELAY]){
				holdingRegs[MB_CYCLE_START_DELAY] = temp_int;
			}
			break;

		case MB_CUT_DELAY:
			temp_int = (unsigned int)(cut_delay);
			if(temp_int != holdingRegs[MB_CUT_DELAY]){
				holdingRegs[MB_CUT_DELAY] = temp_int;
			}
			break;

		case MB_BEND1_DELAY:
			temp_int = (unsigned int)(bend1_delay);
			if(temp_int != holdingRegs[MB_BEND1_DELAY]){
				holdingRegs[MB_BEND1_DELAY] = temp_int;
			}
			break;

		case MB_BEND2_DELAY:
			temp_int = (unsigned int)(bend2_delay);
			if(temp_int != holdingRegs[MB_BEND2_DELAY]){
				holdingRegs[MB_BEND2_DELAY] = temp_int;
			}
			break;

		case MB_LASER_ON_DELAY:
			temp_int = (unsigned int)(laser_on_delay);
			if(temp_int != holdingRegs[MB_LASER_ON_DELAY]){
				holdingRegs[MB_LASER_ON_DELAY] = temp_int;
			}
			break;

		case MB_LASER_OFF_DELAY:
			temp_int = (unsigned int)(laser_off_delay);
			if(temp_int != holdingRegs[MB_LASER_OFF_DELAY]){
				holdingRegs[MB_LASER_OFF_DELAY] = temp_int;
			}
			break;

		case MB_EJECT_DELAY:
			temp_int = (unsigned int)(eject_delay);
			if(temp_int != holdingRegs[MB_EJECT_DELAY]){
				holdingRegs[MB_EJECT_DELAY] = temp_int;
			}
			break;

		case MB_LASER_NUM:
			temp_int = (unsigned int)(no_of_laser);
			if(temp_int != holdingRegs[MB_LASER_NUM]){
				holdingRegs[MB_LASER_NUM] = temp_int;
			}
			break;

		case MB_DRIVE_PPR:
			temp_int = (unsigned int)(drive_PPR);
			if(temp_int != holdingRegs[MB_DRIVE_PPR]){
				holdingRegs[MB_DRIVE_PPR] = temp_int;
			}
			break;

		case MB_PRODUCT_COUNT:
			a.u_val = product_count;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);
			if(holdingRegs[MB_PRODUCT_COUNT] != lw_z || holdingRegs[MB_PRODUCT_COUNT+1] != hw_z){
				holdingRegs[MB_PRODUCT_COUNT]= lw_z;
				holdingRegs[MB_PRODUCT_COUNT+1]= hw_z;
			}
			break;

		case MB_BATCH_COUNT:
			a.u_val = batch_count;
			lw_z = (a.u_val & 0x0000FFFF);
			hw_z = ((a.u_val >> 16) & 0x0000FFFF);
			if(holdingRegs[MB_BATCH_COUNT] != lw_z || holdingRegs[MB_BATCH_COUNT+1] != hw_z){
				holdingRegs[MB_BATCH_COUNT]= lw_z;
				holdingRegs[MB_BATCH_COUNT+1]= hw_z;
			}
			break;

		case MB_JOG_TYPE:
			temp_int = (unsigned int)(jog_type);
			if(temp_int != holdingRegs[MB_JOG_TYPE]){
				holdingRegs[MB_JOG_TYPE] = temp_int;
			}
			break;

		case MB_OP_STATE:
			temp_int = (unsigned int)(op_state);
			if(temp_int != holdingRegs[MB_OP_STATE]){
				holdingRegs[MB_OP_STATE] = temp_int;
			}
			break;
		
		case MB_MOTOR_DIR_1:
			temp_int = (unsigned int)(motor_dir_1);
			if(temp_int != holdingRegs[MB_MOTOR_DIR_1]){
				holdingRegs[MB_MOTOR_DIR_1] = temp_int;
			}
			break;
			
		case MB_MOTOR_DIR_2:
			temp_int = (unsigned int)(motor_dir_2);
			if(temp_int != holdingRegs[MB_MOTOR_DIR_2]){
				holdingRegs[MB_MOTOR_DIR_2] = temp_int;
			}
			break;

		// case MB_CUT_BYPASS:
		// 	temp_int = stage_bypass[0];
		// 	if(temp_int != holdingRegs[MB_CUT_BYPASS])
		// 		holdingRegs[MB_CUT_BYPASS] = temp_int;
		// 	break;


		// case MB_BEND1_BYPASS:
		// 	temp_int = stage_bypass[1];
		// 	if(temp_int != holdingRegs[MB_BEND1_BYPASS])
		// 		holdingRegs[MB_BEND1_BYPASS] = temp_int;
		// 	break;

		// case MB_BEND2_BYPASS:
		// 	temp_int = stage_bypass[2];
		// 	if(temp_int != holdingRegs[MB_BEND2_BYPASS])
		// 		holdingRegs[MB_BEND2_BYPASS] = temp_int;
		// 	break;

		// case MB_LASER_BYPASS:
		// 	temp_int = stage_bypass[3];
		// 	if(temp_int != holdingRegs[MB_LASER_BYPASS])
		// 		holdingRegs[MB_LASER_BYPASS] = temp_int;
		// 	break;

		// case MB_EJECT_BYPASS:
		// 	temp_int = stage_bypass[4];
		// 	if(temp_int != holdingRegs[MB_EJECT_BYPASS])
		// 		holdingRegs[MB_EJECT_BYPASS] = temp_int;
		// 	break;

		// case MB_MODE_SELECTION:
		// 	temp_int = (unsigned int)(mode_selection);
		// 	if(temp_int != holdingRegs[MB_MODE_SELECTION])
		// 		holdingRegs[MB_MODE_SELECTION] = temp_int;
		// 	break;

		// case MB_BEND_INDIVIDUALLY:
		// 	temp_int = (unsigned int)(bend_individually);
		// 	if(temp_int != holdingRegs[MB_BEND_INDIVIDUALLY]){
		// 		holdingRegs[MB_BEND_INDIVIDUALLY] = temp_int;
		// 	}
		// 	break;

		case MB_LED_STATUS:
			led_status = 0;
			if(SM_START == sm_operation.SM_started()){
				led_status = (led_status |(1<<0));
			}
			else{
				led_status = (led_status &~(1<<0)); 
			}
			
			if(stepper_get_running_motor(MOTOR0) != MOTOR_STOPPED){
				led_status = (led_status |(1<<1));
			}
			else{
				led_status = (led_status &~(1<<1));
			}
			
			if(stepper_get_running_motor(MOTOR1) != MOTOR_STOPPED){
				led_status = (led_status |(1<<2));
			}
			else{
				led_status = (led_status &~(1<<2));
			}

			if(current_op_status[0]){
				led_status = (led_status |(1<<3));
			}
			else{
				led_status = (led_status &~(1<<3));
			}

			if(current_op_status[1]){
				led_status = (led_status |(1<<4));
			}
			else{
				led_status = (led_status &~(1<<4));
			}

			if(current_op_status[2]){
				led_status = (led_status |(1<<5));
			}
			else{
				led_status = (led_status &~(1<<5));
			}

			if(current_op_status[3]){
				led_status = (led_status |(1<<6));
			}
			else{
				led_status = (led_status &~(1<<6));
			}

			if(current_op_status[4]){
				led_status = (led_status |(1<<7));
			}
			else{
				led_status = (led_status &~(1<<7));
			}

			if(stage_bypass[0]){
				led_status = (led_status |(1<<8));
			}
			else{
				led_status = (led_status &~(1<<8));
			}

			if(stage_bypass[1]){
				led_status = (led_status |(1<<9));
			}
			else{
				led_status = (led_status &~(1<<9));
			}

			if(stage_bypass[2]){
				led_status = (led_status |(1<<10));
			}
			else{
				led_status = (led_status &~(1<<10));
			}

			if(stage_bypass[3]){
				led_status = (led_status |(1<<11));
			}
			else{
				led_status = (led_status &~(1<<11));
			}

			if(stage_bypass[4]){
				led_status = (led_status |(1<<12));
			}
			else{
				led_status = (led_status &~(1<<12));
			}

			if(mode_selection){
				led_status = (led_status |(1<<13));
			}
			else{
				led_status = (led_status &~(1<<13));
			}

			if(bend_individually){
				led_status = (led_status |(1<<14));
			}
			else{
				led_status = (led_status &~(1<<14));
			}
			temp_int = (unsigned int)led_status;
			if(temp_int != holdingRegs[MB_LED_STATUS]){
				holdingRegs[MB_LED_STATUS] = led_status;
			}
			break;
		default:
			break;
	}
	return 1;

}


int apply_modbus_values(int modbus_cnt){	
	unsigned int temp_int=0;
	
	switch(modbus_cnt){

		case MB_MOTOR_SPD_1:{
			a.u_val = ((holdingRegs[MB_MOTOR_SPD_1+1]<<16) | (holdingRegs[MB_MOTOR_SPD_1] & 0x0000FFFF));
			motor_speed_1 = a.f_val;
			
			flash_write_now = 1;
			break;
		}

		case MB_MOTOR_DIA_1:{
			a.u_val = ((holdingRegs[MB_MOTOR_DIA_1+1]<<16) | (holdingRegs[MB_MOTOR_DIA_1] & 0x0000FFFF));
			motor_diameter_1 = a.f_val;
			
			flash_write_now = 1;
			break;
		}

		case MB_MOTOR_ACC_1:{
			temp_int = (unsigned int)motor_accel_1*1000;
			if(temp_int != holdingRegs[MB_MOTOR_ACC_1]){
				motor_accel_1 = (float)(holdingRegs[MB_MOTOR_ACC_1]/1000.0);
				// printf("\n motor_accel_1 =%f",motor_accel_1);
				flash_write_now = 1;
			}

			break;
		}


		case MB_PLAY_DIST_1:{
			a.u_val = ((holdingRegs[MB_PLAY_DIST_1+1]<<16) | (holdingRegs[MB_PLAY_DIST_1] & 0x0000FFFF));
			play_distance_1 = a.f_val;
			
			flash_write_now = 1;
			break;
		}


		case MB_GEAR_NUM_1:{
			temp_int = (unsigned int)gearratio_num_1;
			if(temp_int != holdingRegs[MB_GEAR_NUM_1]){
				gearratio_num_1 = holdingRegs[MB_GEAR_NUM_1];
				// printf("\n gearratio_num_1 =%d",gearratio_num_1);
				flash_write_now = 1;
			}

			break;
		}

		case MB_GEAR_DEN_1:{
			temp_int = (unsigned int)gearratio_den_1;
			if(temp_int != holdingRegs[MB_GEAR_DEN_1]){
				gearratio_den_1 = holdingRegs[MB_GEAR_DEN_1];
				// printf("\n gearratio_den_1 =%d",gearratio_den_1);
				flash_write_now = 1;
			}

			break;
		}


		case MB_JOG_DIST_1:{
			a.u_val = ((holdingRegs[MB_JOG_DIST_1+1]<<16) | (holdingRegs[MB_JOG_DIST_1] & 0x0000FFFF));
			jog_distance_1 = a.f_val;
			
			flash_write_now = 1;
			break;
		}


		case MB_JOG_SPEED_1:{
			a.u_val = ((holdingRegs[MB_JOG_SPEED_1+1]<<16) | (holdingRegs[MB_JOG_SPEED_1] & 0x0000FFFF));
			jog_speed_1 = a.f_val;
			
			flash_write_now = 1;
			break;
		}




		case MB_MOTOR_SPD_2:{
			a.u_val = ((holdingRegs[MB_MOTOR_SPD_2+1]<<16) | (holdingRegs[MB_MOTOR_SPD_2] & 0x0000FFFF));
			motor_speed_2 = a.f_val;
			
			flash_write_now = 1;
			break;
		}

		case MB_MOTOR_DIA_2:{
			a.u_val = ((holdingRegs[MB_MOTOR_DIA_2+1]<<16) | (holdingRegs[MB_MOTOR_DIA_2] & 0x0000FFFF));
			motor_diameter_2 = a.f_val;
			
			flash_write_now = 1;
			break;
		}

		case MB_MOTOR_ACC_2:{
			temp_int = (unsigned int)motor_accel_2*1000;
			if(temp_int != holdingRegs[MB_MOTOR_ACC_2]){
				motor_accel_2 = (float)(holdingRegs[MB_MOTOR_ACC_2]/1000.0);
				// printf("\n motor_accel_2 =%f",motor_accel_2);
				flash_write_now = 1;
			}
			break;
		}


		case MB_PLAY_DIST_2:{
			a.u_val = ((holdingRegs[MB_PLAY_DIST_2+1]<<16) | (holdingRegs[MB_PLAY_DIST_2] & 0x0000FFFF));
			play_distance_2 = a.f_val;
			
			flash_write_now = 1;
			break;
		}


		case MB_GEAR_NUM_2:{
			temp_int = (unsigned int)gearratio_num_2;
			if(temp_int != holdingRegs[MB_GEAR_NUM_2]){
				gearratio_num_2 = holdingRegs[MB_GEAR_NUM_2];
				// printf("\n gearratio_num_2 =%d",gearratio_num_2);
				flash_write_now = 1;
			}

			break;
		}

		case MB_GEAR_DEN_2:{
			temp_int = (unsigned int)gearratio_den_2;
			if(temp_int != holdingRegs[MB_GEAR_DEN_2]){
				gearratio_den_2 = holdingRegs[MB_GEAR_DEN_2];
				// printf("\n gearratio_den_2 =%d",gearratio_den_2);
				flash_write_now = 1;
			}

			break;
		}


		case MB_JOG_DIST_2:{
			a.u_val = ((holdingRegs[MB_JOG_DIST_2+1]<<16) | (holdingRegs[MB_JOG_DIST_2] & 0x0000FFFF));
			jog_distance_2 = a.f_val;
			
			flash_write_now = 1;
			break;
		}


		case MB_JOG_SPEED_2:{
			a.u_val = ((holdingRegs[MB_JOG_SPEED_2+1]<<16) | (holdingRegs[MB_JOG_SPEED_2] & 0x0000FFFF));
			jog_speed_2 = a.f_val;
			
			flash_write_now = 1;
			break;
		}


		case MB_JOG_ACC:{
			temp_int = (unsigned int)jog_accel*1000;
			if(temp_int != holdingRegs[MB_JOG_ACC]){
				motor_accel_2 = (float)(holdingRegs[MB_JOG_ACC]/1000.0);
				// printf("\n motor_accel_2 =%f",motor_accel_2);
				flash_write_now = 1;
			}

			break;
		}


		case MB_CYCLE_START_DELAY:{
			temp_int = (unsigned int)cycle_start_delay;
			if(temp_int != holdingRegs[MB_CYCLE_START_DELAY]){
				cycle_start_delay = (uint32_t)(holdingRegs[MB_CYCLE_START_DELAY]);
				// printf("\n cycle_start_delay =%d",cycle_start_delay);	
				flash_write_now = 1;
			}

			break;
		}

		case MB_CUT_DELAY:{
			temp_int = (unsigned int)cut_delay;
			if(temp_int != holdingRegs[MB_CUT_DELAY]){
				cut_delay = (uint32_t)(holdingRegs[MB_CUT_DELAY]);
				// printf("\n cut_delay =%d",cut_delay);
				flash_write_now = 1;
			}

			break;
		}

		case MB_BEND1_DELAY:{
			temp_int = (unsigned int)bend1_delay;
			if(temp_int != holdingRegs[MB_BEND1_DELAY]){
				bend1_delay = (uint32_t)(holdingRegs[MB_BEND1_DELAY]);
				// printf("\n bend1_delay =%d",bend1_delay);	
				flash_write_now = 1;
			}

			break;
		}

		case MB_BEND2_DELAY:{
			temp_int = (unsigned int)bend2_delay;
			if(temp_int != holdingRegs[MB_BEND2_DELAY]){
				bend2_delay = (uint32_t)(holdingRegs[MB_BEND2_DELAY]);
				// printf("\n bend2_delay =%d",bend2_delay);
				flash_write_now = 1;
			}

			break;
		}

		case MB_LASER_ON_DELAY:{
			temp_int = (unsigned int)laser_on_delay;
			if(temp_int != holdingRegs[MB_LASER_ON_DELAY]){
				laser_on_delay = (uint32_t)(holdingRegs[MB_LASER_ON_DELAY]);
				// printf("\n laser_on_delay =%d",laser_on_delay);
				flash_write_now = 1;
			}

			break;
		}

		case MB_LASER_OFF_DELAY:{
			temp_int = (unsigned int)laser_off_delay;
			if(temp_int != holdingRegs[MB_LASER_OFF_DELAY]){
				laser_off_delay = (uint32_t)(holdingRegs[MB_LASER_OFF_DELAY]);
				// printf("\n laser_off_delay =%d",laser_off_delay);
				flash_write_now = 1;
			}

			break;
		}

		case MB_EJECT_DELAY:{
			temp_int = (unsigned int)eject_delay;
			if(temp_int != holdingRegs[MB_EJECT_DELAY]){
				eject_delay = (uint32_t)(holdingRegs[MB_EJECT_DELAY]);
				// printf("\n eject_delay =%d",eject_delay);
				flash_write_now = 1;
			}

			break;
		}

		case MB_LASER_NUM:{
			temp_int = (unsigned int)no_of_laser;
			if(temp_int != holdingRegs[MB_LASER_NUM]){
				no_of_laser = (uint8_t)(holdingRegs[MB_LASER_NUM]);
				// printf("\n no_of_laser =%d",no_of_laser);
			}

			flash_write_now = 1;
			break;
		}


		case MB_DRIVE_PPR:{
			temp_int = (uint32_t)drive_PPR;
			if(temp_int != holdingRegs[MB_DRIVE_PPR]){
				drive_PPR = (uint32_t)(holdingRegs[MB_DRIVE_PPR]);
				// printf("\n drive_PPR =%d",drive_PPR);
			}

			flash_write_now = 1;
			break;
		}

		case MB_BATCH_COUNT:{
			a.u_val = ((holdingRegs[MB_BATCH_COUNT+1]<<16) | (holdingRegs[MB_BATCH_COUNT] & 0x0000FFFF));
			batch_count = a.u_val;
			
			flash_write_now = 1;
			break;
		}

		case MB_JOG_TYPE:{
			temp_int = (unsigned int)jog_type;
			if(temp_int != holdingRegs[MB_JOG_TYPE]){
				jog_type = (uint8_t)(holdingRegs[MB_JOG_TYPE]);
				// printf("\n jog_type =%d",jog_type);
				flash_write_now = 1;
			}

			break;
		}


		case MB_OP_STATE:{
			temp_int = (unsigned int)op_state;
			if(temp_int != holdingRegs[MB_OP_STATE]){
				op_state = (uint8_t)(holdingRegs[MB_OP_STATE]);
				// printf("\n op_state =%d",op_state);
				flash_write_now = 1;
			}

			break;
		}


		case MB_MOTOR_DIR_1:{
			temp_int = (unsigned int)motor_dir_1;
			if(temp_int != holdingRegs[MB_MOTOR_DIR_1]){
				motor_dir_1 = (uint8_t)(holdingRegs[MB_MOTOR_DIR_1]);
				// printf("\n motor_dir_1 =%d",motor_dir_1);
				flash_write_now = 1;
			}

			break;
		}


		case MB_MOTOR_DIR_2:{
			temp_int = (unsigned int)motor_dir_2;
			if(temp_int != holdingRegs[MB_MOTOR_DIR_2]){
				motor_dir_2 = (uint8_t)(holdingRegs[MB_MOTOR_DIR_2]);
				// printf("\n motor_dir_2 =%d",motor_dir_2);
				flash_write_now = 1;
			}

			break;
		}


		case MB_BYPASS:{
			hmi_bypass_reg = (uint32_t) (holdingRegs[MB_BYPASS]);
			// printf("\n hmi_bypass_reg =%d",hmi_bypass_reg);
			menu_hmi_bypass(hmi_bypass_reg);
			hmi_bypass_reg = 0;
			holdingRegs[MB_HMI_EXECUTE] = 0;
			// flash_write_now = 1;
			break;
		}


		// case MB_BEND1_BYPASS:{
		// 	temp_int = (unsigned int)stage_bypass[1];
		// 	if(temp_int != holdingRegs[MB_BEND1_BYPASS])
		// 		stage_bypass[1] = (uint8_t)(holdingRegs[MB_BEND1_BYPASS]);

		// 	flash_write_now = 1;
		// 	break;
		// }


		// case MB_BEND2_BYPASS:{
		// 	temp_int = (unsigned int)stage_bypass[2];
		// 	if(temp_int != holdingRegs[MB_BEND2_BYPASS])
		// 		stage_bypass[2] = (uint8_t)(holdingRegs[MB_BEND2_BYPASS]);

		// 	flash_write_now = 1;
		// 	break;
		// }

		// case MB_LASER_BYPASS:{
		// 	temp_int = (unsigned int)stage_bypass[3];
		// 	if(temp_int != holdingRegs[MB_LASER_BYPASS])
		// 		stage_bypass[3] = (uint8_t)(holdingRegs[MB_LASER_BYPASS]);

		// 	flash_write_now = 1;
		// 	break;
		// }

		// case MB_EJECT_BYPASS:{
		// 	temp_int = (unsigned int)stage_bypass[4];
		// 	if(temp_int != holdingRegs[MB_EJECT_BYPASS])
		// 		stage_bypass[4] = (uint8_t)(holdingRegs[MB_EJECT_BYPASS]);

		// 	flash_write_now = 1;
		// 	break;
		// }

		// case MB_MODE_SELECTION:{
		// 	temp_int = (unsigned int)mode_selection;
		// 	if(temp_int != holdingRegs[MB_MODE_SELECTION])
		// 		mode_selection = (uint8_t)(holdingRegs[MB_MODE_SELECTION]);

		// 	flash_write_now = 1;
		// 	break;
		// }

		// case MB_BEND_INDIVIDUALLY:{
		// 	temp_int = (unsigned int)bend_individually;
		// 	if(temp_int != holdingRegs[MB_BEND_INDIVIDUALLY]){
		// 		bend_individually = (uint8_t)(holdingRegs[MB_BEND_INDIVIDUALLY]);
		// 		// printf("\n bend_individually =%d",bend_individually);
		// 		flash_write_now = 1;
		// 	}

		// 	break;
		// }

		case MB_HMI_EXECUTE:{
			hmi_execute_reg = (uint32_t) (holdingRegs[MB_HMI_EXECUTE]);
			menu_hmi_execute(hmi_execute_reg);

			// hmi_execute_reg = 0;
			break;
		}

		case MB_OPERATE_MANUAL:{		

			// if(holdingRegs[MB_OPERATE_MANUAL]){
			// 	if(SM_START != sm_manual.SM_started()){
			// 		sm_manual.SM_start();
			// 	}
			// }else{
			// 	if(SM_START == sm_manual.SM_started()){
			// 		sm_manual.stop_now();
			// 	}
			// }
			if(holdingRegs[MB_OPERATE_MANUAL]){

				if(SM_START != sm_manual.SM_started()){
					sm_manual.SM_start();
					// printf("\n M start");
				}else{
					sm_manual.stop_now();
					// printf("\n M stop");
				}		

			}
			
			break;
		}
	}
	return 1;
}

int menu_hmi_execute(uint32_t hmi_key){

	switch(hmi_key){
		case HMI_START:
			if(SM_START != sm_operation.SM_started()){
				if(product_count < batch_count){
					sm_operation.SM_start();
				}
			}
			break;

		case HMI_STOP:				
			if(SM_START == sm_operation.SM_started()){
				stop_machine = 1;
			}
		break;

		case HMI_COUNTER_RESET:				
			product_count = 0;				
			break;

		case HMI_FWD_JOG_1:				
		case HMI_REV_JOG_1:				
			if((SM_START != sm_manual.SM_started()) && (SM_START != sm_operation.SM_started())){
				if(hmi_key == HMI_FWD_JOG_1){
					sm_jog_m1.direction=JOG_FWD;
				}
				else{
					sm_jog_m1.direction=JOG_REV;
					// printf("\n jog rev");
				}	
				sm_jog_m1.SM_start();
				
			}
			break;

		case HMI_FWD_JOG_2:				
		case HMI_REV_JOG_2:				
			if((SM_START != sm_manual.SM_started()) && (SM_START != sm_operation.SM_started())){
				if(hmi_key == HMI_FWD_JOG_2){
					sm_jog_m2.direction=JOG_FWD;
				}
				else{
					sm_jog_m2.direction=JOG_REV;
				}	
				sm_jog_m2.SM_start();
				
			}
			break;
		
		
		case HMI_NEXT_STAGE:				
			if(SM_START == sm_manual.SM_started()){
				next_stage = 1;
			}
			break;

		case HMI_EXIT_IO_TEST:
		
			CUT_OFF;
			BEND1_OFF;
			BEND2_OFF;
			LASER_OFF;
			EJECT_OFF;
			op_toggle[0] = 0;
			op_toggle[1] = 0;
			op_toggle[2] = 0;
			op_toggle[3] = 0;
			op_toggle[4] = 0;
			break;

		case HMI_CUT_TOGGLE:
			if(!op_toggle[0]){
				op_toggle[0] = 1;
				CUT_ON;
			}else{
				op_toggle[0] = 0;
				CUT_OFF;
			}
			break;

		case HMI_BEND1_TOGGLE:
			if(!op_toggle[1]){
				op_toggle[1] = 1;
				BEND1_ON;
			}else{
				op_toggle[1] = 0;
				BEND1_OFF;
			}
			break;

		case HMI_BEND2_TOGGLE:
			if(!op_toggle[2]){
				op_toggle[2] = 1;
				BEND2_ON;
			}else{
				op_toggle[2] = 0;
				BEND2_OFF;
			}
			break;

		case HMI_LASER_TOGGLE:
			if(!op_toggle[3]){
				op_toggle[3] = 1;
				LASER_ON;
			}else{
				op_toggle[3] = 0;
				LASER_OFF;
			}
			break;
		
		case HMI_EJECT_TOGGLE:
			if(!op_toggle[4]){
				op_toggle[4] = 1;
				EJECT_ON;
			}else{
				op_toggle[4] = 0;
				EJECT_OFF;
			}
			break;

	}
	return 1;
}

int menu_hmi_bypass(uint32_t hmi_key){
	// printf("\n hmi_key= %d",hmi_key);
	switch(hmi_key){

		case MB_CUT_BYPASS:
			if(stage_bypass[0]){
				stage_bypass[0] = 0;
				// printf("\n cut Enb");
			}else{
				stage_bypass[0] = 1;
				// printf("\n cut dis");
			}
			break;

		case MB_BEND1_BYPASS:

			if(stage_bypass[1]){
				stage_bypass[1] = 0;
				// printf("\n B1 Enb");
			}else{
				stage_bypass[1] = 1;
				// printf("\n B1 dis");
			}				
		break;

		case MB_BEND2_BYPASS:

			if(stage_bypass[2]){
				stage_bypass[2] = 0;
				// printf("\n b2 Enb");
			}else{
				stage_bypass[2] = 1;
				// printf("\n b2 dis");
			}				
			break;

		case MB_LASER_BYPASS:

			if(stage_bypass[3]){
				stage_bypass[3] = 0;
				// printf("\n laser Enb");
			}else{
				stage_bypass[3] = 1;
				// printf("\n laser dis");
			}				
			break;

		case MB_EJECT_BYPASS:

			if(stage_bypass[4]){
				stage_bypass[4] = 0;
				// printf("\n eject Enb");
			}else{
				stage_bypass[4] = 1;
				// printf("\n eject dis");
			}				
			break;
		
		
		case HMI_MODE_SELECT:
			if(SM_START != sm_operation.SM_started()){
				
				if(mode_selection){
					mode_selection = 0;

				}else{
					mode_selection = 1;
				}
			}
			break;

		case HMI_SAVE_BYPASS:
			flash_write_now = 1;
			break;

		case HMI_MANUAL_OPERATION :
			// if(SM_START != sm_manual.SM_started()){
			// 	sm_manual.SM_start();
			// 	printf("\n M start");
			// }else{
			// 	sm_manual.stop_now();
			// 	printf("\n M stop");
			// }		
			break;

		case HMI_BEND_INDIVIDUALLY:

			if(bend_individually){
				bend_individually = 0;

			}else{
				bend_individually = 1;
			}
			

			break;
	}
	return 1;
}
