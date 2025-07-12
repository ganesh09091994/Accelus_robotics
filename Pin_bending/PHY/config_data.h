#ifndef _CONFIG_DATA_H_
#define _CONFIG_DATA_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "global.h"

extern "C"{
	#include "i2c_eeprom.h"
	#include "debug_frmwrk.h"
}

#define CONFIG_VERSION_OK	0

typedef struct {
	uint8_t	fw_major_version;
	uint8_t	fw_minor_version;

	float c_jog_distance_1;
	float c_jog_speed_1;
	float c_jog_accel;
	uint8_t c_gearratio_num_1;
	uint8_t c_gearratio_den_1;

	float c_motor_speed_1;
	float c_motor_diameter_1;
	float c_motor_accel_1;
	float c_play_distance_1;

	float c_jog_distance_2;
	float c_jog_speed_2;
	uint8_t c_gearratio_num_2;
	uint8_t c_gearratio_den_2;

	float c_motor_speed_2;
	float c_motor_diameter_2;
	float c_motor_accel_2;
	float c_play_distance_2;


	uint32_t c_cut_delay;
	uint32_t c_bend1_delay;
	uint32_t c_bend2_delay;
	uint32_t c_eject_delay;

	uint8_t c_no_of_laser;

	uint32_t c_drive_PPR;
	
	uint8_t c_jog_type;
	uint8_t c_op_state;
	uint8_t c_motor_dir_1;
	uint8_t c_motor_dir_2;
	uint32_t c_cycle_start_delay;
	uint32_t c_laser_on_delay;
	uint32_t c_laser_off_delay;
	uint8_t c_stage_bypass[5];
	uint8_t c_mode_selection;
	uint8_t c_bend_individually;
	uint32_t c_batch_count;

}EepromStorage;

/* read config data from EEPROM...*/
int write_objects_eeprom(	int16_t , 	// address of the eeprom location..
							void * ,	// object to be stored..
							int 		// size of the object to be stored..
						);
						
int read_objects_eeprom(int16_t ,	// address of the eeprom location..
						void *,		// object to be stored..
						int 		// size of the object to be stored..
						);
						
int read_config_version();
int read_config_data(void);
int read_config_version(uint8_t &, uint8_t &);

int write_config_data(void);

int config_set_defaults(uint8_t , uint8_t );

int write_Sequence_to_EEPROM(char *, int , int );
int read_Sequence_from_EEPROM(char *, int , int );

#endif
