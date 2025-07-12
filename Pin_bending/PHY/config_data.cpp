#include "config_data.h"
#include "delay_timer.h"

EepromStorage eepromStorage;

int write_objects_eeprom(int16_t addr, void *obj, int size){
	// we can write 32 bytes at a time..
	int ret=0;
	int id=0;
	int length=0;
	int arr_idx=0;
	int chunkSize=PAGE_BUFFER_INT;
	float buf_size=PAGE_BUFFER_FLOAT;
	int csize = PAGE_BUFFER_INT;
		
	uint8_t *ptr=(uint8_t *) obj;	
	length=ceil((double)(size/buf_size));	
	for(id=0;id<length;id++){
		arr_idx=id*chunkSize;
		csize = chunkSize;
		if (size - arr_idx < chunkSize){
			csize = size - arr_idx;
		}
		ret=i2cEEPROM_write8(addr,&(ptr[arr_idx]),csize);	
		addr+=chunkSize;
		// addr++;
		if(ret!=0){
			printf("WRITE ERROR\n");
		}
		my_delay(50);
	}	
	I2CDeinit();
	return 1;
}

int read_objects_eeprom(int16_t addr, void *obj, int size){	
	int ret=0;
	int id=0;
	int length=0;
	int arr_idx=0;
	int chunkSize=PAGE_BUFFER_INT;
	float buf_size=PAGE_BUFFER_FLOAT;
	int csize = PAGE_BUFFER_INT;
	
	uint8_t *ptr=(uint8_t *) obj;
	length=ceil((double)(size/buf_size));
	for(id=0;id<length;id++){
		arr_idx=id*chunkSize;
		csize = chunkSize;
		if (size - arr_idx < chunkSize){
			csize = size - arr_idx;
		}
		ret=i2cEEPROM_read8(addr,&ptr[arr_idx],csize);
		addr+=chunkSize;
		// addr++;
		if(ret!=0){
			printf("READ ERROR\n");
		}	
		// my_delay(50);
	}	
	I2CDeinit();
	return 1;
}

int read_config_version(){	
	int ret=0;
	int id=0;
	int length=0;
	int arr_idx=0;
	int chunkSize=PAGE_BUFFER_INT;
	float buf_size=PAGE_BUFFER_FLOAT;
	int csize = PAGE_BUFFER_INT;
	int16_t addr=0;
	EepromStorage obj;
	int size=sizeof(EepromStorage);
	
	uint8_t *ptr=(uint8_t *) (&obj);
	length=ceil((double)(size/buf_size));
	for(id=0;id<length;id++){
		arr_idx=id*chunkSize;
		csize = chunkSize;
		if (size - arr_idx < chunkSize){
			csize = size - arr_idx;
		}
		ret=i2cEEPROM_read8(addr,&ptr[arr_idx],csize);
		addr+=chunkSize;
		// addr++;		
		if(ret!=0){
			printf("READ ERROR\n");
		}
	}
	I2CDeinit();
	EepromStorage *es= &obj;
	if(fw_major_version!=es->fw_major_version)
		return 1;
	if(fw_minor_version!=es->fw_minor_version)
		return 2;	
	return 0;
}

int read_config_version(uint8_t &major, uint8_t &minor){	
	int ret=0;
	int id=0;
	int length=0;
	int arr_idx=0;
	int chunkSize=PAGE_BUFFER_INT;
	float buf_size=PAGE_BUFFER_FLOAT;
	int csize = PAGE_BUFFER_INT;
	int16_t addr=0;
	EepromStorage obj;
	int size=sizeof(EepromStorage);

	uint8_t *ptr=(uint8_t *) (&obj);
	length=ceil((double)(size/buf_size));
	for(id=0;id<length;id++){
		arr_idx=id*chunkSize;
		csize = chunkSize;
		if (size - arr_idx < chunkSize){
			csize = size - arr_idx;
		}
		ret=i2cEEPROM_read8(addr,&ptr[arr_idx],csize);
		addr+=chunkSize;
		// addr++;
		if(ret!=0){
			printf("READ ERROR\n");
		}		
	}
	I2CDeinit();
	EepromStorage *es= &obj;
	major=es->fw_major_version;
	minor=es->fw_minor_version;
	if(CURRENT_CONFIG_MAJOR_VERSION!=major)
		return 1;
	if(CURRENT_CONFIG_MINOR_VERSION!=minor)
		return 2;	
	return 0;
}

int read_config_data(void){
	EepromStorage eeps;
	
	printf("Reading config data %d\n",sizeof(eeps));
	read_objects_eeprom(0,&eeps,sizeof(eeps));
	
	fw_major_version 		= eeps.fw_major_version;
	fw_minor_version 		= eeps.fw_minor_version;
	
	jog_distance_1 			= eeps.c_jog_distance_1;
	jog_speed_1 			= eeps.c_jog_speed_1;
	jog_accel 				= eeps.c_jog_accel;
	gearratio_num_1 		= eeps.c_gearratio_num_1;
	gearratio_den_1 		= eeps.c_gearratio_den_1;

	motor_speed_1 			= eeps.c_motor_speed_1;
	motor_diameter_1 		= eeps.c_motor_diameter_1;
	motor_accel_1 			= eeps.c_motor_accel_1;
	play_distance_1 		= eeps.c_play_distance_1;

	jog_distance_2 			= eeps.c_jog_distance_2;
	jog_speed_2 			= eeps.c_jog_speed_2;
	gearratio_num_2 		= eeps.c_gearratio_num_2;
	gearratio_den_2 		= eeps.c_gearratio_den_2;

	motor_speed_2 			= eeps.c_motor_speed_2;
	motor_diameter_2 		= eeps.c_motor_diameter_2;
	motor_accel_2 			= eeps.c_motor_accel_2;
	play_distance_2 		= eeps.c_play_distance_2;


	cut_delay 				= eeps.c_cut_delay;
	bend1_delay 			= eeps.c_bend1_delay;
	bend2_delay 			= eeps.c_bend2_delay;
	eject_delay 			= eeps.c_eject_delay;

	no_of_laser 			= eeps.c_no_of_laser;

	drive_PPR 				= eeps.c_drive_PPR;
	
	jog_type				= eeps.c_jog_type;
	// op_state				= eeps.c_op_state;
	motor_dir_1				= eeps.c_motor_dir_1;
	motor_dir_2 			= eeps.c_motor_dir_2;
	cycle_start_delay		= eeps.c_cycle_start_delay;
	laser_on_delay			= eeps.c_laser_on_delay;
	laser_off_delay			= eeps.c_laser_off_delay;
	// mode_selection			= eeps.c_mode_selection;
	// bend_individually		= eeps.c_bend_individually;

	batch_count 			= eeps.c_batch_count;
	// for(int i=0; i<5;i++){
	// 	stage_bypass[i]		 = eeps.c_stage_bypass[i];
	// }


	// copy config_daa into their respective sructs...
	
	return 1;
}

int write_config_data(void){
	
	// copy all data to the config_data struct ..	
	eepromStorage.fw_major_version=fw_major_version;
	eepromStorage.fw_minor_version=fw_minor_version;

	eepromStorage.c_jog_distance_1 			= jog_distance_1;
	eepromStorage.c_jog_speed_1 			= jog_speed_1;
	eepromStorage.c_jog_accel 				= jog_accel;
	eepromStorage.c_gearratio_num_1 		= gearratio_num_1;
	eepromStorage.c_gearratio_den_1 		= gearratio_den_1;

	eepromStorage.c_motor_speed_1 			= motor_speed_1;
	eepromStorage.c_motor_diameter_1 		= motor_diameter_1;
	eepromStorage.c_motor_accel_1 			= motor_accel_1;
	eepromStorage.c_play_distance_1 		= play_distance_1;

	eepromStorage.c_jog_distance_2 			= jog_distance_2;
	eepromStorage.c_jog_speed_2 			= jog_speed_2;
	eepromStorage.c_gearratio_num_2 		= gearratio_num_2;
	eepromStorage.c_gearratio_den_2 		= gearratio_den_2;

	eepromStorage.c_motor_speed_2 			= motor_speed_2;
	eepromStorage.c_motor_diameter_2		= motor_diameter_2;
	eepromStorage.c_motor_accel_2 			= motor_accel_2;
	eepromStorage.c_play_distance_2 		= play_distance_2;


	eepromStorage.c_cut_delay 				= cut_delay;
	eepromStorage.c_bend1_delay 			= bend1_delay;
	eepromStorage.c_bend2_delay 			= bend2_delay;
	eepromStorage.c_eject_delay 			= eject_delay;

	eepromStorage.c_no_of_laser 			= no_of_laser;

	eepromStorage.c_drive_PPR 				= drive_PPR;

	eepromStorage.c_jog_type 				= jog_type;
	// eepromStorage.c_op_state 				= op_state;
	eepromStorage.c_motor_dir_1 			= motor_dir_1;
	eepromStorage.c_motor_dir_2 			= motor_dir_2;
	eepromStorage.c_cycle_start_delay 		= cycle_start_delay;
	eepromStorage.c_laser_on_delay 			= laser_on_delay;
	eepromStorage.c_laser_off_delay 		= laser_off_delay;
	// eepromStorage.c_mode_selection 			= mode_selection;
	// eepromStorage.c_bend_individually 		= bend_individually;
	eepromStorage.c_batch_count 			= batch_count;


	// for(int i=0; i<5;i++){
	// 	eepromStorage.c_stage_bypass[i]		= stage_bypass[i];
	// }


	write_objects_eeprom(0,&eepromStorage,sizeof(eepromStorage));
	// printf("Writing config data %d\n",sizeof(eepromStorage));
	return 1;
}


int config_set_defaults(uint8_t, uint8_t){
	printf("Setting defaults %d\n",TRUE);

	motor_speed_1 = 10.0;
	motor_diameter_1 = 5.0;
	motor_accel_1 = 0.3;
	play_distance_1 = 5.0;
	gearratio_num_1 = 1;
	gearratio_den_1 = 1;

	motor_speed_2 = 10.0;
	motor_diameter_2 = 5.0;
	motor_accel_2 = 0.3;
	play_distance_2 = 5.0;
	gearratio_num_2 = 1;
	gearratio_den_2 = 1;

	jog_distance_1 = 5.0;
	jog_speed_1 = 5.0;
	jog_accel = 0.2;


	jog_distance_2 = 5.0;
	jog_speed_2 = 5.0;

	cut_delay = 500;
	bend1_delay = 500;
	bend2_delay = 500;
	eject_delay = 500;

	no_of_laser = 1;

	drive_PPR = 1600;
	
	jog_type = 0;
	// op_state = 1;
	motor_dir_1 = 0;
	motor_dir_2 = 0;
	cycle_start_delay = 500;
	laser_on_delay = 500;
	laser_off_delay = 500;
	mode_selection = 0;
	bend_individually = 0;

	for(int i=0; i<5;i++){
		stage_bypass[i]		 = 0;
	}
	write_config_data();
	return 1;
}


int write_Sequence_to_EEPROM(char *p, int size, int seq_num){
	// seqnum is the num of the sequence to write to...
	//Start breaking the data into 32byte chunks
	// unsigned int i = 0;
	int id = 0;
	int chunkSize = 32;
	int arr_idx = 0;
	int retval = 0;
	char temp_str[33] = {0};
	char read_str[33] = {0};
	
	unsigned char chunk_size = 32;
	unsigned int seq_eeprom_start_offset = 0;
	unsigned int seq_size = 512;
	unsigned int seq_addr = seq_num*seq_size + seq_eeprom_start_offset;
	// printf("SeqAddr: %d\n",seq_addr);
	
	unsigned int write_addr = seq_addr;
	
	// memset(recvd,'0',sizeof(p));
	// sprintf(I2C_recvd,"%s",s);
	
	for (id = 0;id < 16;id++){
		arr_idx = id *chunkSize;
		memcpy(temp_str,&p[arr_idx],chunk_size);
		temp_str[chunk_size] = '\0';
		// printf("Wr%d:%s\n",id,temp_str);
		write_addr = seq_addr + arr_idx;
		retval = i2cEEPROM_write32(write_addr,temp_str);
		if (retval !=0){			
			// printf("Wrong: %d\n",id);
			break;
		}
		//read it back...
		// my_delay(50);
		i2cEEPROM_read32(write_addr,read_str,chunk_size);
		read_str[chunk_size] = '\0';
		// printf("Rd%d:%s\n",id,read_str);
	}	
	I2CDeinit();
	// my_delay(10);
	return retval;
}

int read_Sequence_from_EEPROM(char *p, int size, int seq_num){
	// seq_num is the num of the seq to load...
	// start getting the data for the sequence in chunks of 32 bytes..
	// unsigned int i = 0;
	int id = 0;
	int chunkSize = 32;
	int arr_idx = 0;
	int retval = 0;
	char recvd_str[32];
	char *data = recvd_str;
	
	unsigned char chunk_size = 32;
	unsigned int seq_eeprom_start_offset = 0;
	unsigned int seq_size = 512;
	unsigned int seq_addr = seq_num*seq_size + seq_eeprom_start_offset;
	
	unsigned int read_addr = seq_addr;
	
	memset(p,'0',size);
	p[0] = '\0';
	for (id = 0;id < 16;id++){
		arr_idx = id *chunkSize;		
		read_addr = seq_addr + arr_idx;
		memset(data,'0',sizeof(recvd_str));
		retval = i2cEEPROM_read32(read_addr,recvd_str,chunk_size); 
		if (retval != 0){			
			// printf("Wrong: %d\n",id);
			break;
		}		
		//read it back...		
		recvd_str[chunk_size] = '\0';
		// printf("Rd%d:%s\n",id,recvd_str);
		strcat(p,recvd_str);
	}
	// for(i=0;i<seq_size;i++){
		// printf("RECVD[%d]: %d\n",i,(s[i])-48);
	// }
	// printf("final string = %s",s);
	I2CDeinit();
	// my_delay(10);
	return retval;
	
}
