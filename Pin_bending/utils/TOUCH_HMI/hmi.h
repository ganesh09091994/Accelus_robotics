#ifndef _HMI_H_
#define _HMI_H_

#include "global.h" 
extern "C"{
	#include "debug_frmwrk.h"
}

#define MOTOR_STOPPED 	0
int detect_val_change(unsigned int, unsigned int, int, int, float);
int set_holding_registers_values(int modbus_cnt);
int apply_modbus_values(int modbus_cnt);
int menu_hmi_execute(uint32_t hmi_key);
int menu_hmi_bypass(uint32_t hmi_key);
extern uint8_t third_jog_val;
#endif
