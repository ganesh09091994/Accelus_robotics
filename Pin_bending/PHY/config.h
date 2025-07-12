#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "board.h"

typedef struct{
	char port_num;
	char pin_num;
	char io;		// tells if this pin is configured as an input=1 or output=0..
}GPIOPin;

#define GPIOpin_t GPIOPin

#define PI 3.141592654

#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))


#define CURRENT_CONFIG_MAJOR_VERSION 	1
#define CURRENT_CONFIG_MINOR_VERSION 	0

#define LARGE_DISTANCE	0x7FFFFFFFL
#define HOLD_REG_MAX 	200


#define AUTO_MODE 			0
#define SEMI_AUTO_MODE 		1

#define CUT_STAGE 			0
#define BEND1_STAGE 		1
#define BEND2_STAGE 		2
#define LASER_STAGE 		3
#define EJECT_STAGE 		4

// Motor Numbers..
#define MOTOR0	0
#define MOTOR1	1
#define MOTOR2	2
#define MOTOR3	3


#define MODBUS_AXIS_DATA_OFFSET 	1000

// Defines to keep logical information symbolic go here
#define    HIGH          (1)
#define    LOW           (0)
#define    ON            (1)
#define    OFF           (0)
#define    OUTPUT        (0)
#define    INPUT         (1)
#define    PORTA         (0)
#define    PORTB         (1)

#define CUT_OUTPUT 			8	
#define BEND1_OUTPUT 		9
#define BEND2_OUTPUT 		10
#define LASER_OUTPUT 		11
#define EJECT_OUTPUT 		12

// 8
#define CUT_ON \
	io_set_output(CUT_OUTPUT,(!op_state)); \
	current_op_status[0] = 1;

#define CUT_OFF	\
	io_set_output(CUT_OUTPUT,op_state); \
	current_op_status[0] = 0;

//9
#define BEND1_ON	\
	io_set_output(BEND1_OUTPUT,(!op_state));	\
	current_op_status[1] = 1;

#define BEND1_OFF	\
	io_set_output(BEND1_OUTPUT,op_state);	\
	current_op_status[1] = 0;
//10
#define BEND2_ON \
	io_set_output(BEND2_OUTPUT,(!op_state));	\
	current_op_status[2] = 1;

#define BEND2_OFF \
	io_set_output(BEND2_OUTPUT,op_state);		\
	current_op_status[2] = 0;

//11
#define LASER_ON \
	io_set_output(LASER_OUTPUT,(!op_state));	\
	current_op_status[3] = 1;

#define LASER_OFF \
	io_set_output(LASER_OUTPUT,op_state);	 \
	current_op_status[3] = 0;

//12
#define EJECT_ON \
	io_set_output(EJECT_OUTPUT,(!op_state));	 \
	current_op_status[4] = 1;

#define EJECT_OFF \
	io_set_output(EJECT_OUTPUT,op_state);	 \
	current_op_status[4] = 0;


enum DELAY_TYPES {
	DELAY_HEARTBEAT=0x00,
	DELAY_KEY_TASK_SCAN,
	DELAY_KEY_TASK_GETKEY,
	DELAY_DISPLAY_UPDATE,
	DELAY_SWITCH_UPDATE,
	DELAY_ANIMATE_DISPLAY,
	DELAY_IO_UPDATE,	
	DELAY_BLINK_DIGIT,
	DELAY_CYCLE_START,
	DELAY_CUT,
	DELAY_BEND1,
	DELAY_BEND2,
	DELAY_LASER_ON,
	DELAY_LASER_OFF,
	DELAY_EJECT,
	DELAY_TOTAL_NUM,
};


enum MODBUS_REGISTER_ADDRESS_AXIS{
	MB_MOTOR_SPD_1 = 0,//
	MB_MOTOR_DIA_1 = 2,//
	MB_MOTOR_ACC_1 = 4,//
	MB_PLAY_DIST_1 = 5,//
	MB_GEAR_NUM_1 = 7,//
	MB_GEAR_DEN_1 = 8,//
	MB_JOG_DIST_1 = 9,//
	MB_JOG_SPEED_1 = 11,//
	MB_MOTOR_SPD_2 = 13,//
	MB_MOTOR_DIA_2 = 15,//
	MB_MOTOR_ACC_2 = 17,//
	MB_PLAY_DIST_2 = 18,
	MB_GEAR_NUM_2 = 20,
	MB_GEAR_DEN_2 = 21,
	MB_JOG_DIST_2 = 22,
	MB_JOG_SPEED_2 = 24,
	MB_JOG_ACC = 26,
	MB_CYCLE_START_DELAY = 27,
	MB_CUT_DELAY = 28,
	MB_BEND1_DELAY = 29,
	MB_BEND2_DELAY = 30,
	MB_LASER_ON_DELAY = 31,
	MB_LASER_OFF_DELAY = 32,
	MB_EJECT_DELAY = 33,
	MB_LASER_NUM = 34,
	MB_DRIVE_PPR = 35,
	MB_PRODUCT_COUNT = 36,
	MB_BATCH_COUNT = 38,
	MB_JOG_TYPE = 40,
	MB_OP_STATE = 41,
	MB_MOTOR_DIR_1 = 42,
	MB_MOTOR_DIR_2 = 43,
	MB_BYPASS = 44,
	// MB_BEND1_BYPASS = 45,
	// MB_BEND2_BYPASS = 46,
	// MB_LASER_BYPASS = 47,
	// MB_EJECT_BYPASS = 48,
	// MB_MODE_SELECTION = 49,
	// MB_BEND_INDIVIDUALLY = 50,
	MB_HMI_EXECUTE = 51,
	MB_OPERATE_MANUAL = 52,
	MB_LED_STATUS = 53,
	MB_DUMMY = 55,
};



enum MENU_PLAY_HMI {
	HMI_START = 1,			//0
	HMI_STOP  = 2,			//1
	
	HMI_COUNTER_RESET = 4,		//2
	HMI_FWD_JOG_1 = 8,		//3
	HMI_REV_JOG_1 = 16,		//4
	
	HMI_FWD_JOG_2 = 32,		//5
	HMI_REV_JOG_2 = 64,		//6
	
	HMI_NEXT_STAGE = 128,	//7
	HMI_EXIT_IO_TEST = 256,	//8
	

	HMI_CUT_TOGGLE = 512,	//9
	HMI_BEND1_TOGGLE = 1024,	//10
	
	HMI_BEND2_TOGGLE = 2048,		//11
	HMI_LASER_TOGGLE = 4096,		//12
	HMI_EJECT_TOGGLE = 8192,	//13

	HMI_JOG_STATE = 16384,	//14
	
	MENU_PLAY_SIZE,
};

enum MENU_BYPASS_HMI {

	MB_CUT_BYPASS = 1,			//0
	MB_BEND1_BYPASS  = 2,		//1
	MB_BEND2_BYPASS = 4,		//2
	MB_LASER_BYPASS = 8,		//3
	MB_EJECT_BYPASS = 16,		//4
	
	HMI_MODE_SELECT = 32,		//5
	HMI_SAVE_BYPASS = 64,		//6
	
	HMI_MANUAL_OPERATION = 128,	//7
	HMI_BEND_INDIVIDUALLY = 256,	//8

};
enum INPUT_TYPES {
	ZERO =0x00,
	ONE,
	TWO,
	THREE,
	TOTAL_SENSOR_NUMBER=7,
};

#endif
