//!
/*!
 *Main file 
 * All the various function are called inside the while loop of the main fucntion
 * The various functions are polled using the Timer at regualr preset intervals set at the beginning of the program.
 */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

#include "LPC17xx.h"
#include "ring.h"
#include "syscalls.h"
#include "global.h"
#include "config.h"
#include "delay_timer.h"

#include "SimpleModbusSlave_1769.h"
#include "state_machine.h"
#include "hmi.h"

#include "config_data.h"

#include "SM_Jogging.h"
#include "SM_Operation.h"
#include "SM_Manual.h"
// #include "SM_Gotozero.h"
// #include "SM_Homing.h"
// #include "SM_3R_Design.h"
// #include "SM_MPG.h"

extern "C" {	
	#include "lpc17xx_libcfg.h"
	#include "lpc17xx_ssp.h"
	#include "lpc17xx_spi.h"
	#include "lpc17xx_rtc.h"
	#include "lpc17xx_clkpwr.h"
	#include "lpc17xx_gpio.h"
	#include "lpc17xx_uart.h"
	#include "lpc17xx_pinsel.h"
	#include "debug_frmwrk.h"	
	#include "lpc17xx_nvic.h"
	#include "timer.h"
	#include "multi-steppers.h"
	#include "input_output.h"
	#include "i2c.h"
	#include "i2c_eeprom.h"
	#include "usbSerial.h"
	#include "iap_driver.h"
	#include "qei_test.h"
}

/*!
 *Main file!
 *
*/
/**#########################################################################################*/	
extern "C"{	uint32_t delay_timer_32 = 0; 
int getSavedDataFromEeprom(void);
}

union converter a;

key_code_count kcc;

// design_parameter design_para[TOTAL_LINES];
// axis_para motor_para[TOTAL_AXIS];
int flash_write_now= 0;
char str[35];
uint32_t temp_store;
// float  vel_motor0 ;

unsigned int holdingRegs[HOLD_REG_MAX]; // function 3 and 16 register array
unsigned int holdingRegs_prev[HOLD_REG_MAX]; // function 3 and 16 register array

int baud_rate_select;
uint32_t baud_value;

uint8_t fw_major_version;
uint8_t fw_minor_version;

int modbus_exception_handler(int modbus_ret);
int apply_modbus_values(int modbus_cnt);

int modbus_retVal = 0;

uint32_t product_count = 0;
uint32_t batch_count = 0;

float jog_distance_1 = 5.0;
float jog_speed_1 = 5.0;
float jog_accel = 0.2;
uint8_t gearratio_num_1 = 1;
uint8_t gearratio_den_1 = 1;

float motor_speed_1 = 10.0;
float motor_diameter_1 = 5.0;
float motor_accel_1 = 0.3;
float play_distance_1 = 5.0;

float jog_distance_2 = 5.0;
float jog_speed_2 = 5.0;
uint8_t gearratio_num_2 = 1;
uint8_t gearratio_den_2 = 1;

float motor_speed_2 = 10.0;
float motor_diameter_2 = 5.0;
float motor_accel_2 = 0.3;
float play_distance_2 = 5.0;


uint32_t cut_delay = 500;
uint32_t bend1_delay = 500;
uint32_t bend2_delay = 500;

uint32_t eject_delay = 500;

uint8_t no_of_laser = 1;

uint32_t drive_PPR = 1600;

float motor_ppr_per_mm_1;
float motor_ppr_per_mm_2;

//Remaining
uint8_t jog_type;
uint8_t op_state = 1;
uint8_t motor_dir_1 = 0;
uint8_t motor_dir_2 = 0;
uint32_t cycle_start_delay = 500;
uint32_t laser_on_delay = 500;
uint32_t laser_off_delay = 500;
uint8_t stage_bypass[5];
uint8_t mode_selection = 0;
uint8_t bend_individually;
uint8_t next_stage;
uint8_t stop_machine;
uint8_t current_op_status[5];
/**#########################################################################################*/
SM_MANUAL sm_manual;

SM_OPERATION sm_operation;

SM_Jogging sm_jog_m1(MOTOR0);
SM_Jogging sm_jog_m2(MOTOR1);

int main(void) {
	uint32_t modbus_count=0;
	uint32_t i,cycle_counter;
	static int toggle;	
	/* Initialize debug via UART0
		* – 115200bps
		* – 8 data bit
		* – No parity
		* – 1 stop bit
		* – No flow control
	*/
	// printf("prog execution\n");
	debug_frmwrk_init();
	
	LPC_SC->PCONP |= ( 1 << 15 ); // power up GPIO		
	io_init();
	printf("\nSystemCoreClock = %d Hz\n", SystemCoreClock);
/**#########################################################################################*/		
	dt_set_delay(DELAY_HEARTBEAT, 2000);
	dt_set_delay(DELAY_KEY_TASK_SCAN, 50);		//keyscantask delay
	dt_set_delay(DELAY_KEY_TASK_GETKEY, 100);		//getkey delay	
	dt_set_delay(DELAY_IO_UPDATE, 100);
	dt_set_delay(DELAY_BLINK_DIGIT, 500);
	dt_set_delay(DELAY_ANIMATE_DISPLAY,300);
	dt_set_delay(DELAY_DISPLAY_UPDATE, 50);
	
/**#########################################################################################*/		
#if 1
	if(CONFIG_VERSION_OK != read_config_version(fw_major_version,fw_minor_version)){	
		// now fw_major and fw_minor contain the config version...
		fw_major_version=CURRENT_CONFIG_MAJOR_VERSION;
		fw_minor_version=CURRENT_CONFIG_MINOR_VERSION;
		config_set_defaults(fw_major_version, fw_minor_version);
	}else{	
		// now fw_major and fw_minor contain the config version...
		read_config_data();		
	}
	printf("\nout of config read %d\n",TRUE);
#endif	
/**#########################################################################################*/	
	unsigned int baud_val[5]={1200,9600,19200,38400,57600};
	baud_rate_select = 2;
	baud_value=baud_val[baud_rate_select];

	#ifdef MOD_UART1
	UART_Init1(19200);
	#endif
	usbSerialInit();//cdc initilization

#ifdef MOD_UART1
	modbus_configure(&modbus1,(long)baud_value,1,HOLD_REG_MAX,holdingRegs); // MODBUS setup
#endif

#ifdef MOD_USB
	modbus_configure(&modbusUSB,115200,1,HOLD_REG_MAX,holdingRegs);
#endif

/**#########################################################################################*/	
	dt_init();
	stepper_init();	
	stepper_update_freq();
	iap_init();
	// init_qei();

/**#########################################################################################*/		
	/*Required for multi-steppers library*/
	RESET_MODBUS_TIMER;
	enable_timer(LPC_TIM0);
	enable_timer(LPC_TIM1);
	enable_timer(LPC_TIM2);
	enable_timer(LPC_TIM3);
/**#########################################################################################*/			
	for(i=0;i<DELAY_TOTAL_NUM;i++){
		dt_reset(i);
	}

	toggle = 0;
	/**#################################### Brownout ################################*/
	#if 1
	LPC_PINCON->PINSEL4 |= (1<<24);
	LPC_SC->EXTMODE |= (1<<2);			/* INT2 edge trigger */
	LPC_SC->EXTPOLAR &= ~(1<<2);		/* Falling edge */
	NVIC_SetPriority(EINT2_IRQn,7);
	NVIC_EnableIRQ(EINT2_IRQn);
	LPC_SC->EXTINT = EINT2_CLEAR;		/* Clear that interrupt once */
	#endif
/**#########################################################################################*/
	getSavedDataFromEeprom();
	printf("\nwhile is starting");

	while(1){


		
		/* update function for the modbus operation*/
		modbus_retVal = modbus_update(&modbus1);
		#ifdef MOD_USB
		modbus_retVal = modbus_update(&modbusUSB);
		#endif

		if(holdingRegs[MB_HMI_EXECUTE] == 0) {
			if(jog_type == JOG_CONT){
				if (SM_START == sm_jog_m1.SM_started()){
					// printf("\n 1reg_val=%d",holdingRegs[MB_HMI_EXECUTE]);
					sm_jog_m1.forced_stop = 1;
				}

				if (SM_START == sm_jog_m2.SM_started()){
					sm_jog_m2.forced_stop = 1;
					// printf("\n 1reg_val=%d",holdingRegs[MB_HMI_EXECUTE]);
				}
			}
		}

		if(flash_write_now){
			// printf("\n flash write");
			flash_write_now = 0;
			write_config_data();
		}

		if(dt_is_timeup(DELAY_HEARTBEAT)){
			dt_reset(DELAY_HEARTBEAT);
			if(toggle){
				toggle = 0;
				// CUT_ON;
				// BEND1_ON;
				// BEND2_ON;
				// LASER_ON;
				// EJECT_ON;
			}else{
				toggle = 1;
				// CUT_OFF;
				// BEND1_OFF;
				// BEND2_OFF;
				// LASER_OFF;
				// EJECT_OFF;
			}
		}
	}
	return 0;
}

int modbus_exception_handler(int modbus_ret){
	// printf("\n MB_DATA_WRITTEN %d",modbus_ret);
	switch(modbus_ret){
		case MB_OVERFLOW:
			// do something here..
		break;
		case MB_DATA_WRITTEN:{
			// printf("\n MB_DATA_WRITTEN");
			// int modbus_count;
			// copy the received data to local variables..
			// for(modbus_count=0;modbus_count<HOLDING_REGS_SIZE;modbus_count++){
				// apply_modbus_values(modbus_count);
			// }
			break;
		}
		case MB_CORRUPT_PACKET:
			// printf("\n MB_CORRUPT_PACKET");
			// do something here..
		break;
		case MB_CHECKSUM_FAILED:
			// printf("\n MB_CHECKSUM_FAILED");
			// do something here..
		break;
		case MB_INCORRECT_ID:
			// printf("\n MB_INCORRECT_ID");
			// do something here..
		break;
		case MB_DATA_READ:
			// printf("\n MB_DATA_READ");
			// do something here..
		break;
	}
	return 1;
}





extern "C"{
	int update_sm_operation_every_1ms(){

		if(SM_START == sm_jog_m1.SM_started()){
			sm_jog_m1.update();
		}
		
		if(SM_START == sm_jog_m2.SM_started()){
			sm_jog_m2.update();
		}

		if(SM_START == sm_manual.SM_started()){
			sm_manual.update();
		}

		if(SM_START == sm_operation.SM_started()){
			sm_operation.update();
		}
		return 1;
	}


	int saveDataOnBrownOut(void){
		BROWNOUT_STRUCT bdata;
		bdata.b_product_count		= product_count;
		bdata.b_mode_selection 		= mode_selection;
		for(int i=0 ; i<5; i++){

			bdata.b_stage_bypass[i] = stage_bypass[i];		
		}
		bdata.b_bend_individually 	= bend_individually; 
		i2cEEPROM_write8(128,(uint8_t*)&bdata,sizeof(bdata));
		return 1;
	}

	int getSavedDataFromEeprom(void){
		BROWNOUT_STRUCT bdata;
		i2cEEPROM_read8(128,(uint8_t*)&bdata,sizeof(bdata));
		product_count	= bdata.b_product_count;
		mode_selection	= bdata.b_mode_selection;
		for(int i=0 ; i<5; i++){
			stage_bypass[i] = bdata.b_stage_bypass[i];		
		}
		bend_individually 	= bdata.b_bend_individually; 
		// printf("\nRead: %d %d",product_count,sense_counter);
		return 1;
	}
}

