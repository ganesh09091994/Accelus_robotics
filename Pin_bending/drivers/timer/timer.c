/*****************************************************************************
 *   timer.c:  Timer C file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#include "timer.h"
#include <stdio.h>

TIM_TIMERCFG_Type TIM_ConfigStruct;
TIM_MATCHCFG_Type TIM_MatchConfigStruct;

#define TIMER0_PRIORITY		2
#define TIMER1_PRIORITY		4
#define TIMER2_PRIORITY		5
#define TIMER3_PRIORITY		6 //higher priority than uart2

/******************************************************************************
** Function name:		
**
** Descriptions:		Enable timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void enable_timer(LPC_TIM_TypeDef *TIMx)
{
    if (TIMx == LPC_TIM0)
		TIM_Cmd(LPC_TIM0,ENABLE);

	else if (TIMx == LPC_TIM1)
		TIM_Cmd(LPC_TIM1,ENABLE);

	else if (TIMx == LPC_TIM2)
		TIM_Cmd(LPC_TIM2,ENABLE);

	else if (TIMx == LPC_TIM3)
		TIM_Cmd(LPC_TIM3,ENABLE);
    return;
}
/******************************************************************************
** Function name:		disable_timer
**
** Descriptions:		Disable timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void disable_timer(LPC_TIM_TypeDef *TIMx)
{
    if (TIMx== LPC_TIM0)
		TIM_Cmd(LPC_TIM0,DISABLE);

	else if (TIMx== LPC_TIM1)
		TIM_Cmd(LPC_TIM1,DISABLE);

	else if (TIMx== LPC_TIM2)
		TIM_Cmd(LPC_TIM2,DISABLE);

	else if (TIMx== LPC_TIM3)
		TIM_Cmd(LPC_TIM3,DISABLE);
	return;	
}

/******************************************************************************
** Function name:		reset_timer
**
** Descriptions:		Reset timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void reset_timer(LPC_TIM_TypeDef *TIMx)
{
    if (TIMx== LPC_TIM0)
		TIM_ResetCounter(LPC_TIM0);

	else if (TIMx== LPC_TIM1)
		TIM_ResetCounter(LPC_TIM1);

	else if (TIMx== LPC_TIM2)
		TIM_ResetCounter(LPC_TIM2);

	else if (TIMx== LPC_TIM3)
		TIM_ResetCounter(LPC_TIM3);
		
	return;
}

/******************************************************************************
** Function name:		init_timer
**
** Descriptions:		Initialize timer, set timer interval, reset timer,
**				install timer interrupt handler
**
** parameters:			None
** Returned value:		true or false, if the interrupt handler can't be
**				installed, return false.
** 
******************************************************************************/

//---------Timer 0 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer0 (void){
	// Initialize timer 0, prescale count time of 100uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1; // prescalar value for 1us
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 1;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = ENABLE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 1000mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = 25;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);

//--------------------------------------------------------			
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, TIMER0_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER0_IRQn);	
	return 0;
}

//---------Timer 0 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer0_freq (uint32_t freq){	
	// Initialize timer 0, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = freq;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, TIMER0_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER0_IRQn);
	
	return 0;
}

//---------Timer 0 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer0_mode_match (uint32_t mode, uint32_t match){
	// Initialize timer 0, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = match;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, TIMER0_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER0_IRQn);
	
	return 0;
}

//---------Timer 1 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer1 (void){
	// Initialize timer 0, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = TRUE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = 1000;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM1, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM1,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER1_IRQn, TIMER1_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER1_IRQn);
	
	return 0;
}

//---------Timer 1 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer1_freq (uint32_t freq){
	// Initialize timer 0, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = freq;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM1, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM1,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER1_IRQn, TIMER1_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER1_IRQn);
	
	return 0;
}

//---------Timer 1 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer1_mode_match (uint32_t mode, uint32_t match){
	// Initialize timer 0, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = TRUE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = match;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM1, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM1,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER1_IRQn, TIMER1_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER1_IRQn);
	
	return 0;
}

//---------Timer 2 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer2 (void){
	// Initialize timer 2, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = 1;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM2, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM2,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	// NVIC_SetPriority((IRQn_Type)ISR_Tc2, ((0x01<<3)|0x01));
	// NVIC_SetPriority((IRQn_Type)ISR_Tc2, (3));
	NVIC_SetPriority(TIMER2_IRQn, TIMER2_PRIORITY);
	/* Enable interrupt for timer 0 */
	// NVIC_EnableIRQ((IRQn_Type)ISR_Tc2);
	NVIC_EnableIRQ(TIMER2_IRQn);	
	
	return 0;
}

//---------Timer 2 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer2_freq (uint32_t freq){
	// Initialize timer 0, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = TRUE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = freq;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM2, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM2,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER2_IRQn, TIMER2_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER2_IRQn);
	
	return 0;
}

//---------Timer 2 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer2_mode_match (uint32_t mode, uint32_t match){
	// Initialize timer 0, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = match;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM2, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM2,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER2_IRQn, TIMER2_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER2_IRQn);
	
	return 0;
}

//---------Timer 3 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer3 (void){
	// Initialize timer 2, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1000;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = 1000;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM3, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM3,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */	
	NVIC_SetPriority(TIMER3_IRQn, TIMER3_PRIORITY);
	/* Enable interrupt for timer 0 */	
	NVIC_EnableIRQ(TIMER3_IRQn);	
	
	return 0;
}

//---------Timer 2 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer3_freq (uint32_t freq){
	// Initialize timer 0, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = freq;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM3, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM3,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER3_IRQn, TIMER3_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER3_IRQn);
	
	return 0;
}

//---------Timer 2 initialized---------------

//default timer working at 100Mhz
//--------------------------------------------------------
int init_timer3_mode_match (uint32_t mode,uint32_t match){
	// Initialize timer 0, prescale count time of 1000uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0.0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value of 1000 (1000 * 1000uS = 100mS --> 1 Hz)
	TIM_MatchConfigStruct.MatchValue   = match;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM3, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM3,&TIM_MatchConfigStruct);

//--------------------------------------------------------		
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER3_IRQn, TIMER3_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER3_IRQn);
	
	return 0;
}

	
