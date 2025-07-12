#ifndef _TIMER_H_
#define _TIMER_H_

#include "LPC17xx.h"		/* LPC17XX Peripheral Registers	*/
#include <stdint.h>
#include <stdio.h>
#include "lpc17xx_timer.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"

#ifdef __cplusplus
extern "C"{
#endif

#define TIMER_CLK_FREQUENCY	100000000
#define INTERVAL_10MS	149999			/* 10mSec = 150.000-1 counts */
#define TIMER0_PRESCALE 9			// 10 -1 counts...

void enable_timer(LPC_TIM_TypeDef *TIMx);
void disable_timer(LPC_TIM_TypeDef *TIMx);
void reset_timer(LPC_TIM_TypeDef *TIMx);

int init_timer0();
int init_timer0_freq (uint32_t freq);
int init_timer0_mode_match (uint32_t mode, uint32_t match);

int init_timer1();
int init_timer1_freq (uint32_t freq);
int init_timer1_mode_match (uint32_t mode, uint32_t match);

int init_timer2 ();
int init_timer2_freq (uint32_t freq);
int init_timer2_mode_match (uint32_t mode, uint32_t match);

int init_timer3 ();
int init_timer3_freq (uint32_t freq);
int init_timer3_mode_match (uint32_t mode, uint32_t match);

int init_pwm_timer(uint32_t freq);
int init_pwm_timer_mode_match(uint32_t mode, uint32_t match);
void enable_timer_pwm(void);
void disable_timer_pwm(void);
void reset_timer_pwm(void);

#ifdef __cplusplus
}
#endif
#endif /* end __TIMER_H */
