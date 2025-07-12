#ifndef _QEI_TEST_H_
#define _QEI_TEST_H_

#include "lpc17xx_qei.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_clkpwr.h"
#include "debug_frmwrk.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"

/** Signal Mode setting:
 * - When = 0, PhA and PhB function as quadrature encoder inputs.
 * - When = 1, PhA functions as the direction signal and PhB functions
 * as the clock signal
 */
#define SIGNAL_MODE 		0

/** Capture Mode setting:
 * - When = 0, only PhA edges are counted (2X).
 * - When = 1, BOTH PhA and PhB edges are counted (4X), increasing
 * resolution but decreasing range
 */
#define CAP_MODE 			1

/** Velocity capture period definition (in microsecond) */
#define CAP_PERIOD			250000UL

/** Delay time to Read Velocity Accumulator and display (in microsecond)*/
#define DISP_TIME			3000000UL
/** Max velocity capture times calculated */
#define MAX_CAP_TIMES		(DISP_TIME/CAP_PERIOD)

#define ENC_RES	 			2048UL	/**< Encoder resolution (PPR) */

/* Pin on Port 0 assigned to Phase A */
#define PHASE_A_PIN			(1<<19)
/* Pin on Port 0 assigned to Phase B */
#define PHASE_B_PIN			(1<<21)

#define QEI_steps	2000
#define QEI_offset  24*4

/** Velocity Accumulator */
__IO uint64_t VeloAcc;
/** Times of Velocity capture */
__IO uint32_t VeloCapCnt;
/** Flag indicates Times of Velocity capture is enough to read out */
__IO FlagStatus VeloAccFlag;

/* Pin Configuration selection must be defined in structure following:
 * - Port Number,
 * - Pin Number,
 * - Function Number,
 * - Pin Mode,
 * - Open Drain
 */

/** QEI Phase-A Pin */
const PINSEL_CFG_Type qei_phaA_pin[1] = {{1, 20, 1, 0, 0}};
/** QEI Phase-B Pin */
const PINSEL_CFG_Type qei_phaB_pin[1] = {{1, 23, 1, 0, 0}};
/** QEI Index Pin */
const PINSEL_CFG_Type qei_idx_pin[1] = {{1, 24, 1, 0, 0}};

void QEI_IRQHandler(void);

void init_qei();

#endif