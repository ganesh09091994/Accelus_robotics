#include "qei_test.h"

/*********************************************************************//**
 * @brief		QEI interrupt handler. This sub-routine will update current
 * 				value of captured velocity in to velocity accumulate.
 * @param[in]	None
 * @return 		None
 **********************************************************************/
/*
 void QEI_IRQHandler(void)
{
	// Check whether if velocity timer overflow
	if (QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_TIM_Int) == SET) {
		if (VeloAccFlag == RESET) {

			// Get current velocity captured and update to accumulate
			VeloAcc += QEI_GetVelocityCap(LPC_QEI);

			// Update Velocity capture times
			VeloAccFlag = ((VeloCapCnt++) >= MAX_CAP_TIMES) ? SET : RESET;
		}
		// Reset Interrupt flag pending
		QEI_IntClear(LPC_QEI, QEI_INTFLAG_TIM_Int);
	}

	// Check whether if direction change occurred
	if (QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_DIR_Int) == SET) {
		// Print direction status
		// _DBG("Direction has changed: ");
		// _DBG_((QEI_GetStatus(LPC_QEI, QEI_STATUS_DIR) == SET) ? "1" : "0");
		// Reset Interrupt flag pending
		QEI_IntClear(LPC_QEI, QEI_INTFLAG_DIR_Int);
	}
}
*/

void QEI_IRQHandler (void){
	uint32_t tmp;
	/* Get the current position */
	uint32_t pos = LPC_QEI->QEIPOS;	
	/* Clear the position interrupt */
	QEI_IntClear(LPC_QEI,QEI_INTFLAG_POS0_Int);	
	/* Re-enable QEI index interrupt */
	LPC_QEI->QEICON = (1<<1);
    return;
}

void init_qei(){
	PINSEL_CFG_Type PinCfg;
	QEI_CFG_Type QEIConfig;
	QEI_RELOADCFG_Type ReloadConfig;	
	
	/* Initialize QEI configuration structure to default value */
#if CAP_MODE
	QEIConfig.CaptureMode = QEI_CAPMODE_4X;
#else
	QEIConfig.CaptureMode = QEI_CAPMODE_2X;
#endif
	QEIConfig.DirectionInvert = QEI_DIRINV_NONE;
	QEIConfig.InvertIndex = QEI_INVINX_NONE;
#if SIGNAL_MODE
	QEIConfig.SignalMode = QEI_SIGNALMODE_CLKDIR;
#else
	QEIConfig.SignalMode = QEI_SIGNALMODE_QUAD;
#endif
	
	/* Set QEI function pin
	 * P1.20: MCI0
	 * P1.23: MCI1
	 * P1.24: MCI2
	 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 20;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 23;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 24;
	PINSEL_ConfigPin(&PinCfg);
	
	/* Initialize QEI peripheral with given configuration structure */
	QEI_Init(LPC_QEI, &QEIConfig);
/*
	// Set timer reload value for  QEI that used to set velocity capture period
	ReloadConfig.ReloadOption = QEI_TIMERRELOAD_USVAL;
	ReloadConfig.ReloadValue = CAP_PERIOD;
	QEI_SetTimerReload(LPC_QEI, &ReloadConfig);
*/
	/* Set the QEI Filter clock counter*/
	// QEI_SetDigiFilter(LPC_QEI, 20);
	QEI_SetDigiFilter(LPC_QEI, 0x1fff);
	/* Set the max QEI position */
	// QEI_SetMaxPosition(LPC_QEI, 2000);
	QEI_SetMaxPosition(LPC_QEI, 0xFFFFFFFFL);
	/* Set the first position compare value */
	// QEI_SetPositionComp(LPC_QEI,QEI_COMPPOS_CH_0, 333+QEI_offset);

	LPC_QEI->QEICON = (1<<1);
	
	/* Enable interrupt for velocity Timer overflow for capture velocity into Acc */
	// QEI_IntCmd(LPC_QEI, QEI_INTFLAG_POS0_Int, ENABLE);
	/* preemption = 1, sub-priority = 1 */
	// NVIC_SetPriority(QEI_IRQn, 7);
	/* Enable interrupt for QEI  */
	// NVIC_EnableIRQ(QEI_IRQn);
/*	
	// Enable interrupt for velocity Timer overflow for capture velocity into Acc
	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_TIM_Int, ENABLE);
	// Enable interrupt for direction change
	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_DIR_Int, ENABLE);
*/
}
