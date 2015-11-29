/*
 * txread.c
 *
 *  Created on: 8 Nov 2015
 *      Author: Adam
 */

#include "txread.h"
#include "Board.h"

/* BIOS Header files */
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <driverLib/timer.h>
#include <driverLib/prcm.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

PIN_State  pinState;
PIN_Handle pinHandle;

PIN_Config PPMPin[] = {
	Board_GPIO_0 | PIN_INPUT_EN | PIN_NOPULL,
	PIN_TERMINATE };

static void IntHandler(UArg arg);

/// HW interrupt structure for I/O interrupt handler
static ti_sysbios_family_arm_m3_Hwi_Struct PinHwi;

void InitialiseTXRead(TXReadMode mode){
    Hwi_Params hwiParams;

	//Initialise pins
    pinHandle= PIN_open(&pinState, PPMPin);
    PINCC26XX_setMux(pinHandle, PIN_ID(Board_GPIO_0), PPM_PORT_EVENT);

    //PRCMGPTimerClockDivisionSet(PRCM_CLOCK_DIV_32);//TODO: Fix unresolved symbol when called
    HWREG( PRCM_BASE + PRCM_O_GPTCLKDIV ) = PRCM_CLOCK_DIV_32; //1.5 ticks per microsecond
    PRCMLoadSet();

	Power_setDependency(PPM_TIMER_PERIPH);
	TimerConfigure(PPM_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_CAP_TIME_UP);

	TimerEventControl(PPM_TIMER_BASE, PPM_TIMER, TIMER_EVENT_POS_EDGE);

    Hwi_Params_init(&hwiParams);
    hwiParams.enableInt = 1;
    hwiParams.arg = (UArg)INT_TIMER2B;
    Hwi_construct(&PinHwi, INT_TIMER2B, IntHandler,&hwiParams, NULL);
	//TimerIntRegister(PPM_TIMER_BASE, PPM_TIMER, &IntHandler); //Use Hwi_construct instead

	TimerIntClear(PPM_TIMER_BASE, TIMER_CAPB_EVENT);
	TimerIntEnable(PPM_TIMER_BASE, TIMER_CAPB_EVENT);

    TimerStallControl(PPM_TIMER_BASE, PPM_TIMER, false);

    TimerEnable(PPM_TIMER_BASE, PPM_TIMER);
}

int chanNum = 0;
uint32_t chanValues[6];

static void IntHandler(UArg arg){
	TimerIntClear(PPM_TIMER_BASE, TIMER_CAPB_EVENT); //Must do first

	uint32_t value = TimerValueGet(PPM_TIMER_BASE, PPM_TIMER);
	HWREG(PPM_TIMER_BASE + GPT_O_TBV) = 0; //reset timer to abvid overflow

	if ((value > 6000) ) {//start of frame. >4ms is not a channel
		chanNum = 0; //reset counter
		return;
	}

	chanValues[chanNum] = value;
	chanNum++;

	if ((chanNum >= 6) ) {//end of frame
		chanNum = 0; //reset counter
	}

}
