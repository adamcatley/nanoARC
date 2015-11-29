/*
 * txread.h
 *
 *  Created on: 8 Nov 2015
 *      Author: Adam
 */

#ifndef TXREAD_H_
#define TXREAD_H_

#include <xdc/std.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <driverLib/timer.h>

#define PPM_TIMER 			TIMER_B
#define PPM_TIMER_BASE		GPT2_BASE
#define PPM_TIMER_PERIPH	PERIPH_GPT2
#define PPM_PORT_EVENT 		IOC_PORT_MCU_PORT_EVENT5

//UART mode defines

typedef enum TXReadMode{
	TXReadMode_PPM,
	TXReadMode_UART
} TXReadMode;

void InitialiseTXRead(TXReadMode mode);

#endif /* TXREAD_H_ */
