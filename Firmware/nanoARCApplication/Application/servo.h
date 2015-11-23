/*
 * servo.h
 *
 *  Created on: 19 Oct 2015
 *      Author: Adam
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <xdc/std.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <driverLib/timer.h>
#include "Board.h"

#define SERVO_TIMER 		TIMER_A
#define SERVO_TIMER_BASE	GPT2_BASE
#define SERVO_TIMER_PERIPH	PERIPH_GPT2
#define SERVO_PORT_EVENT 	IOC_PORT_MCU_PORT_EVENT4

#define TIMER_LOADSET 960000 //48,000,000 / 50 = 960,000

typedef enum GPIO{
	GPIO0 = Board_GPIO_0,
	GPIO1 = Board_GPIO_1,
	GPIO2 = Board_GPIO_2,
	GPIO3 = Board_GPIO_3,
	GPIO4 = Board_GPIO_4
} GPIO;

bool ServoInitialise(GPIO gpio);

void ServoSet(int16_t value); //-1000 to 1000, 0 is mid point

#endif /* SERVO_H_ */
