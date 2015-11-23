/*
 * servo.c
 *
 *  Created on: 19 Oct 2015
 *      Author: Adam
 */

#include "servo.h"
#include "Board.h"

/* BIOS Header files */
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <driverLib/timer.h>

PIN_Config servoPin[] = {
	Board_GPIO_0    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE };

PIN_State  pinState;
PIN_Handle pinHandle;

bool ServoInitialise(GPIO gpio){
	//TODO: check for pin already open
	
	servoPin[0] = gpio  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX;

	// Register client for pins in motorPins array
    pinHandle = PIN_open(&pinState, servoPin);

    // Route pin to IO event port 0 (0 = Timer0A, 1 = Timer0B, 2 = Timer1A..)
    PINCC26XX_setMux(pinHandle, PIN_ID(gpio), SERVO_PORT_EVENT);

	// Turn on PERIPH power domain and clock for GPT
	Power_setDependency(SERVO_TIMER_PERIPH);

    TimerConfigure(SERVO_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
	
	TimerPrescaleSet(SERVO_TIMER_BASE, SERVO_TIMER, TIMER_LOADSET >> 16); //prescaler is bits 23:16 of timer load value
    TimerLoadSet(SERVO_TIMER_BASE, SERVO_TIMER, TIMER_LOADSET & 0xffff); //Lower 16 bits of load value

    //Invert output
    TimerLevelControl(SERVO_TIMER_BASE, SERVO_TIMER, true);

    //Stall timer when halting debugger
    TimerStallControl(SERVO_TIMER_BASE, SERVO_TIMER, false);

    ServoSet(0); //Set to neutral

    //Enable timers
    TimerEnable(SERVO_TIMER_BASE, SERVO_TIMER);

    return true;
}

void ServoSet(int16_t value){ 

	//Match must be < load value
	if (value > 1000) value = 1000;
	else if (value < -1000) value = -1000;
	
	uint32_t match = 72000 + (48 * value); //Midpoint = 960,000 * 0.075 = 72000

	TimerMatchSet(SERVO_TIMER_BASE, SERVO_TIMER, (match - 1) & 0xffff);
	TimerPrescaleMatchSet(SERVO_TIMER_BASE, SERVO_TIMER, (match - 1) >> 16);
}
