/*
 * motors.c
 *
 *  Created on: 19 Oct 2015
 *      Author: Adam
 */

#include "motors.h"
#include "Board.h"

/* BIOS Header files */
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <driverLib/timer.h>

PIN_Config motorPinsPrimary[] = {
	Board_MOTOR1_EN    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_MOTOR1_PH    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_MOTOR2_EN    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_MOTOR2_PH    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE };

PIN_Config motorPinsSecondary[] = {
	Board_MOTOR3_EN    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_MOTOR3_PH    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_MOTOR4_EN    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_MOTOR4_PH    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE };

PIN_State  pinState;
PIN_Handle pinHandlePrimary;
PIN_Handle pinHandleSecondary;

void MotorGetTimer(Motor motor, uint32_t* base, uint32_t* timer);

bool MotorInitialisePrimary(){
	// Register client for pins in motorPins array
    pinHandlePrimary = PIN_open(&pinState, motorPinsPrimary);

	if (pinHandlePrimary == NULL) return false; //Already configured

    // Route pin to IO event port 0 (0 = Timer0A, 1 = Timer0B, 2 = Timer1A..)
    PINCC26XX_setMux(pinHandlePrimary, PIN_ID(Board_MOTOR1_EN), MOTOR1_PORT_EVENT);
    PINCC26XX_setMux(pinHandlePrimary, PIN_ID(Board_MOTOR2_EN), MOTOR2_PORT_EVENT);

	// Turn on PERIPH power domain and clock for GPT
	Power_setDependency(MOTOR1_TIMER_PERIPH);
	Power_setDependency(MOTOR2_TIMER_PERIPH);

    TimerConfigure(MOTOR1_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
    TimerConfigure(MOTOR2_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);

    TimerLoadSet(MOTOR1_TIMER_BASE, MOTOR1_TIMER, TIMER_LOADSET);
    TimerLoadSet(MOTOR2_TIMER_BASE, MOTOR2_TIMER, TIMER_LOADSET);

    //Stall timer when halting debugger
    TimerStallControl(MOTOR1_TIMER_BASE, MOTOR1_TIMER, false);
    TimerStallControl(MOTOR2_TIMER_BASE, MOTOR2_TIMER, false);

    MotorSetDirection(Motor1, MotorDirection_Forward);
    MotorSetDirection(Motor2, MotorDirection_Forward);
    MotorSetSpeed(Motor1, 0);
    MotorSetSpeed(Motor2, 0);

    //Enable timers
    TimerEnable(MOTOR1_TIMER_BASE, MOTOR1_TIMER);
    TimerEnable(MOTOR2_TIMER_BASE, MOTOR2_TIMER);

    return true;
}

bool MotorInitialiseSecondary(bool track){
    // Register client for pins in motorPins array
    pinHandleSecondary = PIN_open(&pinState, motorPinsSecondary);

	if (pinHandleSecondary == NULL) return false; //Already configured

	if (track == true){//Follow primary motors
		// Route pin to IO event port 0 (0 = Timer0A, 1 = Timer0B, 2 = Timer1A..)
		PINCC26XX_setMux(pinHandleSecondary, PIN_ID(Board_MOTOR3_EN), MOTOR1_PORT_EVENT);
		PINCC26XX_setMux(pinHandleSecondary, PIN_ID(Board_MOTOR4_EN), MOTOR2_PORT_EVENT);
		return true; //No need to configure any additional timers
	}
	else { //4WD
	    // Route pin to IO event port 0 (0 = Timer0A, 1 = Timer0B, 2 = Timer1A..)
	    PINCC26XX_setMux(pinHandleSecondary, PIN_ID(Board_MOTOR3_EN), MOTOR3_PORT_EVENT);
	    PINCC26XX_setMux(pinHandleSecondary, PIN_ID(Board_MOTOR4_EN), MOTOR4_PORT_EVENT);
	}

	// Turn on PERIPH power domain and clock for GPT
	Power_setDependency(MOTOR3_TIMER_PERIPH);
	Power_setDependency(MOTOR4_TIMER_PERIPH);

    TimerConfigure(MOTOR3_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
    TimerConfigure(MOTOR4_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);

    TimerLoadSet(MOTOR3_TIMER_BASE, MOTOR3_TIMER, TIMER_LOADSET);
    TimerLoadSet(MOTOR4_TIMER_BASE, MOTOR4_TIMER, TIMER_LOADSET);

    //Stall timer when halting debugger
    TimerStallControl(MOTOR1_TIMER_BASE, MOTOR1_TIMER, false);
    TimerStallControl(MOTOR2_TIMER_BASE, MOTOR2_TIMER, false);

    MotorSetDirection(Motor3, MotorDirection_Forward);
    MotorSetDirection(Motor4, MotorDirection_Forward);
    MotorSetSpeed(Motor3, 0);
    MotorSetSpeed(Motor4, 0);

    //Enable timers
    TimerEnable(MOTOR3_TIMER_BASE, MOTOR3_TIMER);
    TimerEnable(MOTOR4_TIMER_BASE, MOTOR4_TIMER);

    return true;
}

void MotorSetSpeed(Motor motor, uint8_t speed){
	uint32_t base;
	uint32_t timer;
	MotorGetTimer(motor, &base, &timer);

	if (speed == 0) speed = 1; //Match must be < load value

	TimerMatchSet(base, timer, SPEED_STEPS - 1 - speed);
}

void MotorSetDirection(Motor motor, MotorDirection dir){
	PIN_Handle handle;
	PIN_Id id;
	uint_t val;

	switch (motor) {
			case Motor1:
				handle = pinHandlePrimary;
				id = Board_MOTOR1_PH;
				break;
			case Motor2:
				handle = pinHandlePrimary;
				id = Board_MOTOR2_PH;
				break;
			case Motor3:
				handle = pinHandleSecondary;
				id = Board_MOTOR3_PH;
				break;
			case Motor4:
				handle = pinHandleSecondary;
				id = Board_MOTOR4_PH;
				break;
			default:
				break;
		}

	if (dir == MotorDirection_Forward){
		val = 1;
	}
	else {
		val = 0;
	}

    PIN_setOutputValue(handle, id, val);
}

void MotorSetForward(Motor motor){
	MotorSetDirection(motor, MotorDirection_Forward);
}

void MotorSetBackward(Motor motor){
	MotorSetDirection(motor, MotorDirection_Backward);
}

void MotorGetTimer(Motor motor, uint32_t* base, uint32_t* timer){
	switch (motor) {
			case Motor1:
				*base = MOTOR1_TIMER_BASE;
				*timer = MOTOR1_TIMER;
				break;
			case Motor2:
				*base = MOTOR2_TIMER_BASE;
				*timer = MOTOR2_TIMER;
				break;
			case Motor3:
				*base = MOTOR3_TIMER_BASE;
				*timer = MOTOR3_TIMER;
				break;
			case Motor4:
				*base = MOTOR4_TIMER_BASE;
				*timer = MOTOR4_TIMER;
				break;
			default:
				break;
		}
}
