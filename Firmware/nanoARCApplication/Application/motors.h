/*
 * motors.h
 *
 *  Created on: 19 Oct 2015
 *      Author: Adam
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <xdc/std.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <driverLib/timer.h>

#define MOTOR1_TIMER 		TIMER_A
#define MOTOR1_TIMER_BASE	GPT0_BASE
#define MOTOR1_TIMER_PERIPH	PERIPH_GPT0
#define MOTOR1_PORT_EVENT 	IOC_PORT_MCU_PORT_EVENT0

#define MOTOR2_TIMER 		TIMER_B
#define MOTOR2_TIMER_BASE	GPT0_BASE
#define MOTOR2_TIMER_PERIPH	PERIPH_GPT0
#define MOTOR2_PORT_EVENT 	IOC_PORT_MCU_PORT_EVENT1

#define MOTOR3_TIMER 		TIMER_A
#define MOTOR3_TIMER_BASE	GPT1_BASE
#define MOTOR3_TIMER_PERIPH	PERIPH_GPT1
#define MOTOR3_PORT_EVENT 	IOC_PORT_MCU_PORT_EVENT2

#define MOTOR4_TIMER 		TIMER_B
#define MOTOR4_TIMER_BASE	GPT1_BASE
#define MOTOR4_TIMER_PERIPH	PERIPH_GPT1
#define MOTOR4_PORT_EVENT 	IOC_PORT_MCU_PORT_EVENT3

#define SPEED_STEPS 256 //48MHz / 256 = 187.5kHz
#define TIMER_LOADSET (SPEED_STEPS-1)

typedef enum Motor{
	Motor1,
	Motor2,
	Motor3,
	Motor4
} Motor;

typedef enum MotorDirection{
	MotorDirection_Forward,
	MotorDirection_Backward
} MotorDirection;

bool MotorInitialisePrimary();
bool MotorInitialiseSecondary(bool track);

void MotorSetSpeed(Motor motor, uint8_t speed);
void MotorSetDirection(Motor motor, MotorDirection dir);
void MotorSetForward(Motor motor);
void MotorSetForward(Motor motor);


#endif /* MOTORS_H_ */
