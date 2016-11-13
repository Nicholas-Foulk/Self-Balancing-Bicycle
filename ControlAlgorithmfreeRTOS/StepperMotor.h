/*
 * StepperMotor.h
 *
 *  Created on: Nov 1, 2016
 *      Author: nick
 */
/*
 * StepperMotor.h
 *
 *  Created on: Oct 16, 2016
 *      Author: Dustin
 */

#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

#include <cr_section_macros.h>
#include <stdint.h>
#include "LPC17xx.h"
#include "FreeRTOS.h"

//make a struct

void stepperInit(uint32_t port, uint32_t pin);
void stepperTurnF(uint32_t port, uint32_t pin, uint32_t dirport, uint32_t dirpin, uint32_t duration, uint32_t PWM);
void stepperTurnR(uint32_t port, uint32_t pin, uint32_t dirport, uint32_t dirpin, uint32_t duration, uint32_t PWM);
void stepperRelease(uint32_t port, uint32_t pin);


#endif /* STEPPERMOTOR_H_ */
