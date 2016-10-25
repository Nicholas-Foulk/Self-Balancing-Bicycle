/*
 * PWM.h
 *
 *  Created on: Oct 16, 2016
 *      Author: Dustin
 */

#ifndef PWM_H_
#define PWM_H_

#include <cr_section_macros.h>
#include <stdint.h>
#include "LPC17xx.h"

typedef struct
{
	uint32_t dcycle;
	uint32_t period;
	uint32_t pin;
} PWM;

void create(PWM *newPWM, uint32_t port, uint32_t pin, uint32_t dcycle, uint32_t period);
void init_PWM(uint32_t PWM_pin, uint32_t dcycle, uint32_t period);
void set_period(uint32_t PWM_pin, uint32_t period);
void set_speed(uint32_t PWM_pin, uint32_t dcycle);

#endif /* PWM_H_ */
