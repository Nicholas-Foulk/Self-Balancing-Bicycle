/*
 * StepperMotor.c
 *
 *  Created on: Nov 1, 2016
 *      Author: nick
 */
/*
 * StepperMotor.c
 *
 *  Created on: Oct 25, 2016
 *      Author: Dustin
 */

#include "StepperMotor.h"

void stepperInit(uint32_t port, uint32_t pin)
{
	if(port == 0)
	{
	 	LPC_PINCON->PINSEL1 &= ~(0x63<<14); // fix later
		LPC_GPIO0->FIODIR |= 0x1<<pin;
	}
	else if(port == 2)
	{
		LPC_PINCON->PINSEL4 &= ~(3<<pin);
		LPC_GPIO2->FIODIR |= (0x1<<pin);
	}
}
void stepperTurnF(uint32_t port, uint32_t pin, uint32_t dirport, uint32_t dirpin, uint32_t duration, uint32_t PWM)
{
	int opp = 10 - PWM;
	if(port == 0)
		{
			if(dirport == 0)
			{
				LPC_GPIO0->FIOSET |= 1<<dirpin;
			}
			else if(dirport == 2)
			{
				LPC_GPIO2->FIOSET |= 1<<dirpin;
			}
			int m;
			for(m = 0; m < duration; m=m+1)
			{
				LPC_GPIO0->FIOCLR |= 1<<pin;
				vTaskDelay(PWM);
				LPC_GPIO0->FIOSET |= 1<<pin;
				vTaskDelay(opp);
			}
		}
		else if(port == 2)
		{
			if(dirport == 0)
			{
				LPC_GPIO0->FIOSET |= 1<<dirpin;
			}
			else if(dirport == 2)
			{
				LPC_GPIO2->FIOSET |= 1<<dirpin;
			}
			int m;
			for(m = 0; m < duration; m=m+1)
			{
				LPC_GPIO2->FIOCLR |= 1<<pin;
				vTaskDelay(PWM);
				LPC_GPIO2->FIOSET |= 1<<pin;
				vTaskDelay(opp);
			}
		}
}
void stepperTurnR(uint32_t port, uint32_t pin, uint32_t dirport, uint32_t dirpin, uint32_t duration, uint32_t PWM)
{
	int opp = 10 - PWM;
	if(port == 0)
	{
		if(dirport == 0)
		{
			LPC_GPIO0->FIOCLR |= 1<<dirpin;
		}
		else if(dirport == 2)
		{
			LPC_GPIO2->FIOCLR |= 1<<dirpin;
		}
		int m;
		for(m = 0; m < duration; m=m+1)
		{
			LPC_GPIO0->FIOCLR |= 1<<pin;
			vTaskDelay(PWM);
			LPC_GPIO0->FIOSET |= 1<<pin;
			vTaskDelay(opp);
		}
	}
	else if(port == 2)
	{
		if(dirport == 0)
		{
			LPC_GPIO0->FIOCLR |= 1<<dirpin;
		}
		else if(dirport == 2)
		{
			LPC_GPIO2->FIOCLR |= 1<<dirpin;
		}
		int m;
		for(m = 0; m < duration; m=m+1)
		{
			LPC_GPIO2->FIOCLR |= 1<<pin;
			vTaskDelay(PWM);
			LPC_GPIO2->FIOSET |= 1<<pin;
			vTaskDelay(opp);
		}
	}
}
void stepperRelease(uint32_t port, uint32_t pin) //used to clear out configurations for stepper motor, not important to implement
{
	//PINCON [whatever] = 00
	//DIR [whatever] = 00
}


