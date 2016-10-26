/*
 * PWM.c
 *
 *  Created on: Oct 16, 2016
 *      Author: Dustin
 */

#include "PWM.h"

//void create(PWM *newPWM, uint32_t pin, uint32_t dcycle, uint32_t period)
//{
//	newPWM->dcycle = dcycle;
//	newPWM->pin = pin;
//	newPWM->period = period;
//}

void initPWM(uint32_t PWM_pin, uint32_t dcycle, uint32_t period)
{
		if(0 <= PWM_pin <= 5)
		{
			//Power for PWM1, sets 6th bit of PCONP register to 1 which is PWM1;
			//PCONP register is peripheral power control register
			LPC_SC->PCONP |= (1 << 6);
			//peripheral clock select for PWM cclk/8
			LPC_SC->PCLKSEL0 |= ((1 << 13) | (1 << 12));
			//pin select
			LPC_PINCON->PINSEL4 &= ~(0x3<<PWM_pin*2);
			LPC_PINCON->PINSEL4 |= (0x1<<PWM_pin*2);
			LPC_PWM1->TCR = (1<<1);   	//counter reset, set 2nd bit of TCR register to 1;
			LPC_PWM1->PR = 0;        //count frequency, prescale register
			LPC_PWM1->PC = 0;		//prescale counter
			LPC_PWM1->MCR = (1 << 1); //reset TC on Match 0
			//set period & duty cycle
			LPC_PWM1->MR0 = period;	  //set PWM period/cycle to 1khz
			if(PWM_pin == 0) LPC_PWM1->MR1 = dcycle;	  //set 50% duty cycle; period/2
			else if(PWM_pin == 1) LPC_PWM1->MR2 = dcycle;
			//write enable for match registers
			LPC_PWM1->LER = (1<<0)|(1<<(PWM_pin+1)); //latch MR0 and MR(n) (must be used for those registers to be overwritten)
			//pwm enable, settings
			LPC_PWM1->PCR |= (1<<(9+PWM_pin));   //PWM output enable, single-edged operation, must be set else otherwise PWM is a counter
			LPC_PWM1->TCR = (1<<0) | (1<<3); //TC enable, PWM enable
		}
		else
		{
			//do nothing
		}


}

void setPWMperiod(uint32_t period) //affects all pins on PWM1
{
	LPC_PWM1->MR0 = period;
}
void setPWMspeed(uint32_t PWM_pin, uint32_t dcycle)
{
	if(0 <= PWM_pin <= 5)
	{
		if(PWM_pin == 0) LPC_PWM1->MR1 = dcycle;		//duty cycle is a %, so dcycle = dividend/period --> dividend = dcycle*period ;
		else if (PWM_pin == 1)LPC_PWM1->MR2 = dcycle;
		LPC_PWM1->LER = (1<<(PWM_pin+1)); //latch MR(n) (must be used for those registers to be overwritten)
	}
}
