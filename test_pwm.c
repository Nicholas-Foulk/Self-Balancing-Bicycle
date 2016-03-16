/*
===============================================================================
 Name        : test_pwm.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif
#include <cr_section_macros.h>
#include <stdio.h>


void init_PWM(uint32_t PWMinval);
void init_GPIO();
void set_DIR(uint32_t dir);

int main(void) {
	uint32_t dir, cycle;
	init_GPIO();


while(1){
	printf("Input direction: ");
		scanf("%i", &dir);
		printf("\nInput speed 0,1,2,3: ");
		scanf("%x", &cycle);
		set_DIR(dir);

		if(cycle == 0)
		{
			init_PWM(0);
		}
		if(cycle == 1)
		{
			init_PWM(250);
		}
		if(cycle == 2)
		{
			init_PWM(500);
		}
		if (cycle == 3)
		{
			init_PWM(900);
		}

}
    return 0;
}


void init_PWM(uint32_t PWMinval)
{
	//Power for PWM1, sets 6th bit of PCONP register to 1 which is PWM1;
		//PCONP register is peripheral power control register
		LPC_SC->PCONP |= (1 << 6);
		//peripheral clock select for PWM cclk/8
		LPC_SC->PCLKSEL0 |= ((1 << 13) | (1 << 12));
		//pin select
		LPC_PINCON->PINSEL4 |= (0x1<<0); //sets 01 for bits [1:0] of PINSEL4 register, which sets P2.0 to PWM1.1 operation

		LPC_PWM1->TCR = (1<<1);   	//counter reset, set 2nd bit of TCR register to 1;
		LPC_PWM1->PR = 0;        //count frequency, prescale register
		LPC_PWM1->PC = 0;		//prescale counter
		LPC_PWM1->MCR = (1 << 1); //reset TC on Match 0
		//set period & duty cycle
		LPC_PWM1->MR0 = 1000;	  //set PWM period/cycle to 1khz
		LPC_PWM1->MR1 = PWMinval;	  //set 50% duty cycle; period/2
		//write enable for match registers
		LPC_PWM1->LER = (1<<0)|(1<<1); //latch MR0 and MR1 (must be used for those registers to be overwritten)
		//pwm enable, settings
		LPC_PWM1->PCR = (1<<9);   //PWM output enable, single-edged operation, must be set else otherwise PWM is a counter
		LPC_PWM1->TCR = (1<<0) | (1<<3); //TC enable, PWM enable

}


void init_GPIO()
{
	LPC_GPIO2->FIODIR |= (1 << 6); //set P2.6 as GPIO output
}

void set_DIR(uint32_t dir)
{
	if(dir)
		LPC_GPIO2->FIOSET = (1 << 6); //sets P2.6 as HI
	else
		LPC_GPIO2->FIOCLR = (1 << 6); //sets P2.6 as LO
}


