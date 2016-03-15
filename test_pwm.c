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

// TODO: insert other include files here

// TODO: insert other definitions and declarations here
void init_PWM();
void set_PWM(int PWMval);

int main(void) {

	//Power for PWM1, sets 6th bit of PCONP register to 1 which is PWM1;
	//PCONP register is peripheral power control register
	LPC_SC->PCONP |= (1 << 6);
	//peripheral clock select for PWM cclk/8
	LPC_SC->PCLKSEL0 |= ((1 << 13) | (1 << 12));
	//pin select
	LPC_PINCON->PINSEL4 |= (0x1<<0); //sets 01 for bits [1:0] of PINSEL4 register, which sets P2.0 to PWM1.1 operation
	/*
	LPC_PINCON->PINSEL4 &= ~(1<<1); //set bit 1 of PINSEL4 register to 0
	LPC_PINCON->PINSEL4 |= 1; //set PWM1.1 at P2.0 J6-42; When bits 1:0 are set to 01 then pin is PWM1.1 operation
	*/
	LPC_PWM1->TCR = (1<<1);   	//counter reset, set 2nd bit of TCR register to 1;
	LPC_PWM1->PR = 0;        //count frequency, prescale register
	LPC_PWM1->PC = 0;		//prescale counter
	LPC_PWM1->MCR = (1 << 1); //reset TC on Match 0
	//set period & duty cycle
	LPC_PWM1->MR0 = 1000;	  //set PWM period/cycle to 1khz
	LPC_PWM1->MR1 = 500;	  //set 50% duty cycle; period/2
	//write enable for match registers
	LPC_PWM1->LER = (1<<0)|(1<<1); //latch MR0 and MR1 (must be used for those registers to be overwritten)
	//pwm enable, settings
	LPC_PWM1->PCR = (1<<9);   //PWM output enable, single-edged operation, must be set else otherwise PWM is a counter
	LPC_PWM1->TCR = (1<<0) | (1<<3); //TC enable, PWM enable
    return 0 ;
}

void init_PWM()
{


}

void set_PWM(int PWMval)
{
	LPC_PWM1->MR1 = PWMval;

}


/*
LPC_SC->PCONP |= (1<<6); //--- PWM1 power On
LPC_SC->PCLKSEL0 |= ((1<<13)|(1<<12)); //--- CCLK/8
LPC_PWM1->TCR = (1<<1); //--- Reset counter
LPC_PWM1->PR = 0;
LPC_PWM1->PC = 0;
LPC_PINCON->PINSEL4 &= ~(1<<1);
LPC_PINCON->PINSEL4 |= (1<<0);
LPC_PINCON->PINMODE4 &= ~(1<<0);//-- no pull-up
LPC_PINCON->PINMODE4 |= (1<<1);
LPC_PWM1->MR0 = 1000; //--- test period
LPC_PWM1->MR1 = 500; //--- test period/2
LPC_PWM1->MCR = (1<<1);
LPC_PWM1->PCR = (1<<9); //--- PWM1 output
LPC_PWM1->TCR = (1<<0)|(1<<3);
 */
