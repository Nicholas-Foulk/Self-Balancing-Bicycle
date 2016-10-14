/*
    FreeRTOS V8.0.1 - Copyright (C) 2014 Real Time Engineers Ltd.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo includes. */
#include "basic_io.h"
#include "math.h"
#include <cr_section_macros.h>
#include <stdio.h>

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xfffff )

/*Added definitions from Control Algorithm Section*/
#define SSP_BUFSIZE	16
#define FIFOSIZE	8

/* The task functions. */
void vTask1( void *pvParameters );
void vTask3( void *pvParameters );
void vTask4( void *pvParameters );

/*Added functions from Control Algorithm Section*/

uint8_t read(void);
void SSP_init();
//void SSPSend(uint8_t address, uint8_t *buf, uint32_t length);
//void SSPReceive(uint8_t address, uint8_t *buf, uint32_t length);
void SSPSend(uint8_t address, uint8_t buf);
uint8_t SSPReceive(uint8_t address);
void SSP_SSEL(int port, int toggle);
int is_Tx_not_full();
int is_Tx_empty();
int is_Rx_full();
int is_Rx_not_empty();
int is_busy();
void sleep_us (int us);
void init_PWM(uint32_t PWM_num, uint32_t PWMinval, uint32_t speed);

/*-------------------------- ---------------------------------*/

int main( void )
{
	/* Init the semi-hosting. */
	printf( "\n" );

	/* Create one of the two tasks. */
	//xTaskCreate(	vTask1,		/* Pointer to the function that implements the task. */
				//	"Task 1",	/* Text name for the task.  This is to facilitate debugging only. */
				//	240,		/* Stack depth in words. */
				//	NULL,		/* We are not using the task parameter. */
				//	1,			/* This task will run at priority 1. */
				//	NULL );		/* We are not using the task handle. */

	/* Create the other task in exactly the same way. */
	//xTaskCreate( vTask2, "Task 2", 240, NULL, 1, NULL );

	xTaskCreate( vTask3, "Task 3", 240, NULL, 1, NULL );
	xTaskCreate( vTask4, "Task 4", 240, NULL, 1, NULL );

	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	for( ;; );
	return 0;
}
/*-----------------------------------------------------------*/

void vTask1( void *pvParameters )
{
const char *pcTaskName = "Task 1 is running\n";
volatile unsigned long ul;

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		vPrintString( pcTaskName );

		/* Delay for a period. */
		for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
		{
			/* This loop is just a very crude delay implementation.  There is
			nothing to do in here.  Later exercises will replace this crude
			loop with a proper delay/sleep function. */
		}
	}
}
/*-----------------------------------------------------------*/
void vTask3( void *pvParameters )
{
	int j;
	LPC_GPIO2 -> FIODIR |= 1 << 11; //direction initialization
	SSP_init();
	uint8_t sendBuf[FIFOSIZE], recBuf[FIFOSIZE];
	//Write on CTRL1 Register (address = 0x20)
	//Enable X, Y, Z Accelerometer
	SSPSend(0x20, 0x27);
	printf("CTRL1 > %x\n", SSPReceive(0x20));
	//Write on CTRL3 Register (address = 0x23)
	SSPSend(0x23, 0x40);
	printf("CTRL4 > %x\n", SSPReceive(0x23));

	SSPSend(0x25, 0x00);
	printf("CTRL6 > %x\n", SSPReceive(0x25));

	int16_t accX, accY, accZ;
	int16_t oldy=1400;
	int16_t result=0;
	uint8_t ACC_Data[6];

	// Force the counter to be placed into memory
	volatile static int i = 0 ;
	// Enter an infinite loop, just incrementing a counter

	int steppermotor = 0;
	while(1)
	{
		ACC_Data[0] = SSPReceive(0x28);
		ACC_Data[1] = SSPReceive(0x29);
		ACC_Data[2] = SSPReceive(0x2A);
		ACC_Data[3] = SSPReceive(0x2B);
		ACC_Data[4] = SSPReceive(0x2C);
		ACC_Data[5] = SSPReceive(0x2D);
		accX = (int)(ACC_Data[1] << 8) | ACC_Data[0];
		accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
		accZ = (int)(ACC_Data[5] << 8) | ACC_Data[4];

		//printf("accX > %d ", accX);
		//printf("accY > %d ", accY);
		//printf("accZ > %d\n", accZ);

		result=accY-oldy;
		init_PWM(0,0,10000);
		if(abs(result) > 1000)
		{
			while(accY > 3600)
			{
				//If the bicycle is tipping to the right, we want the motor to constantly spin counterclockwise for
				//counter weight until we reach our limit
				//the reason we have this if loop is because we only want the stepper motor to take 100 microsteps
				if(steppermotor < 2) //this limit is arbitrary
				{
					++steppermotor;
					LPC_GPIO2 -> FIOSET |= 1 << 11; //Making the motor spin left
					init_PWM(0,50,10000); //the higher the frequency the slower the motor turns
					sleep_us(1);
					init_PWM(0,50,10000); //the left number in init_PWM is duty cycle and the right is frequency
					ACC_Data[3] = SSPReceive(0x2B);
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
					printf("steppermotor: %d\n", steppermotor);
					printf("accX > %d ", accX);
					printf("accY > %d ", accY);
					printf("accZ > %d\n", accZ);
				}
				else
				{
					init_PWM(0,0,10000);
					//We still want the data from the accelerometer to print out
					ACC_Data[3] = SSPReceive(0x2B);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
					printf("steppermotor: %d\n", steppermotor);
					printf("accX > %d ", accX);
					printf("accY > %d ", accY);
					printf("accZ > %d\n", accZ);
				}
			}
			//The reason we don't reset steppermotor variable is because if the pendulum system is tipping left,
			//we have to go more than 100 steps to hit out limit on the left.
			while(accY < -900)
			{
				//If the bicycle is tipping to the left, we want the motor to constantly spin clockwise for counter weight
				//until we reach our limit
				//the reason we have this if loop is because we only want the stepper motor to take 100 microsteps
				if(steppermotor > -2)
				{
					--steppermotor;
					LPC_GPIO2 -> FIOCLR |= 1 << 11; //Making the motor spin right
					//the left number in init_PWM is duty cycle and the right is frequency
					init_PWM(0,50,10000);   //the higher the frequency the slower the motor turns
					sleep_us(1); //we sleep a given amount of time to make sure the program doesn't change anything for that time
					init_PWM(0,50,10000);
					ACC_Data[3] = SSPReceive(0x2B); //We are taking data from the SSP setup accelerometer and putting it into data
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
					printf("steppermotor: %d\n", steppermotor);
					printf("accX > %d ", accX); //We decided to print here to make sure the accelerometer is still working.
					printf("accY > %d ", accY);
					printf("accZ > %d\n", accZ);
				}
				else
				{
					//We still want the data from the accelerometer to print out
					init_PWM(0,0,10000);
					ACC_Data[3] = SSPReceive(0x2B);
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
					printf("steppermotor: %d\n", steppermotor);
					printf("accX > %d ", accX);
					printf("accY > %d ", accY);
					printf("accZ > %d\n", accZ);
				}
			}
			while((accY >= -900) &&  (accY <= 3600) && (steppermotor != 0))
			{
				//If the bicycle is not tipping at all, we want to move back to being balanced.
				//the reason we have this if loop is because we only want the stepper motor to take 100 microsteps
				if(steppermotor > 0)
				{
					--steppermotor;
					LPC_GPIO2 -> FIOCLR |= 1 << 11; //Making the motor spin right
					//the left number in init_PWM is duty cycle and the right is frequency
					init_PWM(0,50,10000);   //the higher the frequency the slower the motor turns
					sleep_us(1); //we sleep a given amount of time to make sure the program doesn't change anything for that time
					init_PWM(0,50,10000);
					ACC_Data[3] = SSPReceive(0x2B); //We are taking data from the SSP setup accelerometer and putting it into data
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
					printf("steppermotor: %d\n", steppermotor);
					printf("accX > %d ", accX); //We decided to print here to make sure the accelerometer is still working.
					printf("accY > %d ", accY);
					printf("accZ > %d\n", accZ);
				}
				else if(steppermotor < 0)
				{
					++steppermotor;
					LPC_GPIO2 -> FIOSET |= 1 << 11; //Making the motor spin left
					init_PWM(0,50,10000); //the higher the frequency the slower the motor turns
					sleep_us(1);
					init_PWM(0,50,10000); //the left number in init_PWM is duty cycle and the right is frequency
					ACC_Data[3] = SSPReceive(0x2B);
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
					printf("steppermotor: %d\n", steppermotor);
					printf("accX > %d ", accX);
					printf("accY > %d ", accY);
					printf("accZ > %d\n", accZ);
				}
			}
		}
	}
	return;
}
int mtr = 0;
/*-----------------------------------------------------------*/
void vTask4( void *pvParameters )
{

	LPC_SC->PCONP |= 1<<24;
	LPC_SC->PCLKSEL1 &= ~(3<<16);
	LPC_SC->PCLKSEL1 |= 1<<16;
	uint16_t DL = SystemCoreClock / (16*38400);
	LPC_UART2->LCR |= 1<<7;
	LPC_UART2->DLL = DL&(0xFF); //lower 8 bits of DL
	LPC_UART2->DLM = DL>>8; //upper 8 bits of DL
	//Disable FIFO
	LPC_UART2->FCR &= 0;

	//Set 8-bit exchange
	LPC_UART2->LCR &= ~(3);
	LPC_UART2->LCR |= 3;

	LPC_PINCON->PINSEL0 &= ~(15<<20);
	LPC_PINCON->PINSEL0 |= 15<<20;

	//DLAB = 0
	LPC_UART2->LCR &= ~(1<<7);
	//disable the divisor latch

	//GPIO Init
 	LPC_PINCON->PINSEL1 &= ~(0x63<<14);

	LPC_GPIO0->FIODIR |= 0x7<<23;

	while(1)
	{
		uint8_t char_in;
		printf("Your baud rate is %d", DL);
		//PWM motor_adj(PWM::pwm1,100);
		//init_PWM(1,50,100); //the higher the frequency the slower the motor turns
		char_in = read();
		printf("Read: %c, %i\n", char_in, mtr);
		// if(char_in == 'M')
		// {
		// 	LPC_GPIO0->FIOSET |= 1<<23;

		// }
		// if(char_in == 'C')
		// {
		// 	LPC_GPIO0->FIOCLR |= 1<<23;
		// }
		// if(char_in == 'L')
		// {
		// 	if(mtr > -45)
		// 	{
		// 		mtr=mtr-1;
		// 		LPC_GPIO0->FIOCLR |= 1<<24;
		// 		int m;
		// 		for(m = 0; m < 5; m=m+1)
		// 		{
		// 			LPC_GPIO0->FIOCLR |= 1<<25;
		// 			vTaskDelay(1);
		// 			LPC_GPIO0->FIOSET |= 1<<25;
		// 			vTaskDelay(1);
		// 		}
		// 	}
		// }
		// else if(char_in == 'R')
		// {
		// 	if(mtr < 45)
		// 	{
		// 		mtr=mtr+1;
		// 		LPC_GPIO0->FIOSET |= 1<<24;
		// 		int d;
		// 		for(d = 0; d < 5; d = d+1)
		// 		{
		// 			LPC_GPIO0->FIOCLR |= 1<<25;
		// 			vTaskDelay(1);
		// 			LPC_GPIO0->FIOSET |= 1<<25;
		// 			vTaskDelay(1);
		// 		}
		// 	}
		// }
	}

}

uint8_t read(void)
{
  while(!(LPC_UART2->LSR & 1));
  return LPC_UART2->RBR;
}

/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}

void SSP_init()
{
	uint8_t dummy;

	printf("SSP Init\n");
	//Power the SPP0 Peripheral
	LPC_SC -> PCONP |= 1 << 21;

	//Divide the SSP0 clock by 8
	LPC_SC -> PCLKSEL1 |= (1 << 10) | (1<<11);

	//Configure P0.15 to SPP0 CLK pin
	LPC_PINCON -> PINSEL0 |= 1 << 31;

	/* Configure P0.16 to SSEL   */
	//LPC_PINCON -> PINSEL1 |= 1 << 1;
	LPC_GPIO0 -> FIODIR |= 1 << 16;
	LPC_GPIO0 -> FIOSET |= 1 << 16;

	//Configure P0.17 to MISO0
	LPC_PINCON -> PINSEL1 |= 1 << 3;

	//Configure P0.18 to MOSI0
	LPC_PINCON -> PINSEL1 |= 1 << 5;


	//PULL DOWN
	LPC_PINCON -> PINMODE0 |= 0x3 << 30;
	LPC_PINCON -> PINMODE1 |= (0x3) | (0x3 << 2) | (0x3<<4);

	//SPP0 work on Master Mode and SSP enable
	LPC_SSP0 -> CR1 |= (1 << 1);

	/*
	 * SSP0 Control Register 0
	 * Set DSS data to 8-bit	(7 at 3:0)
	 * frame format SPI			(00 at 5:4)
	 * CPOL =0, CPHA=0, and SCR is 15
	 */
	LPC_SSP0 -> CR0 |= 0x0707;

	/* SSPCPSR clock prescale register, master mode, minimum divisor is 0x02*/
	LPC_SSP0 -> CPSR |= 0x5E;

	int i=0;
	/* Clear RxFIFO */
	for(i=0; i<FIFOSIZE; i++){
		dummy = LPC_SSP0 -> DR;
	}


}

void SSPSend(uint8_t address, uint8_t buf)
{

	SSP_SSEL(0,0);

	//Send Address of the register to write on
	//Move only if is not busy and TX FIFO is NOT FULL
	while(is_busy() & !is_Tx_not_full());
	LPC_SSP0 -> DR = (address);
	while(is_busy());
	uint8_t dummy= LPC_SSP0 -> DR;
	//SSP_SSEL(0,1);
	//SSP_SSEL(0,0);

	//Send Data
	while(is_busy() & !is_Tx_not_full());
	LPC_SSP0 -> DR = buf;

	while(is_busy());
	dummy = LPC_SSP0 -> DR;

	SSP_SSEL(0,1);
}


uint8_t SSPReceive(uint8_t address)
{
	uint8_t dummy, data;
	address |= 1 << 7; //READ Bit
	int i=0;
	SSP_SSEL(0,0);

	LPC_SSP0 -> DR = address;
	//Read only if it's not busy and the receiver FIFO is not empty
	while(is_busy() & !is_Rx_not_empty());
	dummy = LPC_SSP0 -> DR;
	LPC_SSP0 -> DR = 0xff;
	while(is_busy() & !is_Rx_not_empty());
	data = LPC_SSP0->DR;

	SSP_SSEL(0,1);

	return data;

}

/* Description : Manual set for SSP Chip Select (CS) */
void SSP_SSEL(int port, int toggle)
{
	if(port == 0){
		if(!toggle)
			LPC_GPIO0 -> FIOCLR |= 1<<16;
		else
			LPC_GPIO0 -> FIOSET |= 1<<16;
	}
	else if(port == 1){
		if(!toggle)
			LPC_GPIO1 -> FIOCLR |= 1<<6;
		else
			LPC_GPIO1 -> FIOSET |= 1<<6;
	}
}

/* Status Register
 * Bit	Symbol		Description
 * 0	TFE			Transmit FIFO empty
 * 1	TNF			Transmit FIFO Not Full
 * 2	RNE			Receiver FIFO Not Empty
 * 3	RFF			Receiver FIFO Full
 * 4	BSY			Busy sending/receiving
 */


/* returns 1 if Transmitter FIFO Not FULL
 * returns 0 if Transmitter FIFO is FULL
 */
int is_Tx_not_full()
{
	uint32_t reg = LPC_SSP0 -> SR;
	return ((reg & 0x2) >> 1);
}

/* returns 1 if Transmitter FIFO empty
 * returns 0 if Transmitter FIFO is not empty
 */
int is_Tx_empty()
{
	uint32_t reg = LPC_SSP0 -> SR;
	return (reg & 0x1);
}

/* returns 1 if Receiver FIFO FULL
 * returns 0 if Receiver FIFO is NOT FULL
 */
int is_Rx_full()
{
	uint32_t reg = LPC_SSP0 -> SR;
	return ((reg & 0x4) >> 2);
}

/* returns 1 if Receiver FIFO Not empty
 * returns 0 if Receiver FIFO is empty
 */
int is_Rx_not_empty(){
	uint32_t reg = LPC_SSP0 -> SR;
	return (reg & (1 << 2)) >> 2;
}

int is_busy(){
	uint32_t reg = LPC_SSP0 -> SR;
	return ((reg & (1<<4)) >> 4);
}




void sleep_us (int us)
{
    volatile int    i;
    int US_TIME= SystemCoreClock/100000;
    while (us--) {
        for (i = 0; i < US_TIME; i++) {
            ;    /* Burn cycles. */
        }
    }
    return 0;
}

void init_PWM(uint32_t PWM_num, uint32_t PWMinval, uint32_t speed)
{
	//Power for PWM1, sets 6th bit of PCONP register to 1 which is PWM1;
		//PCONP register is peripheral power control register
		LPC_SC->PCONP |= (1 << 6);
		//peripheral clock select for PWM cclk/8
		LPC_SC->PCLKSEL0 |= ((1 << 13) | (1 << 12));
		//pin select
		if(0 < PWM_num < 4)
		{
			LPC_PINCON->PINSEL4 |= (0x1<<PWM_num*2); //sets 01 for bits [1:0] of PINSEL4 register, which sets P2.0 to PWM1.1 operation
		}
		LPC_PWM1->TCR = (1<<1);   	//counter reset, set 2nd bit of TCR register to 1;
		LPC_PWM1->PR = 0;        //count frequency, prescale register
		LPC_PWM1->PC = 0;		//prescale counter
		LPC_PWM1->MCR = (1 << 1); //reset TC on Match 0
		//set period & duty cycle
		LPC_PWM1->MR0 = speed;	  //set PWM period/cycle to 1khz
		LPC_PWM1->MR1 = PWMinval;	  //set 50% duty cycle; period/2
		//write enable for match registers
		LPC_PWM1->LER = (1<<0)|(1<<1); //latch MR0 and MR1 (must be used for those registers to be overwritten)
		//pwm enable, settings
		LPC_PWM1->PCR = (1<<9);   //PWM output enable, single-edged operation, must be set else otherwise PWM is a counter
		LPC_PWM1->TCR = (1<<0) | (1<<3); //TC enable, PWM enable

}
