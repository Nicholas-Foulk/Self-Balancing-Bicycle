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
#include "PWM.h"
#include "StepperMotor.h"

/* Demo includes. */
#include "basic_io.h"
#include "stdlib.h"
#include "math.h"
#include <cr_section_macros.h>
#include <stdio.h>

/* Defines to make boolean values easier to assign*/
#define TRUE 1
#define FALSE 0

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xfffff )

/*Added definitions from Control Algorithm Section*/
#define SSP_BUFSIZE	16
#define FIFOSIZE	8

/* The task functions. */
void vTask1(void *pvParameters);
void MainBalanceTask(void *pvParameters);
void BluetoothTask(void *pvParameters);
void vTask5(void *pvParameters);

/*Globals for inter-task communication*/
int cent = 0;
int mtr = 0;
int spd = 0;
int on = FALSE;
int left = FALSE;
int right = FALSE;

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

/*-------------------------- ---------------------------------*/


int main(void)

{
	/* Init the semi-hosting. */

	xTaskCreate(MainBalanceTask, "MainBalanceTask", 720, NULL, 1, NULL );
	xTaskCreate(BluetoothTask, "BluetoothTask", 240, NULL, 1, NULL );
	xTaskCreate(vTask5, "Task 5", 1080, NULL, 1, NULL);
	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	 running.  If we do reach here then it is likely that there was insufficient
	 heap available for the idle task to be created. */
	for (;;)
		;
	return 0;
}

/*-----------------------------------------------------------*/
void mainBalanceTask(void *pvParameters)
{
	LPC_GPIO2->FIODIR |= 1 << 11; //direction initialization
	SSP_init();
	uint8_t sendBuf[FIFOSIZE], recBuf[FIFOSIZE];
	//Write on CTRL1 Register (address = 0x20)
	//Enable X, Y, Z Accelerometer
	SSPSend(0x20, 0x27);
//	printf("CTRL1 > %x\n", SSPReceive(0x20));
	//Write on CTRL3 Register (address = 0x23)
	SSPSend(0x23, 0x40);
//	printf("CTRL4 > %x\n", SSPReceive(0x23));

	SSPSend(0x25, 0x00);
//	printf("CTRL6 > %x\n", SSPReceive(0x25));

	int16_t accX, accY, accZ;
	uint8_t ACC_Data[6];

	stepperInit(2, 0);
	
	/*
	 * QEI initial pin/register configuration
	 */
	LPC_SC->PCONP |= 1 << 18;
	LPC_SC->PCLKSEL1 |= (1 << 0) | (1 << 1);
	LPC_PINCON->PINSEL3 |= 1 << 8;   //P1.20  MCI0
	LPC_PINCON->PINSEL3 |= 1 << 14;  //P1.23  MCI1
	LPC_PINCON->PINSEL3 |= 1 << 16;  //P1.24  MCI2

	//reset all counters
	LPC_QEI->QEICON |= (0xF << 0);
	//turn off signal mode
	LPC_QEI->QEICONF &= ~(1 << 1);

	//new initialization registers
	LPC_QEI->CMPOS1 = 0x00;
	LPC_QEI->CMPOS2 = 0x00;
	LPC_QEI->INXCMP = 0x00;
	//LPC_QEI->QEILOAD = 0x00;
	LPC_QEI->VELCOMP = 0x00;
	LPC_QEI->QEIIES |= 0x01;
	LPC_QEI->QEICLR |= 0x41;
	LPC_QEI->QEICONF |= 0x04; //Cap x4

	LPC_QEI->QEIMAXPOS = 0x50;

	LPC_QEI->FILTER = 0x100;
	LPC_QEI->QEICONF |= 0x1;
	 /*
	  * End of the QEI initialization section
	  */

	int calc_target()
	{
		int c;
		int summation = 0;

		for (c = 0; c < 5; c = c + 1)
		{
			ACC_Data[0] = SSPReceive(0x28);
			ACC_Data[1] = SSPReceive(0x29);
			ACC_Data[2] = SSPReceive(0x2A);
			ACC_Data[3] = SSPReceive(0x2B);
			ACC_Data[4] = SSPReceive(0x2C);
			ACC_Data[5] = SSPReceive(0x2D);
			accX = (int) (ACC_Data[1] << 8) | ACC_Data[0];
			accY = (int) (ACC_Data[3] << 8) | ACC_Data[2];
			accZ = (int) (ACC_Data[5] << 8) | ACC_Data[4];
			summation = summation + accY;
		}
		return -((summation/5) - 2600);
	}
	
	/*
	 * Here are our declarations for the PID control loop
	 */
	 
	int mean = 0;
	mean = calc_target();

	int turnoffset = 800;
	int baloffset = 50;
	int center = 0;

	/*Variables for steering*/
//	int steer_derivative = 0;
//	int steer_integral = 0;
//	int steer_CV = 0;
//	int steer_error = 0;
//	int steer_last_error = mean;
//	int steer_pos = 0;
//	int steer_targetpos = 0;
//	int steer_Kp = 4;  //constant variable used for multiplying error
//	int steer_Ki = 1;  //constant variable used for multiplying integral
//	int steer_Kd = 2;  //constant variable used for multiplying derivative

	/*Variables for balancing mechanism*/

	int pos = 0;
	int last_error = mean;
	int targetpos = mean;  //out set target position
	int integral = 0;
	int derivative = 0;
	int error = 0;
	int CV = 0;
	int limit = 0;
	int Kp = 4;  //constant variable used for multiplying error
	int Ki = 1;  //constant variable used for multiplying integral
	int Kd = 2;  //constant variable used for multiplying derivative
	
	/*
	 * Here is the end of our declarations for the PID control loop
	 */	

	while (1)
	{
		ACC_Data[0] = SSPReceive(0x28);
		ACC_Data[1] = SSPReceive(0x29);
		ACC_Data[2] = SSPReceive(0x2A);
		ACC_Data[3] = SSPReceive(0x2B);
		ACC_Data[4] = SSPReceive(0x2C);
		ACC_Data[5] = SSPReceive(0x2D);
		//accX = (int)(ACC_Data[1] << 8) | ACC_Data[0];
		accY = (int) (ACC_Data[3] << 8) | ACC_Data[2];
		//accZ = (int)(ACC_Data[5] << 8) | ACC_Data[4];

		accY = -(accY - 2600);

		if (cent == TRUE)	//reset all values when motor is re-centered
		{
			targetpos = 0;
			mean = calc_target();
			cent = FALSE;
			error = 0;
			limit = 0;
			CV = 0;
			integral = 0;
			LPC_QEI->QEICON |= 1;
		}


		if(LPC_QEI->QEIPOS > 40)
		{
			pos = (-80+LPC_QEI->QEIPOS); //Position is value from 0-80, translates this value to 0-40
		}
		else
		{
			pos = LPC_QEI->QEIPOS; //Retreive position
		}
		if(targetpos > 40)
		{
			error = pos - (80 - targetpos); //target position - current position
		}
		else if(targetpos < 40)
		{
			error = pos - targetpos; //target position - current position
		}

		derivative = error - last_error; //derivative
		integral = integral + error;         //integral portion of the algorithm
		CV = (error * Kp) + (integral * Ki) + (derivative * Kd); //Control variable
		CV = abs(CV/50);

		if(CV < 10)
		{
			CV = 10;	//Max speed
		}
		else if(CV > 50)
		{
			CV = 50;	//Min speed
		}

		if (accY <= mean-turnoffset && limit > -50)
		{
			center = FALSE;
			limit = limit - 1;
			stepperTurnF(2, 0, 2, 11, steer_CV, 5);
		}
		else if (accY >= mean+turnoffset && limit < 50)
		{
			center = FALSE;
			limit = limit + 1;
			stepperTurnR(2, 0, 2, 11, steer_CV, 5);

		}
		else if ((mean-baloffset) < accY && accY < (mean+baloffset) && center == FALSE)
		{
			targetpos = 0;
			while (limit != 0)
			{

				if (limit > 0)
				{
					limit = limit - 1;
					stepperTurnF(2, 0, 2, 11, steer_CV, 5);
					vTaskDelay(10);
				}
				else if (limit < 0)
				{
					limit = limit + 1;
					stepperTurnR(2, 0, 2, 11, steer_CV, 5);
					vTaskDelay(10);
				}
			}
			center = TRUE;
			vTaskDelay(10);
		}
		else if(steer_pos == steer_targetpos)
		{
			error = 0;
			CV = 0;
			last_error = 0;
			integral = 0;
		}
		else
		{
			//do nothing
		}
		last_error = error;


/*		Steering code
		if((mean-400) <= accY && accY <= (mean+400))
		{
//			steer_targetpos = 0;
//			center = TRUE;
//			vTaskDelay(10);
		}
		else if (accY < mean-200)
		{
			center = FALSE;
			limit = limit - 1;
			stepperTurnR(0, 25, 0, 24, steer_CV, 5);
			steer_targetpos = -20;
		}
		else if (accY > mean+200)
		{
			center = FALSE;
			limit = limit + 1;
			stepperTurnF(0, 25, 0, 24, steer_CV, 5);
			steer_targetpos = 20;
		}
		else if ((mean-200) <= accY && accY <= (mean+200))
		{
			steer_targetpos = 0;
			while (limit != 0)
			{

				if (limit > 0)
				{
					limit = limit - 1;
					stepperTurnF(2, 0, 2, 11, steer_CV, 5);
//					stepperTurnR(0, 25, 0, 24, 50, 5);
					vTaskDelay(10);
				}
				else if (limit < 0)
				{
					limit = limit + 1;
					stepperTurnR(2, 0, 2, 11, steer_CV, 5);
//					stepperTurnF(0, 25, 0, 24, 50, 5);
					vTaskDelay(10);
				}
			}
			center = TRUE;
			vTaskDelay(10);
		}
		else
		{
			steer_error = 0;
			steer_CV = 0;
			steer_last_error = 0;
			steer_integral = 0;
		}

		*/

//		if(steer_targetpos != steer_pos)
//		{
//			if((steer_targetpos < 0) )
//			{
//				if(steer_pos < steer_targetpos)
//				{
//					stepperTurnR(0, 25, 0, 24, steer_CV, 5);
//				}
//				else
//				{
//					stepperTurnF(0, 25, 0, 24, steer_CV, 5);
//				}
//			}
//			else if((steer_targetpos > 0))
//			{
//				if(steer_pos > steer_targetpos)
//				{
//					stepperTurnF(0, 25, 0, 24, steer_CV, 5);
//				}
//				else
//				{
//					stepperTurnR(0, 25, 0, 24, steer_CV, 5);
//				}
//			}
//			else if(steer_targetpos == 0)
//			{
//				if(steer_pos < 0)
//				{
//					stepperTurnR(0, 25, 0, 24, steer_CV, 5);
//				}
//				else if (steer_pos > 0)
//				{
//					stepperTurnF(0, 25, 0, 24, steer_CV, 5);
//				}
//			}
//		}
//		printf("%i, %i\n", steer_pos, steer_targetpos);

//		steer_last_error = steer_error;
	}

	return;
}

/*-----------------------------------------------------------*/
void BluetoothTask(void *pvParameters)
{

//==========Bluetooth Init==========
	LPC_SC->PCONP |= 1 << 24;
	LPC_SC->PCLKSEL1 &= ~(3 << 16);
	LPC_SC->PCLKSEL1 |= 1 << 16;
	uint16_t DL = SystemCoreClock / (16 * 38400); //set baud rate to 38400
	LPC_UART2->LCR |= 1 << 7;
	LPC_UART2->DLL = DL & (0xFF); //lower 8 bits of DL
	LPC_UART2->DLM = DL >> 8; //upper 8 bits of DL
	//Disable FIFO
	LPC_UART2->FCR &= 0;

	//Set 8-bit exchange
	LPC_UART2->LCR &= ~(3);
	LPC_UART2->LCR |= 3;

	LPC_PINCON->PINSEL0 &= ~(15 << 20);
	LPC_PINCON->PINSEL0 |= 5 << 20;

	//DLAB = 0
	LPC_UART2->LCR &= ~(1 << 7);
	//disable the divisor latch
//==========End Bluetooth Init==========

	//GPIO Init 0.23,0.24,0.25
	LPC_PINCON->PINSEL1 &= ~(0x63 << 14);
	LPC_GPIO0->FIODIR |= 0x7 << 23;

	initPWM(1, 60, 100);
	while (1)
	{
		uint8_t char_in;
		if (on)
		{
			setPWMspeed(1, spd);
		}
		else
		{
			setPWMspeed(1, 60);
		}

		char_in = read();

		if (char_in == 'G')
		{
			spd = 60;
			on = TRUE;
		}
		else if (char_in == 'S')
		{
			spd = 60;
			on = FALSE;
		}
		else if (char_in == 'U' && on && spd < 100)
		{
			spd = spd + 2;
		}
		else if (char_in == 'D' && on && spd > 60)
		{
			spd = spd - 2;
		}
		else if (char_in == 'L')
		{
			mtr = mtr - 1;
			stepperTurnF(0, 25, 0, 24, 50, 25);
		}
		else if (char_in == 'R')
		{
			mtr = mtr + 1;
			stepperTurnR(0, 25, 0, 24, 50, 25);
		}
		else if (char_in == 'C')
		{
			cent = TRUE;
		}
		else if(char_in == 'O')
		{
			left = TRUE;
		}
		else if(char_in == 'P')
		{
			right = TRUE;
		}

		vTaskDelay(10);
	}

}


void vTask5(void *pvParameters) //task to debug QEI
{
	/*
	 * Here is the QEI initialization
	 */
	LPC_GPIO2->FIODIR |= 1 << 11; //direction initialization
	int current = 0;
	stepperInit(2, 0);
	LPC_SC->PCONP |= 1 << 18;
	LPC_SC->PCLKSEL1 |= (1 << 0) | (1 << 1);
	LPC_PINCON->PINSEL3 |= 1 << 8;   //P1.20  MCI0
	LPC_PINCON->PINSEL3 |= 1 << 14;  //P1.23  MCI1
	LPC_PINCON->PINSEL3 |= 1 << 16;  //P1.24  MCI2

	//reset all counters
	LPC_QEI->QEICON |= (0xF << 0);
	//turn off signal mode
	LPC_QEI->QEICONF &= ~(1 << 1);

	//new initialization registers
	LPC_QEI->CMPOS1 = 0x00;
	LPC_QEI->CMPOS2 = 0x00;
	LPC_QEI->INXCMP = 0x00;
	//LPC_QEI->QEILOAD = 0x00;
	LPC_QEI->VELCOMP = 0x00;
	LPC_QEI->QEIIES |= 0x01;
	LPC_QEI->QEICLR |= 0x41;
	LPC_QEI->QEICONF |= 0x04; //Cap x4

	LPC_QEI->QEIMAXPOS = 0x50;

	//LPC_QEI->QEILOAD = SystemCoreClock / 4;

	LPC_QEI->FILTER = 0x100;
	LPC_QEI->QEICONF |= 0x1;
	int steer_derivative = 0;
	int steer_integral = 0;
	int steer_CV = 0;
	int steer_error = 0;
	int steer_last_error = 0;
	int steer_pos = 0;
	int steer_targetpos = 0;
	int Kp = 4;
	int Kd = 2;
	int Ki = 1;


	while (1)
	{

		if(LPC_QEI->QEIPOS > 40)
		{
			steer_pos = (-80+LPC_QEI->QEIPOS); //Retreive position
		}
		else
		{
			steer_pos = LPC_QEI->QEIPOS; //Retreive position
		}

		if(right)
		{
//			if(steer_targetpos < -20)
//			{
				steer_targetpos = 2;
//			}
			right = FALSE;
//			printf("%i, %i\n", steer_pos, steer_targetpos);
		}
		else if(left)
		{
//			if(steer_targetpos > 20)
//			{
				steer_targetpos = -2;
			//}
//			printf("%i, %i\n", steer_pos, steer_targetpos);
			left = FALSE;
		}
		else if(cent)
		{
			steer_targetpos = 0;
//			printf("%i, %i\n", steer_pos, steer_targetpos);
			cent = FALSE;
		}

		//Steering PID Controller
		if(steer_pos != steer_targetpos)
		{
			if(steer_targetpos > 0)
			{
				steer_error = steer_pos - (5 - steer_targetpos); //target position - current position
			}
			else if(steer_targetpos < 0)
			{
				steer_error = steer_pos - steer_targetpos; //target position - current position
			}

			steer_derivative = steer_error - steer_last_error; //derivative
			steer_integral = steer_integral + steer_error;         //integral portion of the algorithm
			steer_CV = (steer_error * Kp) + (steer_integral * Ki) + (steer_derivative * Kd); //Control variable
			steer_CV = abs(1000/steer_CV);

			if(steer_CV < 75)
			{
				steer_CV = 75;
			}
			else if(steer_CV > 500)
			{
				steer_CV = 500;
			}

//			printf("%i | %i\n", steer_error, steer_CV);

			if((steer_targetpos < 0) )
			{
				if(steer_pos < steer_targetpos)
				{
					stepperTurnR(2, 0, 2, 11, steer_CV, 1);
//					stepperTurnR(0, 25, 0, 24, steer_CV, 5);
				}
				else
				{
					stepperTurnF(2, 0, 2, 11, steer_CV, 1);
//					stepperTurnF(0, 25, 0, 24, steer_CV, 5);
				}
			}
			else if((steer_targetpos > 0))
			{
				if(steer_pos > steer_targetpos)
				{
					stepperTurnF(2, 0, 2, 11, steer_CV, 1);
//					stepperTurnF(0, 25, 0, 24, steer_CV, 5);
				}
				else
				{
					stepperTurnR(2, 0, 2, 11, steer_CV, 1);
//					stepperTurnR(0, 25, 0, 24, steer_CV, 5);
				}
			}
			else
			{
				if(steer_pos < 0)
				{
					stepperTurnR(2, 0, 2, 11, steer_CV, 1);
//					stepperTurnR(0, 25, 0, 24, steer_CV, 5);
				}
				else if (steer_pos > 0)
				{
					stepperTurnF(2, 0, 2, 11, steer_CV, 1);
//					stepperTurnF(0, 25, 0, 24, steer_CV, 5);
				}
			}
			steer_last_error = steer_error;
			vTaskDelay(10);
		}
		else
		{
			steer_error = 0;
			steer_CV = 0;
			steer_last_error = 0;
			steer_integral = 0;
		}
	}

}
uint8_t read(void)
{
	while (!(LPC_UART2->LSR & 1))
		;
	return LPC_UART2->RBR;
}
static void EINT3_ISR(void)
{
	if (LPC_GPIOINT->IO2IntStatF & (1 << 6))
	{
		LPC_GPIOINT->IO2IntClr |= (1 << 6); //table 123,
		return;
	}
	else if (LPC_GPIOINT->IO2IntStatF & (1 << 6))
	{
		LPC_GPIOINT->IO2IntClr |= (1 << 7); //table 123,
		return;
	}

}
/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook(void) {
	/* This function will only be called if an API call to create a task, queue
	 or semaphore fails because there is too little heap RAM remaining. */
	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask,
		signed char *pcTaskName) {
	/* This function will only be called if a task overflows its stack.  Note
	 that stack overflow checking does slow down the context switch
	 implementation. */
	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void) {
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void) {
	/* This example does not use the tick hook to perform any processing. */
}

void SSP_init() {
	uint8_t dummy;

//	printf("SSP Init\n");
	//Power the SPP0 Peripheral
	LPC_SC->PCONP |= 1 << 21;

	//Divide the SSP0 clock by 8
	LPC_SC->PCLKSEL1 |= (1 << 10) | (1 << 11);

	//Configure P0.15 to SPP0 CLK pin
	LPC_PINCON->PINSEL0 |= 1 << 31;

	/* Configure P0.16 to SSEL   */
	//LPC_PINCON -> PINSEL1 |= 1 << 1;
	LPC_GPIO0->FIODIR |= 1 << 16;
	LPC_GPIO0->FIOSET |= 1 << 16;

	//Configure P0.17 to MISO0
	LPC_PINCON->PINSEL1 |= 1 << 3;

	//Configure P0.18 to MOSI0
	LPC_PINCON->PINSEL1 |= 1 << 5;

	//PULL DOWN
	LPC_PINCON->PINMODE0 |= 0x3 << 30;
	LPC_PINCON->PINMODE1 |= (0x3) | (0x3 << 2) | (0x3 << 4);

	//SPP0 work on Master Mode and SSP enable
	LPC_SSP0->CR1 |= (1 << 1);

	/*
	 * SSP0 Control Register 0
	 * Set DSS data to 8-bit	(7 at 3:0)
	 * frame format SPI			(00 at 5:4)
	 * CPOL =0, CPHA=0, and SCR is 15
	 */
	LPC_SSP0->CR0 |= 0x0707;

	/* SSPCPSR clock prescale register, master mode, minimum divisor is 0x02*/
	LPC_SSP0->CPSR |= 0x5E;

	int i = 0;
	/* Clear RxFIFO */
	for (i = 0; i < FIFOSIZE; i++) {
		dummy = LPC_SSP0->DR;
	}

}

void SSPSend(uint8_t address, uint8_t buf) {

	SSP_SSEL(0, 0);

	//Send Address of the register to write on
	//Move only if is not busy and TX FIFO is NOT FULL
	while (is_busy() & !is_Tx_not_full())
		;
	LPC_SSP0->DR = (address);
	while (is_busy())
		;
	uint8_t dummy = LPC_SSP0->DR;
	//SSP_SSEL(0,1);
	//SSP_SSEL(0,0);

	//Send Data
	while (is_busy() & !is_Tx_not_full())
		;
	LPC_SSP0->DR = buf;

	while (is_busy())
		;
	dummy = LPC_SSP0->DR;

	SSP_SSEL(0, 1);
}

uint8_t SSPReceive(uint8_t address) {
	uint8_t dummy, data;
	address |= 1 << 7; //READ Bit
	int i = 0;
	SSP_SSEL(0, 0);

	LPC_SSP0->DR = address;
	//Read only if it's not busy and the receiver FIFO is not empty
	while (is_busy() & !is_Rx_not_empty())
		;
	dummy = LPC_SSP0->DR;
	LPC_SSP0->DR = 0xff;
	while (is_busy() & !is_Rx_not_empty())
		;
	data = LPC_SSP0->DR;

	SSP_SSEL(0, 1);

	return data;

}

/* Description : Manual set for SSP Chip Select (CS) */
void SSP_SSEL(int port, int toggle) {
	if (port == 0) {
		if (!toggle)
			LPC_GPIO0->FIOCLR |= 1 << 16;
		else
			LPC_GPIO0->FIOSET |= 1 << 16;
	} else if (port == 1) {
		if (!toggle)
			LPC_GPIO1->FIOCLR |= 1 << 6;
		else
			LPC_GPIO1->FIOSET |= 1 << 6;
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
int is_Tx_not_full() {
	uint32_t reg = LPC_SSP0->SR;
	return ((reg & 0x2) >> 1);
}

/* returns 1 if Transmitter FIFO empty
 * returns 0 if Transmitter FIFO is not empty
 */
int is_Tx_empty() {
	uint32_t reg = LPC_SSP0->SR;
	return (reg & 0x1);
}

/* returns 1 if Receiver FIFO FULL
 * returns 0 if Receiver FIFO is NOT FULL
 */
int is_Rx_full() {
	uint32_t reg = LPC_SSP0->SR;
	return ((reg & 0x4) >> 2);
}

/* returns 1 if Receiver FIFO Not empty
 * returns 0 if Receiver FIFO is empty
 */
int is_Rx_not_empty() {
	uint32_t reg = LPC_SSP0->SR;
	return (reg & (1 << 2)) >> 2;
}

int is_busy() {
	uint32_t reg = LPC_SSP0->SR;
	return ((reg & (1 << 4)) >> 4);
}

