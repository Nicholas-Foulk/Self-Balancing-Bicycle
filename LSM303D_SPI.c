/*
===============================================================================
 Name        : LSM303D.c
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
void SPI_init();
uint8_t readSPI(uint8_t address);
void writeSPI(uint8_t address, uint8_t data);
// TODO: insert other definitions and declarations here

int main(void) {

	SPI_init();
	LPC_GPIO0 -> FIODIR |= 1<<23;

	int j;


	uint8_t dummy;


	//Write on CTRL1 Register (address = 0x20)
	//Enable X, Y, Z Accelerometer
	writeSPI(0x20, 0x27);

	//Write on CTRL3 Register (address = 0x23)
	writeSPI(0x23, 0x40);


	uint8_t ACC_Data[6];


	uint32_t statusReg;
    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {


    	//LPC_SPI -> SPDR = 0x28;
    	ACC_Data[0] = readSPI(0x28);
    	printf("ACC_Data[0] > %x\n", ACC_Data[0]);

    	ACC_Data[1] = readSPI(0x29);
    	printf("ACC_Data[1] > %x\n", ACC_Data[1]);


    	ACC_Data[2] = readSPI(0x2A);
    	printf("ACC_Data[2] > %x\n", ACC_Data[2]);

    	ACC_Data[3] = readSPI(0x2B);
    	printf("ACC_Data[3] > %x\n", ACC_Data[3]);


    	ACC_Data[4] = readSPI(0x2C);
    	printf("ACC_Data[4] > %x\n", ACC_Data[4]);


    	ACC_Data[0] = readSPI(0x2D);
    	printf("ACC_Data[5] > %x\n", ACC_Data[5]);


    }
    return 0 ;
}


void SPI_init(){

	printf("SPI Init\n");
	//Power the SPI Peripheral
    LPC_SC -> PCONP |= 1 << 8;

    //Divide the system clock by 4
    LPC_SC -> PCLKSEL0 |= (1 << 17);

    //Configure P0.15 to SPI CLK pin
    LPC_PINCON -> PINSEL0 |= 1 << 31 | 1 << 30;

    //Configure P0.16 to SSEL
    //LPC_PINCON -> PINSEL1 |= 1 << 1 | 1 << 0;

    //Configure P0.17 to MISO
    LPC_PINCON -> PINSEL1 |= 1 << 2 | 1 << 3;

    //Configure P0.18 to MOSI
    LPC_PINCON -> PINSEL1 |= 1 << 5 | 1 << 4;

    //PULL DOWN
    LPC_PINCON -> PINMODE0 |= 0x3 << 30;
    LPC_PINCON -> PINMODE1 |= (0x3) | (0x3 << 2) | (0x3<<4);

    //SPI work on Master Mode
    LPC_SPI -> SPCR = 0x30;
    LPC_SPI -> SPCR &= ~(0xF << 8);

    printf("SPCR - > %x\n", LPC_SPI -> SPCR);

    //SPI Clock Peripheral
    LPC_SPI -> SPCCR = 250;


}

uint8_t readSPI(uint8_t address){
	int i=0;

	address |= 1 << 7; //READ bit

	LPC_GPIO0 -> FIOCLR |= 1 << 23;
	LPC_SPI -> SPDR = address;
	while((LPC_SPI->SPSR & 0x80) != 0x80);
	uint8_t datain = LPC_SPI -> SPDR;
	LPC_GPIO0 -> FIOSET |= 1 << 23;
	printf("Datain = %x\n", datain);
	//printf("status = %x\n", LPC_SPI->SPSR);
	return datain;
}

void writeSPI(uint8_t address, uint8_t data){
	int i;
	LPC_GPIO0 -> FIOCLR |= 1 << 23;
	//LPC_SPI -> SPDR = (address << 8) | data;
	LPC_SPI -> SPDR = (address);
	while((LPC_SPI->SPSR & 0x80) != 0x80);
	LPC_SPI -> SPDR = data;
	uint8_t dummy = LPC_SPI -> SPDR;
	while((LPC_SPI->SPSR & 0x80) != 0x80);
	LPC_GPIO0 -> FIOSET |= 1 << 23;
	printf("dummy = %x\n", dummy);
	//printf("status = %x\n", LPC_SPI->SPSR);

}
