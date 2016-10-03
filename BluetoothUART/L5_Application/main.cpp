/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "stdio.h"
#include "io.hpp"
#include "LPC17xx.h"
#include "i2c2.hpp"
#include "time.h"
#include "event_groups.h"
#include "storage.hpp"
#include "string.h"
#include "lpc_pwm.hpp"

#define up 1
#define down 2
#define left 3
#define right 4
/*
class TEMPLATE : public scheduler_task
        {
            public:
                TEMPLATE(uint8_t priority) :
                    scheduler_task("TEMPLATE", 512*4, priority)
                {
                    // Nothing to init
                }

                bool init(void)
                {


                    return true;
                }

                bool run(void *p)
                {


                    return true;
                }
        };
*/

    class gpioTask : public scheduler_task
    {
        public:
            gpioTask(uint8_t priority) :
                scheduler_task("gpio", 512*4, priority)
            {
                /* Nothing to init */
            }

            bool init(void)
            {

                //To set P1.28, P1.29 to GPIO, set bits 24-27 to 0
                LPC_PINCON->PINSEL3 &= ~(0xF<<24);
                //Set P1.28 to output
                LPC_GPIO1->FIODIR |= 1<<28;
                //Set P1.29 to input
                LPC_GPIO1->FIODIR &= ~(1<<29);
                //if(LPC_GPIO->FIODIR = (0x2000_0000) afterwards ^
                //Set P1.8, P1.15 to GPIO (LPC_PINCON->PINSEL2(0x3003_0000))
                LPC_PINCON->PINSEL2 &= 0x3<<16;
                LPC_PINCON->PINSEL2 &= 0X3<<30;
                //Set P1.8 to output
                LPC_GPIO1->FIODIR |= 1<<8;
                //Set P1.15 to input
                LPC_GPIO1->FIODIR &= ~(1<<15);
                //if(LPC_GPIO->FIODIR = (0x2000_0100) afterwards ^
                return true;
            }

            bool run(void *p)
            {
                //External LED
                uint32_t ext_signal = 0;
                ext_signal = (LPC_GPIO1->FIOPIN)&(1<<29);
                //if(LPC_GPIO->FIOPIN = (0x0000_0000) afterwards ^
                if(ext_signal)
                {
                    LPC_GPIO1->FIOCLR |= 1<<28;
                }
                else
                {
                    LPC_GPIO1->FIOSET |= 1<<28;
                }

                //On-board LED
                uint32_t sw_signal = 0;
                sw_signal = (LPC_GPIO1->FIOPIN)&(1<<15);
                if(sw_signal)
                {
                    LPC_GPIO1->FIOCLR |= 1<<8;
                }
                else
                {
                    LPC_GPIO1->FIOSET |= 1<<8;
                }

                return true;
            }
    };

    /*
     * Changed made:
     * Set to master mode
     * Changed baud rate to 38.4
     */
    int mtr = 0;
    //apparently this is the only task I should be doing edits to.
    class BluetoothUART : public scheduler_task
            {
                public:
                    BluetoothUART(uint8_t priority) :
                    scheduler_task("BluetoothUART", 512*4, priority)
                    {
                        /* Nothing to init */
                    }

                    bool init(void)
                    {
                        //init pins
                        LPC_SC->PCONP |= 1<<25;
                        LPC_SC->PCLKSEL1 &= ~(3<<18);
                        LPC_SC->PCLKSEL1 |= 1<<18;
                        if(!(LPC_SC->PCONP ==(0x0200_0000)))
                        {
                            printf("Error: LPC_SC->PCONP\n");
                            exit(1);
                        }

                        if(!(LPC_SC->PCLKSEL1 (0x0004_0000)))
                        {
                            printf("Error: LPC_SC->PCLKSEL1\n");
                            exit(1);
                        }
                        //Set Baud Rate
                        uint16_t DL = sys_get_cpu_clock() / (16*38400);
                        LPC_UART3->LCR |= 1<<7;
                        LPC_UART3->DLL = DL&(0xFF);
                        LPC_UART3->DLM = DL>>8;

                        //Disable FIFO
                        LPC_UART3->FCR &= 0;

                        //Set 8-bit exchange
                        LPC_UART3->LCR &= ~(3);
                        LPC_UART3->LCR |= 3;
                        if(!(LPC_UART3->LCR == 0x03)) //this is for word length
                        {
                            printf("Error: LPC_UART3 LCR register\n");
                            exit(1);
                        }

                        LPC_PINCON->PINSEL9 &= ~(15<<24);
                        LPC_PINCON->PINSEL9 |= 15<<24;
                        //LPC_PINCON->PINSEL9 (0x0F00_0000)
                        //DLAB = 0
                        LPC_UART3->LCR &= ~(1<<7);
                        //disable the divisor latch
                        if (!(LPC_UART3->LCR ==(0b0000_0011)))// I think
                        {
                            printf("Error: LPC_UART3->LCR\n");
                            exit(1);
                        }
                        //GPIO Init
                        LPC_PINCON->PINSEL3 &= ~(0x3<<14);
                        LPC_PINCON->PINSEL3 &= ~(0x15<<24);
                        if(!(LPC_PINCON->PINSEL3 == 0x0000_0000))
                        {
                            printf("Error: LPC_PINCON->PINSEL3 location.\n");
                            exit(1);

                        }
                        LPC_GPIO1->FIODIR |= 0x1<<23;
                        LPC_GPIO1->FIODIR |= 0x3<<28;
                        if(!(LPC_GPIO1->FIODIR == 0x3080_000))
                        {
                            printf("Error: LPC_GPIO1->FIODIR location .\n");
                            exit(1);
                        }
                        return true;
                    }


                    void send(uint8_t data_out)
                    {
                        LPC_UART3->THR = data_out;
                        while(!(LPC_UART3->LSR & 1<<5));
                    }

                    uint8_t read(void)
                    {
                       while(!(LPC_UART3->LSR & 1));
                       return LPC_UART3->RBR;
                    }

                    //write
                    //wait for read
                    //write

                    bool run(void *p)
                    {

                        uint8_t char_in;
                        PWM motor_adj(PWM::pwm1,100);
                        //scanf("%c", &out);
                        //send(out);
                        char_in = read();
                        printf("Read: %c, %i\n", char_in, mtr);
                        if(char_in == 'M')
                        {
                            LPC_GPIO1->FIOSET |= 1<<23;

                        }
                        if(char_in == 'C')
                        {
                            LPC_GPIO1->FIOCLR |= 1<<23;
                        }
                        if(char_in == 'L')
                        {
                            if(mtr > -45)
                            {
                                mtr=mtr-1;
                                LPC_GPIO1->FIOCLR |= 1<<29;
                                for(int m = 0; m < 5; m++)
                                {
                                LPC_GPIO1->FIOCLR |= 1<<28;
                                vTaskDelay(1);
                                LPC_GPIO1->FIOSET |= 1<<28;
                                vTaskDelay(1);
                                }
                            }
                        }
                        else if(char_in == 'R')
                        {
                            if(mtr < 45)
                            {
                                mtr=mtr+1;
                                LPC_GPIO1->FIOSET |= 1<<29;
                                for(int d = 0; d < 5; d++)
                                {
                                LPC_GPIO1->FIOCLR |= 1<<28;
                                vTaskDelay(1);
                                LPC_GPIO1->FIOSET |= 1<<28;
                                vTaskDelay(1);
                                }
                            }
                        }
//                        else if(char_in == 'C') //Code to center on press, need to test motor before implementation
//                        {
//                            if(mtr != 0)
//                            {
//
//                            }
//                        }
                        vTaskDelay(10);


                        return true;
                    }
            };



/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
int main(void)
{
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */

    //send = xQueueCreate(1, sizeof(bool));
    //sensor_queue = xQueueCreate(1, sizeof(int));
    //event_group = xEventGroupCreate();
    //qh = xQueueCreate(2, sizeof(int));
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */

//    scheduler_add_task(new SteeringTest(PRIORITY_HIGH));
//    scheduler_add_task(new SteeringMotorDriverSpdControl(PRIORITY_HIGH));
//    scheduler_add_task(new SteeringMotorDriverDirControl(PRIORITY_HIGH));
//    scheduler_add_task(new PWMTask(PRIORITY_HIGH));
    scheduler_add_task(new BluetoothUART(PRIORITY_HIGH));
    //scheduler_add_task(new SteeringMotorDriverDirControl(PRIORITY_HIGH));
    //xTaskCreate(rx, "rx", 1024, NULL, PRIORITY_LOW, NULL);
    //xTaskCreate(tx, "tx", 1024, NULL, PRIORITY_LOW, NULL);
    //scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
    //scheduler_add_task(new gpioTask(PRIORITY_HIGH));
    //scheduler_add_task(new SPITask(PRIORITY_HIGH));
    //scheduler_add_task(new UART3Task(PRIORITY_HIGH));
    //scheduler_add_task(new EIntTask(PRIORITY_HIGH));
    //scheduler_add_task(new I2CTask(PRIORITY_HIGH));
    //scheduler_add_task(new task1(PRIORITY_HIGH));
    //scheduler_add_task(new task2(PRIORITY_HIGH));
    //scheduler_add_task(new producer(PRIORITY_MEDIUM));
    //scheduler_add_task(new consumer(PRIORITY_MEDIUM));
    //scheduler_add_task(new watchdog(PRIORITY_HIGH));
    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
