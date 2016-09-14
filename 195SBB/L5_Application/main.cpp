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

                //Set P1.8, P1.15 to GPIO
                LPC_PINCON->PINSEL2 &= 0x3<<16;
                LPC_PINCON->PINSEL2 &= 0X3<<30;
                //Set P1.8 to output
                LPC_GPIO1->FIODIR |= 1<<8;
                //Set P1.15 to input
                LPC_GPIO1->FIODIR &= ~(1<<15);

                return true;
            }

            bool run(void *p)
            {
                //External LED
                uint32_t ext_signal = 0;
                ext_signal = (LPC_GPIO1->FIOPIN)&(1<<29);
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


    class SPITask : public scheduler_task
        {
            public:
                SPITask(uint8_t priority) :
                    scheduler_task("SPI", 512*4, priority)
                {
                    /* Nothing to init */
                }

                bool init(void)
                {
                    //Power up SPI
                    //PCONP for SSP1, [10]
                    LPC_SC->PCONP |= 1<<10;
                    //PCLKSEL0 for SSP1, [21:20], 0b01 for no divisor;
                    LPC_SC->PCLKSEL0 &= ~(3<<20);
                    LPC_SC->PCLKSEL0 |= 1<<20;

                    //Initialize all SPI related pins
                    //MOSI, [19:18] <- 10
                    LPC_PINCON->PINSEL0 &= ~( (3 << 14) | (3 << 16) | (3 << 18) );
                    LPC_PINCON->PINSEL0 |= 0x2<<18;
                    //MISO1, [17:16] <- 10
                    LPC_PINCON->PINSEL0 |= 0x2<<16;
                    //SCK1, [15:14] <- 10
                    LPC_PINCON->PINSEL0 |= 0x2<<14;
                    //GPIO for CS, [13:12] <- 00
                   // LPC_PINCON->PINSEL0 &= ~(0x3<<12);
                   // LPC_GPIO0->FIODIR |= 1<<6;

                    //Initialize SPI registers
                    LPC_SSP1->CR0 = 7; //0b x|0|0|00|0111
                    LPC_SSP1->CR1 = 2; //0b x|0|0|1|0
                    LPC_SSP1->CPSR = 8; //CPSDVSR, [7:0] (set to 2)
                    DS();
                    return true;
                }

                void CS(void)
                {
                    BIT(LPC_GPIO0->FIOPIN).b6 = 0;
                }

                void DS(void)
                {
                    BIT(LPC_GPIO0->FIOPIN).b6 = 1;
                }

                uint8_t ssptrx(uint8_t in)
                {

                    LPC_SSP1->DR = in;
                    while(LPC_SSP1->SR&(1<<4));
                    return LPC_SSP1->DR;
                }

                bool run(void *p)
                {
                    //uint8_t received;
                    //Read manufacturer ID, 0x9F, 4 bytes
                    printf("\nDevice ID: ");
                    CS();
                    ssptrx(0x9F);
                    for(int i = 0; i < 5; i++)
                    {
                        printf("%02X", ssptrx(0xAA));
                    }
                    DS();
                    vTaskDelay(100);
                    //Read status register, 0xD7, 2 bytes
                    printf("\nStatus Register: ");
                    CS();
                    ssptrx(0xD7);
                    for(int k = 0; k < 2; k++)
                    {
                        printf("%02X", ssptrx(0xAA));
                    }
                    DS();
                    vTaskDelay(1000);

                    return true;
                }
        };


    //UART 0 is hardwired to the USB port
    //one person writes for UART 2/3
    //UART has a 16 level FIFO buffer

    //Handshake signals (flow control, UART 1 ONLY)
    //DTR
    //CTS (clear to send)
    //RTS (request to send)

    //RBR
    //THR - send out data as soon as you write
    //-> SHR ->wire
    //DLL, watch for DLAB
    //  hidden to prevent user from writing to it
    //

    /*
     *
     * Key steps in making a driver
     * 1) init uart & clock regs
     * 2) PINSEL
     *
     * Baud Rate
     * bits per second
     * ideal bit width 26 us
     * (default values for DivAddVal and MulVal are 1)
     * DLM = {UnDLM, UnDLL}
     *
     */

    class UART3Task : public scheduler_task
        {
            public:
                UART3Task(uint8_t priority) :
                    scheduler_task("UART3", 512*4, priority)
                {
                    /* Nothing to init */
                }

                bool init(void)
                {
                    //init pins
                    LPC_SC->PCONP |= 1<<25;
                    LPC_SC->PCLKSEL1 &= ~(3<<18);
                    LPC_SC->PCLKSEL1 |= 1<<18;

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

                    //PINSEL
                    LPC_PINCON->PINSEL9 &= ~(15<<24);
                    LPC_PINCON->PINSEL9 |= 15<<24;

                    //DLAB = 0
                    LPC_UART3->LCR &= ~(1<<7);

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

                    uint8_t out, in;
                    scanf("%c", &out);
                    send(out);
                    in = read();
                    printf("Read: %c\n", in);
                    vTaskDelay(100);


                    return true;
                }
        };

    //Saleae logic analyzer screenshot of UART

    int exint = 0;

    void EINT3_IRQHandler(void)
    {

        exint = 1;
        if(LPC_GPIOINT->IO2IntStatR & (1<<6))
        {
            LPC_GPIOINT->IO2IntClr |= 1<<6;
        }
        if(LPC_GPIOINT->IO2IntStatR & (1<<7))
        {
            LPC_GPIOINT->IO2IntClr |= 1<<7;
        }

    }


    class EIntTask : public scheduler_task
            {
                public:
                    EIntTask(uint8_t priority) :
                        scheduler_task("EIntTask", 512*4, priority)
                    {
                        /* Nothing to init */
                    }

                    bool init(void)
                    {

                        //set 2.6, 2.7 as input
                        LPC_PINCON->PINSEL4 &= ~(15<<12);
                        LPC_GPIO2->FIODIR &= ~(3<<6);

                        //LED
                        LPC_PINCON->PINSEL2 &= ~(0x3<<16);
                        LPC_PINCON->PINSEL2 &= ~(0x3);
                        LPC_GPIO1->FIODIR |= 1;
                        LPC_GPIO1->FIODIR |= 1<<8;

                        //enable rising edge interrupts on pin 2.6 & 2.7
                        LPC_GPIOINT->IO2IntEnR |= 3<<6;
                        isr_register(EINT3_IRQn, EINT3_IRQHandler);
                        NVIC_EnableIRQ(EINT3_IRQn);

                        return true;
                    }

                    bool run(void *p)
                    {

                        bool on = true;

                        if(exint)
                        {
                            if(BIT(LPC_GPIO2->FIOPIN).b6)
                            {
                                LPC_GPIO1->FIOCLR |= 1<<8;
                                on = true;
                            }
                            if(BIT(LPC_GPIO2->FIOPIN).b7)
                            {

                                LPC_GPIO1->FIOCLR |= 1<<0;
                                on = true;
                            }
                            exint = 0;
                        }
                        if(on)
                        {
                            vTaskDelay(100);
                            LPC_GPIO1->FIOSET |= 1<<8;
                            LPC_GPIO1->FIOSET |= 1<<0;
                        }
                        return true;
                    }
            };

    class PWMTask : public scheduler_task
            {
                public:
                    PWMTask(uint8_t priority) :
                        scheduler_task("PWMTask", 512*4, priority)
                    {
                        // Nothing to init
                    }

                    bool init(void)
                    {

                        return true;
                    }

                    bool run(void *p)
                    {
                        PWM motor(PWM::pwm1, 1000);
                        for(float i = 2.5; i <= 12.5; i++)
                        {
                            motor.set(i+.5);
                            vTaskDelay(1000);
                        }
                        return true;
                    }
            };

//Attach logic analyzer to SDA / SCL
//Configure logic anlyzer for I2C

//Learn the I2C command
    //I2C Discover
    //I2C write 0x20 0x00 0xFF
    //          DEV  REG  Data
/*
 * device address (8 bit)
 *  W-even R-odd
 *
 *  1-byte read/write
 *  +multiple
 *  +creative I2C device
 *
 *  main ()
 *   uint32 mem,copy
 *   i2c.initslave(0xF0m &mem, 3);
 *   while(1)
 *   {
 *      if(copy != mem)
 *      {
 *      puts("Modified")
 *      copy = mem;
 *      }
 *   }
 *
 */
    class I2CTask : public scheduler_task
    {
        #define myAddress   0xBB
            uint8_t buffer[10] = { 0 };
        public:
            I2CTask(uint8_t priority) :
                scheduler_task("I2C", 512*4, priority)
            {
                /* Nothing to init */
            }

            bool init(void)
            {
                if (I2C2::getInstance().init(SYS_CFG_I2C2_CLK_KHZ))
                    I2C2::getInstance().initSlave(myAddress, &buffer[0], sizeof(buffer));
                else
                    puts("ERROR: Possible short on SDA or SCL wire (I2C2)!");
                return true;
            }
            bool run(void *p)
            {
                vTaskDelay(3000);                        //Unblock vTask = ~3000ms
                for (int i = 0; i < sizeof(buffer); i++)
                    printf("%x ", buffer[i]);
                printf("\n");
                return true;
            }
    };

    QueueHandle_t send = 0;

    class task1 : public scheduler_task
            {
                public:
                    task1(uint8_t priority) :
                        scheduler_task("task1", 512*4, priority)
                    {
                        // Nothing to init
                    }

                    bool init(void)
                    {

                        return true;
                    }

                    bool run(void *p)
                    {

                        int orient = 0;
                        int tilt_x = AS.getX();
                        int tilt_y = AS.getY();

                        if(tilt_y > 300)
                        {
                            orient = up;
                        }
                        else if(tilt_y < -300)
                        {
                            orient = down;
                        }

                        if(tilt_x > 300)
                        {
                            orient = left;
                        }
                        else if(tilt_x < -300)
                        {
                            orient = right;
                        }
                        printf("Sending %i to queue.\n", orient);
                        xQueueSend(send, &orient, 1000);
                        vTaskDelay(1000);

                        return true;
                    }
            };


    class task2 : public scheduler_task
            {
                public:
                       task2(uint8_t priority) :
                        scheduler_task("task2", 512*4, priority)
                    {
                        // Nothing to init
                    }

                    bool init(void)
                    {

                        //Init LED pins P1.0/1/4/8

                        LPC_PINCON->PINSEL2 &= ~(0x15); //0,1
                        LPC_PINCON->PINSEL2 &= ~(0x3<<16); //8
                        LPC_PINCON->PINSEL2 &= ~(0x3<<8); //4
                        LPC_GPIO1->FIODIR |= 3; //0, 1
                        LPC_GPIO1->FIODIR |= 1<<4; //4
                        LPC_GPIO1->FIODIR |= 1<<8; //8
                        //100010011 = 275

                        return true;
                    }

                    bool run(void *p)
                    {

                        int on = 0;

                        if(xQueueReceive(send, &on, 1000))
                        {
                            printf("Received %i.\n", on);
                        }
                        if(on == left || on == right)
                        {
                           LPC_GPIO1->FIOCLR = 275;
                        }
                        else
                        {
                           LPC_GPIO1->FIOSET = 275;
                        }
                        vTaskDelay(1000);
                        return true;
                    }
            };

    QueueHandle_t sensor_queue;
    EventGroupHandle_t event_group;

    class producer : public scheduler_task
            {
                public:
                        producer(uint8_t priority) :
                        scheduler_task("producer", 512*4, priority)
                    {
                        // Nothing to init
                    }

                    bool run(void *p)
                    {
                        int avg = 0;
                        for(int i = 0; i < 100; i++)
                        {
                            avg += LS.getRawValue();
                            vTaskDelay(10);
                        }
                        avg = avg / 100;
                        xQueueSend(sensor_queue, &avg, 1000);
                        xEventGroupSetBits(event_group, 0x2);

                        return true;
                    }
            };

    class consumer : public scheduler_task
            {
                public:
                        consumer(uint8_t priority) :
                        scheduler_task("consumer", 512*4, priority)
                    {
                        // Nothing to init
                    }

                    bool run(void *p)
                    {
                        int light = 0;
                        time_t timestamp_buffer[10];
                        int buffer[10];

                        for(int k = 0; k < 10; k++)
                        {
                            if(xQueueReceive(sensor_queue, &light, portMAX_DELAY) == pdTRUE)
                            {
                                buffer[k] = light;
                                timestamp_buffer[k] = time(NULL);
                            }
                            else
                            {
                                printf("Buffer read failed!\n");
                            }

                        }

                        //Convert data into string, print string to file
                        char to_print[100];
                        unsigned int size;

                        for(int j = 0; j < 10; j++)
                        {
                            size = snprintf(to_print, 100, "%li, %u\n", timestamp_buffer[j], buffer[j]);
                            Storage::append("0:sensor.txt", &to_print, size , 0);
                        }
                        printf("Finished write.\n");

                        xEventGroupSetBits(event_group, 0x4);

                        return true;
                    }
            };

    class watchdog : public scheduler_task
            {
                public:
                        watchdog(uint8_t priority) :
                        scheduler_task("watchdog", 512*4, priority)
                    {
                        // Nothing to init
                    }

                    bool run(void *p)
                    {
                        int check_bits = xEventGroupWaitBits(event_group, 0x6,pdTRUE,pdTRUE,20000);
                        char to_print[100];
                        int size;
                        bool write = false;

                        size = snprintf(to_print, 100, "Tasks that have not responded: ");
                        if(!(check_bits & 0x2))
                        {
                            strncat(to_print, " Producer", 9);
                            size += 10;
                            write = true;
                        }
                        if(!(check_bits & 0x4))
                        {
                            strncat(to_print, " Consumer", 9);
                            size += 10;
                            write = true;
                        }
                        strncat(to_print, "\n", 1);
                        size++;
                        if(write)
                        {
                            Storage::append("0:stuck.txt", &to_print, size , 0);
                        }

                        return true;
                    }
            };

//    QueueHandle_t qh = 0;
//
//        class rxTask : public scheduler_task
//        {
//            public:
//                rxTask(uint8_t priority) :
//                    scheduler_task("rx", 512*4, priority)
//                {
//                    /* Nothing to init */
//                }
//
//                bool init(void)
//                {
//                    int item = 0;
//
//                    puts("rx task");
//                    if (xQueueReceive(qh, &item, portMAX_DELAY))
//                    {
//                        puts("Rx received an item!");
//                    }
//
//                    vTaskSuspend(0);
//                    puts("Rx is suspended!");
//                }
//                };
//
//            class txTask : public scheduler_task
//            {
//                public:
//                    txTask(uint8_t priority) :
//                        scheduler_task("tx", 512*4, priority)
//                    {
//                        /* Nothing to init */
//                    }
//
//                bool init(void)
//                {
//
//                        int item = 0;
//                        while(1)
//                        {
//                            puts("Yield");
//                            taskYIELD();
//
//                            xQueueSend(qh, &item, 0);
//                            puts("Did I send an item?");
//
//                            xQueueSend(qh, &item, portMAX_DELAY);
//                            puts("I must have sent an item");
//                        }
//                    }
//                };

    class char_Dev
    {
        public:
            char getcha();
            void putcha(char c);
            void printf();
            void puts_upper();
            void gets(char *c);
            void scanf();
    };

    class SteeringMotorDriverSpdControl : public scheduler_task
            {
                public:
                        SteeringMotorDriverSpdControl(uint8_t priority) :
                        scheduler_task("SteeringMotorDriverSpdControl", 512*4, priority)
                    {
                        // Nothing to init
                    }

                    bool init(void)
                    {

                        LPC_PINCON->PINSEL3 &= ~(0x3<<24);
                        LPC_GPIO1->FIODIR |= 0x1<<28;

                        return true;
                    }

                    bool run(void *p)
                    {

//                        int speed = 1000; //Lower speed, faster spin
//                        int rotation_amount = 10;
//                        #define peak_time 2000 // Length of peaks don't matter, arbitrary number
//
//                        if(speed < 15 && speed != 0)
//                        {
//                            printf("Invalid motor speed");
//                        }
//                        else
//                        {
//                            for(int i = 0; i < rotation_amount; i++)
//                            {
                                LPC_GPIO1->FIOCLR |= 1<<28;
                                vTaskDelay(1);
                                LPC_GPIO1->FIOSET |= 1<<28;
                                vTaskDelay(1);
//                            }
//                        }

                        return true;
                    }
            };

    class SteeringMotorDriverDirControl : public scheduler_task
                {
                    public:
                            SteeringMotorDriverDirControl(uint8_t priority) :
                            scheduler_task("SteeringMotorDriverDirControl", 512*4, priority)
                        {
                            // Nothing to init
                        }

                        bool init(void)
                        {

                            LPC_PINCON->PINSEL3 &= ~(0x3<<26);
                            LPC_GPIO1->FIODIR |= (0x1<<29);

                            return true;
                        }

                        bool run(void *p)
                        {

                            int s = SW.getSwitch(1);

                            if(s)
                            {
                                LPC_GPIO1->FIOCLR |= 1<<29;
                            }
                            else
                            {
                                LPC_GPIO1->FIOSET |= 1<<29;
                            }
                            vTaskDelay(10);

                            return true;
                        }
                };

    int turncount = 0;

    class SteeringTest : public scheduler_task
                {
                    public:
                            SteeringTest(uint8_t priority) :
                            scheduler_task("SteeringTest", 512*4, priority)
                        {
                            // Nothing to init
                        }

                        bool init(void)
                        {

                            LPC_PINCON->PINSEL3 &= ~(0x15<<24);
                            LPC_GPIO1->FIODIR |= 0x3<<28;

                            return true;
                        }

                        bool run(void *p)
                        {

                        if(SW.getSwitch(1) || SW.getSwitch(2))
                        {
                            if(SW.getSwitch(1))
                            {
                                LPC_GPIO1->FIOCLR = 1<<29;
                            }
                            else if(SW.getSwitch(2))
                            {
                                LPC_GPIO1->FIOSET = 1<<29;
                            }
                            LPC_GPIO1->FIOCLR |= 1<<28;
                            vTaskDelay(10);
                            LPC_GPIO1->FIOSET |= 1<<28;
                            vTaskDelay(10);
                        }

                            return true;
                        }
                };

    QueueHandle_t qh = 0;

    void rx(void *p)
    {
        int item = 0;

        puts("rx task");
        if (xQueueReceive(qh, &item, portMAX_DELAY))
        {
            puts("Rx received an item!");
        }

        vTaskSuspend(0);
        puts("Rx is suspended!");
    }

    void tx(void *p)
    {
        int item = 0;
        while(1)
        {
            puts("Yield");
            taskYIELD();

            xQueueSend(qh, &item, 0);
            puts("Did I send an item?");

            xQueueSend(qh, &item, portMAX_DELAY);
            puts("I must have sent an item");
        }
    }

    /*
     * Changed made:
     * Set to master mode
     * Changed baud rate to 38.4
     */
    int mtr = 0;
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

                        //PINSEL
                        LPC_PINCON->PINSEL9 &= ~(15<<24);
                        LPC_PINCON->PINSEL9 |= 15<<24;

                        //DLAB = 0
                        LPC_UART3->LCR &= ~(1<<7);

                        //GPIO Init
                        LPC_PINCON->PINSEL3 &= ~(0x3<<14);
                        LPC_PINCON->PINSEL3 &= ~(0x15<<24);
                        LPC_GPIO1->FIODIR |= 0x1<<23;
                        LPC_GPIO1->FIODIR |= 0x3<<28;

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

    send = xQueueCreate(1, sizeof(bool));
    sensor_queue = xQueueCreate(1, sizeof(int));
    event_group = xEventGroupCreate();
    qh = xQueueCreate(2, sizeof(int));
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
