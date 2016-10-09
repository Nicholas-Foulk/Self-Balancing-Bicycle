################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../event_groups.c \
../list.c \
../queue.c \
../tasks.c \
../timers.c 

OBJS += \
./event_groups.o \
./list.o \
./queue.o \
./tasks.o \
./timers.o 

C_DEPS += \
./event_groups.d \
./list.d \
./queue.d \
./tasks.d \
./timers.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -I"C:\Users\nick\Documents\LPCXpresso_7.9.0_455\workspace\FreeRTOS_Library\include" -I"C:\Users\nick\Documents\LPCXpresso_7.9.0_455\workspace\CMSISv1p30_LPC17xx\inc" -I"C:\Users\nick\Documents\LPCXpresso_7.9.0_455\workspace\FreeRTOS_Library\portable" -I"C:\Users\nick\Documents\LPCXpresso_7.9.0_455\workspace\FreeRTOS_Library\demo_code" -O1 -g3 -Wall -c -fmessage-length=0 -mcpu=cortex-m3 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


