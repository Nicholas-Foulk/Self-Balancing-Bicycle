################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/cr_startup_lpc17.c \
../src/main.c \
../src/system_LPC17xx.c \
../src/uart.c 

OBJS += \
./src/cr_startup_lpc17.o \
./src/main.o \
./src/system_LPC17xx.o \
./src/uart.o 

C_DEPS += \
./src/cr_startup_lpc17.d \
./src/main.d \
./src/system_LPC17xx.d \
./src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DNDEBUG -D__CODE_RED -D__USE_CMSIS=CMSIS_CORE_LPC17xx -I"C:\Users\Dragon\Documents\LPCXpresso_7.4.0_229\workspace\CMSIS_CORE_LPC17xx\inc" -Os -g -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


