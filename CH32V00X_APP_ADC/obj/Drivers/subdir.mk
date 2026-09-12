################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/D_adc.c \
../Drivers/D_i2c.c \
../Drivers/D_iwdg.c \
../Drivers/D_motor.c \
../Drivers/D_mt6701.c \
../Drivers/D_opa.c \
../Drivers/D_spi.c \
../Drivers/D_tim.c \
../Drivers/D_tmp112.c \
../Drivers/D_uart.c \
../Drivers/D_ws2812.c 

C_DEPS += \
./Drivers/D_adc.d \
./Drivers/D_i2c.d \
./Drivers/D_iwdg.d \
./Drivers/D_motor.d \
./Drivers/D_mt6701.d \
./Drivers/D_opa.d \
./Drivers/D_spi.d \
./Drivers/D_tim.d \
./Drivers/D_tmp112.d \
./Drivers/D_uart.d \
./Drivers/D_ws2812.d 

OBJS += \
./Drivers/D_adc.o \
./Drivers/D_i2c.o \
./Drivers/D_iwdg.o \
./Drivers/D_motor.o \
./Drivers/D_mt6701.o \
./Drivers/D_opa.o \
./Drivers/D_spi.o \
./Drivers/D_tim.o \
./Drivers/D_tmp112.o \
./Drivers/D_uart.o \
./Drivers/D_ws2812.o 

DIR_OBJS += \
./Drivers/*.o \

DIR_DEPS += \
./Drivers/*.d \

DIR_EXPANDS += \
./Drivers/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Drivers/%.o: ../Drivers/%.c
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/Admin/Desktop/1/CH32V00X_APP_ADC/SRC/Core" -I"c:/Users/Admin/Desktop/1/CH32V00X_APP_ADC/SRC/Debug" -I"c:/Users/Admin/Desktop/1/CH32V00X_APP_ADC/SRC/Peripheral/inc" -I"c:/Users/Admin/Desktop/1/CH32V00X_APP_ADC/User" -I"c:/Users/Admin/Desktop/1/CH32V00X_APP_ADC/System" -I"c:/Users/Admin/Desktop/1/CH32V00X_APP_ADC/Drivers" -I"c:/Users/Admin/Desktop/1/CH32V00X_APP_ADC/Common" -I"c:/Users/Admin/Desktop/1/CH32V00X_APP_ADC/Application" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

