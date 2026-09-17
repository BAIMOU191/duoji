################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SRC/Peripheral/src/ch32v00X_adc.c \
../SRC/Peripheral/src/ch32v00X_dbgmcu.c \
../SRC/Peripheral/src/ch32v00X_dma.c \
../SRC/Peripheral/src/ch32v00X_exti.c \
../SRC/Peripheral/src/ch32v00X_flash.c \
../SRC/Peripheral/src/ch32v00X_gpio.c \
../SRC/Peripheral/src/ch32v00X_i2c.c \
../SRC/Peripheral/src/ch32v00X_iwdg.c \
../SRC/Peripheral/src/ch32v00X_misc.c \
../SRC/Peripheral/src/ch32v00X_opa.c \
../SRC/Peripheral/src/ch32v00X_pwr.c \
../SRC/Peripheral/src/ch32v00X_rcc.c \
../SRC/Peripheral/src/ch32v00X_spi.c \
../SRC/Peripheral/src/ch32v00X_tim.c \
../SRC/Peripheral/src/ch32v00X_usart.c \
../SRC/Peripheral/src/ch32v00X_wwdg.c 

C_DEPS += \
./SRC/Peripheral/src/ch32v00X_adc.d \
./SRC/Peripheral/src/ch32v00X_dbgmcu.d \
./SRC/Peripheral/src/ch32v00X_dma.d \
./SRC/Peripheral/src/ch32v00X_exti.d \
./SRC/Peripheral/src/ch32v00X_flash.d \
./SRC/Peripheral/src/ch32v00X_gpio.d \
./SRC/Peripheral/src/ch32v00X_i2c.d \
./SRC/Peripheral/src/ch32v00X_iwdg.d \
./SRC/Peripheral/src/ch32v00X_misc.d \
./SRC/Peripheral/src/ch32v00X_opa.d \
./SRC/Peripheral/src/ch32v00X_pwr.d \
./SRC/Peripheral/src/ch32v00X_rcc.d \
./SRC/Peripheral/src/ch32v00X_spi.d \
./SRC/Peripheral/src/ch32v00X_tim.d \
./SRC/Peripheral/src/ch32v00X_usart.d \
./SRC/Peripheral/src/ch32v00X_wwdg.d 

OBJS += \
./SRC/Peripheral/src/ch32v00X_adc.o \
./SRC/Peripheral/src/ch32v00X_dbgmcu.o \
./SRC/Peripheral/src/ch32v00X_dma.o \
./SRC/Peripheral/src/ch32v00X_exti.o \
./SRC/Peripheral/src/ch32v00X_flash.o \
./SRC/Peripheral/src/ch32v00X_gpio.o \
./SRC/Peripheral/src/ch32v00X_i2c.o \
./SRC/Peripheral/src/ch32v00X_iwdg.o \
./SRC/Peripheral/src/ch32v00X_misc.o \
./SRC/Peripheral/src/ch32v00X_opa.o \
./SRC/Peripheral/src/ch32v00X_pwr.o \
./SRC/Peripheral/src/ch32v00X_rcc.o \
./SRC/Peripheral/src/ch32v00X_spi.o \
./SRC/Peripheral/src/ch32v00X_tim.o \
./SRC/Peripheral/src/ch32v00X_usart.o \
./SRC/Peripheral/src/ch32v00X_wwdg.o 

DIR_OBJS += \
./SRC/Peripheral/src/*.o \

DIR_DEPS += \
./SRC/Peripheral/src/*.d \

DIR_EXPANDS += \
./SRC/Peripheral/src/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
SRC/Peripheral/src/%.o: ../SRC/Peripheral/src/%.c
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC/SRC/Core" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC/SRC/Debug" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC/SRC/Peripheral/inc" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC/User" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC/System" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC/Drivers" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC/Common" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_ADC/Application" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

