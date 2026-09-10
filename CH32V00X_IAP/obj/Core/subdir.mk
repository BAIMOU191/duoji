################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/core_riscv.c 

C_DEPS += \
./Core/core_riscv.d 

OBJS += \
./Core/core_riscv.o 

DIR_OBJS += \
./Core/*.o \

DIR_DEPS += \
./Core/*.d \

DIR_EXPANDS += \
./Core/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Core/%.o: ../Core/%.c
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/19128/Desktop/CH32V00X_IAP/Debug" -I"c:/Users/19128/Desktop/CH32V00X_IAP/Core" -I"c:/Users/19128/Desktop/CH32V00X_IAP/User" -I"c:/Users/19128/Desktop/CH32V00X_IAP/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

