################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SRC/Core/core_riscv.c 

C_DEPS += \
./SRC/Core/core_riscv.d 

OBJS += \
./SRC/Core/core_riscv.o 

DIR_OBJS += \
./SRC/Core/*.o \

DIR_DEPS += \
./SRC/Core/*.d \

DIR_EXPANDS += \
./SRC/Core/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
SRC/Core/%.o: ../SRC/Core/%.c
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/SRC/Core" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/SRC/Debug" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/SRC/Peripheral/inc" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/User" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/System" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/Drivers" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/Common" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/Application" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

