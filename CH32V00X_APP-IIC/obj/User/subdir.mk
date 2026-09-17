################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32v00X_it.c \
../User/main.c 

C_DEPS += \
./User/ch32v00X_it.d \
./User/main.d 

OBJS += \
./User/ch32v00X_it.o \
./User/main.o 

DIR_OBJS += \
./User/*.o \

DIR_DEPS += \
./User/*.d \

DIR_EXPANDS += \
./User/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP-IIC/SRC/Core" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP-IIC/SRC/Debug" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP-IIC/SRC/Peripheral/inc" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP-IIC/User" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP-IIC/System" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP-IIC/Drivers" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP-IIC/Common" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP-IIC/Application" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

