################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/A_Calib_Min.c \
../Application/A_Config.c \
../Application/A_Protect.c \
../Application/A_Sensor.c \
../Application/A_Servo.c \
../Application/A_Tasks.c \
../Application/A_UartCmd.c 

C_DEPS += \
./Application/A_Calib_Min.d \
./Application/A_Config.d \
./Application/A_Protect.d \
./Application/A_Sensor.d \
./Application/A_Servo.d \
./Application/A_Tasks.d \
./Application/A_UartCmd.d 

OBJS += \
./Application/A_Calib_Min.o \
./Application/A_Config.o \
./Application/A_Protect.o \
./Application/A_Sensor.o \
./Application/A_Servo.o \
./Application/A_Tasks.o \
./Application/A_UartCmd.o 

DIR_OBJS += \
./Application/*.o \

DIR_DEPS += \
./Application/*.d \

DIR_EXPANDS += \
./Application/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Application/%.o: ../Application/%.c
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/SRC/Core" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/SRC/Debug" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/SRC/Peripheral/inc" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/User" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/System" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/Drivers" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/Common" -I"c:/Users/Admin/Desktop/work/duoji/CH32V00X_APP/Application" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

