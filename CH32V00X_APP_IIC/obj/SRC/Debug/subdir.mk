################################################################################
# MRS Version: 3.0.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SRC/Debug/debug.c 

C_DEPS += \
./SRC/Debug/debug.d 

OBJS += \
./SRC/Debug/debug.o 

DIR_OBJS += \
./SRC/Debug/*.o \

DIR_DEPS += \
./SRC/Debug/*.d \

DIR_EXPANDS += \
./SRC/Debug/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
SRC/Debug/%.o: ../SRC/Debug/%.c $(FLAGS_C_FILE)
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_IIC/SRC/Core" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_IIC/SRC/Debug" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_IIC/SRC/Peripheral/inc" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_IIC/User" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_IIC/System" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_IIC/Drivers" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_IIC/Common" -I"c:/Users/Admin/Desktop/duoji/CH32V00X_APP_IIC/Application" -std=gnu99 -flto -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

