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
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"../SRC/Core" -I"../SRC/Debug" -I"../SRC/Peripheral/inc" -I"../User" -I"../System" -I"../Drivers" -I"../Common" -I"../Application" -std=gnu99 -flto -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

