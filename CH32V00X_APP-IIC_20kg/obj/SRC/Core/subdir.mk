################################################################################
# MRS Version: 3.0.0
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
SRC/Core/%.o: ../SRC/Core/%.c $(FLAGS_C_FILE)
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"../SRC/Core" -I"../SRC/Debug" -I"../SRC/Peripheral/inc" -I"../User" -I"../System" -I"../Drivers" -I"../Common" -I"../Application" -std=gnu99 -flto -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

