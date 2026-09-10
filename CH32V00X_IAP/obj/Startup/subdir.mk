################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../Startup/startup_ch32v00X_iap.S 

S_UPPER_DEPS += \
./Startup/startup_ch32v00X_iap.d 

OBJS += \
./Startup/startup_ch32v00X_iap.o 

DIR_OBJS += \
./Startup/*.o \

DIR_DEPS += \
./Startup/*.d \

DIR_EXPANDS += \
./Startup/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Startup/%.o: ../Startup/%.S
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -x assembler-with-cpp -I"c:/Users/19128/Desktop/CH32V00X_IAP/Startup" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

