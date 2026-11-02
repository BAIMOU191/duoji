################################################################################
# MRS Version: 3.0.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Common/C_Motor_Model.c \
../Common/C_Pos_Ctrl.c \
../Common/C_Ring_Buf.c \
../Common/C_Speed_Observer.c \
../Common/C_Task_Scheduler.c \
../Common/C_Traj_Planner.c 

C_DEPS += \
./Common/C_Motor_Model.d \
./Common/C_Pos_Ctrl.d \
./Common/C_Ring_Buf.d \
./Common/C_Speed_Observer.d \
./Common/C_Task_Scheduler.d \
./Common/C_Traj_Planner.d 

OBJS += \
./Common/C_Motor_Model.o \
./Common/C_Pos_Ctrl.o \
./Common/C_Ring_Buf.o \
./Common/C_Speed_Observer.o \
./Common/C_Task_Scheduler.o \
./Common/C_Traj_Planner.o 

DIR_OBJS += \
./Common/*.o \

DIR_DEPS += \
./Common/*.d \

DIR_EXPANDS += \
./Common/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Common/%.o: ../Common/%.c $(FLAGS_C_FILE)
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"../SRC/Core" -I"../SRC/Debug" -I"../SRC/Peripheral/inc" -I"../User" -I"../System" -I"../Drivers" -I"../Common" -I"../Application" -std=gnu99 -flto -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

