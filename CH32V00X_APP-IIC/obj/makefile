################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include System/subdir.mk
-include SRC/Startup/subdir.mk
-include SRC/Peripheral/src/subdir.mk
-include SRC/Debug/subdir.mk
-include SRC/Core/subdir.mk
-include Drivers/subdir.mk
-include Common/subdir.mk
-include Application/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
CH32V00X_APP.bin \
CH32V00X_APP.hex \

SECONDARY_LIST += \
CH32V00X_APP.lst \

SECONDARY_SIZE += \
CH32V00X_APP.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 
	$(MAKE) --no-print-directory post-build 

main-build: CH32V00X_APP.elf secondary-outputs

post-build: 
	 riscv-wch-elf-objcopy -O ihex "CH32V00X_APP.elf" "CH32V00X_APP.hex" && riscv-wch-elf-objcopy -O binary "CH32V00X_APP.elf" "CH32V00X_APP.bin"

# Tool invocations
CH32V00X_APP.elf: $(OBJS) $(USER_OBJS_ESCAPE)
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "c:\Users\Admin\Desktop\work\duoji\CH32V00X_APP\SRC\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"CH32V00X_APP.map" --specs=nano.specs --specs=nosys.specs -o "CH32V00X_APP.elf" $(OBJS) $(LIBS)
CH32V00X_APP.bin: CH32V00X_APP.elf
	@	riscv-wch-elf-objcopy -O binary "CH32V00X_APP.elf" "CH32V00X_APP.bin"
CH32V00X_APP.hex: CH32V00X_APP.elf
	@	riscv-wch-elf-objcopy -O ihex "CH32V00X_APP.elf" "CH32V00X_APP.hex"
CH32V00X_APP.lst: CH32V00X_APP.elf
	@	riscv-wch-elf-objdump --all-headers --demangle --disassemble -M xw "CH32V00X_APP.elf" > "CH32V00X_APP.lst"
CH32V00X_APP.siz: CH32V00X_APP.elf
	riscv-wch-elf-size --format=berkeley "CH32V00X_APP.elf"

# Other Targets
clean:
	-$(RM) $(DIR_OBJS) $(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE) CH32V00X_APP.elf
	-$(RM) $(DIR_EXPANDS) $(CALLGRAPH_DOT)
	-$(RM) $(DIR_DEPS)
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets