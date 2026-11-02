################################################################################
# MRS Version: 3.0.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf
FLAGS_C :=  -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"../SRC/Core" -I"../SRC/Debug" -I"../SRC/Peripheral/inc" -I"../User" -I"../System" -I"../Drivers" -I"../Common" -I"../Application" -std=gnu99 -flto
FLAGS_C_FILE := .compile_c_flags

FLAGS_CPP :=  -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -std=gnu++11 -fabi-version=0
FLAGS_CPP_FILE := .compile_cpp_flags

FLAGS_ASM :=  -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -x assembler-with-cpp -I"../SRC/Startup"
FLAGS_ASM_FILE := .compile_asm_flags

FLAGS_LD:=  -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "..\SRC\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"CH32V00X_APP.map" --specs=nano.specs --specs=nosys.specs -flto
FLAGS_LD_FILE := .ld_flags

$(FLAGS_C_FILE): FORCE
	@echo '$(FLAGS_C)' | cmp -s - $@ || echo '$(FLAGS_C)' > $@
$(FLAGS_CPP_FILE): FORCE
	@echo '$(FLAGS_CPP)' | cmp -s - $@ || echo '$(FLAGS_CPP)' > $@
$(FLAGS_ASM_FILE): FORCE
	@echo '$(FLAGS_ASM)' | cmp -s - $@ || echo '$(FLAGS_ASM)' > $@
$(FLAGS_LD_FILE): FORCE
	@echo '$(FLAGS_LD)' | cmp -s - $@ || echo '$(FLAGS_LD)' > $@
.PHONY: FORCE
FORCE:

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

SOURCES_LIST := .sources_list
$(SOURCES_LIST): FORCE
	@echo '$(C_SRCS) $(CPP_SRCS) $(S_UPPER_SRCS)' | tr ' ' '\n' | sort > $@.tmp
	@cmp -s $@.tmp $@ || mv $@.tmp $@
	@rm -f $@.tmp

LD_SCRIPTS := ../SRC/Ld/Link.ld

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
CH32V00X_APP.elf: $(OBJS) $(USER_OBJS_ESCAPE) $(SOURCES_LIST) $(FLAGS_LD_FILE) $(LD_SCRIPTS)
	@	riscv-wch-elf-gcc -march=rv32ec_zmmul_xw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "..\SRC\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"CH32V00X_APP.map" --specs=nano.specs --specs=nosys.specs -flto -o "CH32V00X_APP.elf" $(OBJS) $(LIBS)
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
	-$(RM) $(FLAGS_C_FILE) $(FLAGS_CPP_FILE) $(FLAGS_ASM_FILE) $(FLAGS_LD_FILE) $(SOURCES_LIST)
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets