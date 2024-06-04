################################################################################
# MRS Version: 1.9.1
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Task/subdir.mk
-include StdPeriphDriver/subdir.mk
-include Startup/subdir.mk
-include RVMSIS/subdir.mk
-include HAL/subdir.mk
-include APP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
H01_Air780E_app.hexAndbin \

SECONDARY_LIST += \
H01_Air780E_app.lst \

SECONDARY_SIZE += \
H01_Air780E_app.siz \


# All Target
all: H01_Air780E_app.elf secondary-outputs

# Tool invocations
H01_Air780E_app.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -T "../Ld/Link.ld" -nostartfiles -Xlinker --gc-sections -L"../" -L"R:\CODE\CHcode\CH582\H01_Air780E\H01_Air780E_app\LIB" -L"../StdPeriphDriver" -Xlinker --print-memory-usage -Wl,-Map,"H01_Air780E_app.map" --specs=nano.specs -u _printf_float --specs=nosys.specs -o "H01_Air780E_app.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
H01_Air780E_app.hexAndbin: H01_Air780E_app.elf
	@	riscv-none-embed-objcopy -O ihex "H01_Air780E_app.elf"  "H01_Air780E_app.hex" && riscv-none-embed-objcopy -O binary "H01_Air780E_app.elf"  "H01_Air780E_app.bin"
	@	@
H01_Air780E_app.lst: H01_Air780E_app.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble "H01_Air780E_app.elf" > "H01_Air780E_app.lst"
	@	@
H01_Air780E_app.siz: H01_Air780E_app.elf
	@	riscv-none-embed-size --format=berkeley "H01_Air780E_app.elf"
	@	@
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) H01_Air780E_app.elf
	-@
post-build:
	-riscv-none-embed-objcopy -O ihex "H01_Air780E_app.elf" "H01_Air780E_app.hex" && riscv-none-embed-objcopy -O binary "H01_Air780E_app.elf" "H01_Air780E_app.bin"
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents post-build

-include ../makefile.targets
