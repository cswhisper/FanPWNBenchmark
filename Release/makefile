################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
FanPWMBenchmark.lss \

FLASH_IMAGE += \
FanPWMBenchmark.hex \

EEPROM_IMAGE += \
FanPWMBenchmark.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: FanPWMBenchmark.elf secondary-outputs

# Tool invocations
FanPWMBenchmark.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,FanPWMBenchmark.map,--cref -mrelax -Wl,--gc-sections -mmcu=atmega328p -o "FanPWMBenchmark.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

FanPWMBenchmark.lss: FanPWMBenchmark.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S FanPWMBenchmark.elf  >"FanPWMBenchmark.lss"
	@echo 'Finished building: $@'
	@echo ' '

FanPWMBenchmark.hex: FanPWMBenchmark.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex FanPWMBenchmark.elf  "FanPWMBenchmark.hex"
	@echo 'Finished building: $@'
	@echo ' '

FanPWMBenchmark.eep: FanPWMBenchmark.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex FanPWMBenchmark.elf  "FanPWMBenchmark.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: FanPWMBenchmark.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p FanPWMBenchmark.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) FanPWMBenchmark.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
