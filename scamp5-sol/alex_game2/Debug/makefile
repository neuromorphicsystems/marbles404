################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++M_DEPS)),)
-include $(C++M_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(CCM_DEPS)),)
-include $(CCM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXXM_DEPS)),)
-include $(CXXM_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := example_1_image_capture_and_display
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: example_1_image_capture_and_display.axf

# Tool invocations
example_1_image_capture_and_display.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU C++ Linker'
	arm-none-eabi-c++ -nostdlib -L"D:\SCAMP\scamp5d_lib\s5d_m0\Release" -Xlinker -Map="example_1_image_capture_and_display.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m0 -mthumb -T example_1_image_capture_and_display_Debug.ld -o "example_1_image_capture_and_display.axf" $(OBJS) $(USER_OBJS) $(LIBS) -ls5d_m0 -llpc_chip_43xx_m0 && arm-none-eabi-objcopy --target elf32-littlearm --verbose --strip-all --remove-section=.ARM.attributes --redefine-sym __vectors_start__=__vectors_start___core_m0app --keep-symbol __vectors_start___core_m0app --redefine-sym _data=__start_data_core_m0app --keep-symbol __start_data_core_m0app --redefine-sym _edata=__end_data_core_m0app --keep-symbol __end_data_core_m0app --remove-section=".bss*" --remove-section=".noinit*" --rename-section .ARM.exidx=".core_m0app.ARM.exidx" --rename-section .ARM.extab=".core_m0app.ARM.extab" --rename-section .text=".core_m0app" --rename-section .data=".core_m0app.data" "example_1_image_capture_and_display.axf" "example_1_image_capture_and_display.axf.o" && mcux-fixelf "example_1_image_capture_and_display.axf.o"
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) example_1_image_capture_and_display.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "example_1_image_capture_and_display.axf"; # arm-none-eabi-objcopy -v -O binary "example_1_image_capture_and_display.axf" "example_1_image_capture_and_display.bin" ; # checksum -p LPC4357_M0 -d "example_1_image_capture_and_display.bin";
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
