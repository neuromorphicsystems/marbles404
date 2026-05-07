################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/scamp5_main.cpp 

CPP_DEPS += \
./src/scamp5_main.d 

OBJS += \
./src/scamp5_main.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C++ Compiler'
	arm-none-eabi-c++ -std=gnu++11 -D__NEWLIB__ -DNDEBUG -D__CODE_RED -DCORE_M0 -D__USE_LPCOPEN -DNO_BOARD_LIB -DCPP_USE_HEAP -D__LPC43XX__ -D__MULTICORE_M0APP -DCORE_M0APP -DWORKSPACE_PATH='R"RRR(C:\Users\levus\Documents\MCUXpressoIDE_25.6.136\workspace)RRR"' -DPROJECT_NAME='R"RRR(example_1_image_capture_and_display)RRR"' -I"D:\SCAMP\scamp5d_lib\lpc_chip_43xx_m0\inc" -I"D:\SCAMP\scamp5d_lib\s5d_m0\inc" -I"D:\SCAMP\scamp5d_lib\s5d_m4\inc" -Og -fno-common -g3 -gdwarf-4 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0 -mthumb -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/scamp5_main.d ./src/scamp5_main.o

.PHONY: clean-src

