################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/SW4STM32/startup_stm32h750xx.s 

OBJS += \
./Application/SW4STM32/startup_stm32h750xx.o 

S_DEPS += \
./Application/SW4STM32/startup_stm32h750xx.d 


# Each subdirectory must supply rules for building sources it contributes
Application/SW4STM32/startup_stm32h750xx.o: C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/SW4STM32/startup_stm32h750xx.s Application/SW4STM32/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Application-2f-SW4STM32

clean-Application-2f-SW4STM32:
	-$(RM) ./Application/SW4STM32/startup_stm32h750xx.d ./Application/SW4STM32/startup_stm32h750xx.o

.PHONY: clean-Application-2f-SW4STM32

