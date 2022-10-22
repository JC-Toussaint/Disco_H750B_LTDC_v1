################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32H750B-DK/stm32h750b_discovery.c \
../Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_bus.c 

OBJS += \
./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery.o \
./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_bus.o 

C_DEPS += \
./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery.d \
./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_bus.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32H750B-DK/%.o Drivers/BSP/STM32H750B-DK/%.su: ../Drivers/BSP/STM32H750B-DK/%.c Drivers/BSP/STM32H750B-DK/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DAPPLICATION_ADDRESS=0x90000000U -DUSE_STM32H750B_DISCO -DUSE_HAL_DRIVER -DSTM32H750xx -DDEBUG -c -I../Core/Inc -I../Drivers/BSP/Components/Common -I../Drivers/BSP/Components/rk043fn48h -I../Drivers/BSP/STM32H750B-DK -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32H750B-2d-DK

clean-Drivers-2f-BSP-2f-STM32H750B-2d-DK:
	-$(RM) ./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery.d ./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery.o ./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery.su ./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_bus.d ./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_bus.o ./Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_bus.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32H750B-2d-DK

