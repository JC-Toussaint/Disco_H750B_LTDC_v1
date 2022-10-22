################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/fmc.c \
C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/main.c \
C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/memory_msp.c \
C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/mmc.c \
C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/qspi.c \
C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/stm32h7xx_it.c 

OBJS += \
./Application/User/fmc.o \
./Application/User/main.o \
./Application/User/memory_msp.o \
./Application/User/mmc.o \
./Application/User/qspi.o \
./Application/User/stm32h7xx_it.o 

C_DEPS += \
./Application/User/fmc.d \
./Application/User/main.d \
./Application/User/memory_msp.d \
./Application/User/mmc.d \
./Application/User/qspi.d \
./Application/User/stm32h7xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/fmc.o: C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/fmc.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Utilities -I../../../../../../../Utilities/Fonts -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Middlewares/Third_Party/FatFs/src/drivers -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/main.o: C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Utilities -I../../../../../../../Utilities/Fonts -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Middlewares/Third_Party/FatFs/src/drivers -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/memory_msp.o: C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/memory_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Utilities -I../../../../../../../Utilities/Fonts -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Middlewares/Third_Party/FatFs/src/drivers -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mmc.o: C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/mmc.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Utilities -I../../../../../../../Utilities/Fonts -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Middlewares/Third_Party/FatFs/src/drivers -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/qspi.o: C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/qspi.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Utilities -I../../../../../../../Utilities/Fonts -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Middlewares/Third_Party/FatFs/src/drivers -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32h7xx_it.o: C:/Users/toussaij/STM32Cube/Repository/STM32Cube_FW_H7_V1.10.0/Projects/STM32H750B-DK/Templates/ExtMem_Boot/Src/stm32h7xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../../../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/Components -I../../../../../../../Utilities -I../../../../../../../Utilities/Fonts -I../../../../../../../Middlewares/Third_Party/FatFs/src -I../../../../../../../Middlewares/Third_Party/FatFs/src/drivers -I../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/fmc.d ./Application/User/fmc.o ./Application/User/fmc.su ./Application/User/main.d ./Application/User/main.o ./Application/User/main.su ./Application/User/memory_msp.d ./Application/User/memory_msp.o ./Application/User/memory_msp.su ./Application/User/mmc.d ./Application/User/mmc.o ./Application/User/mmc.su ./Application/User/qspi.d ./Application/User/qspi.o ./Application/User/qspi.su ./Application/User/stm32h7xx_it.d ./Application/User/stm32h7xx_it.o ./Application/User/stm32h7xx_it.su

.PHONY: clean-Application-2f-User

