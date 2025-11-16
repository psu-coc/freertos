################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.o: C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.c Drivers/BSP/STM32L5xx_Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L552xx -DDEBUG -c -I../../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Application/User/Crypto/hmac-sha256 -I../../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../../Secure_nsclib -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/non_secure -I../../../Secure/Inc -I../../../Drivers/BSP/STM32L5xx_Nucleo -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure -I../../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../../Drivers/CMSIS/Include -I../Application/User -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/aes-gcm" -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/Aesnew" -Os -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32L5xx_Nucleo

clean-Drivers-2f-BSP-2f-STM32L5xx_Nucleo:
	-$(RM) ./Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.cyclo ./Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.d ./Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.o ./Drivers/BSP/STM32L5xx_Nucleo/stm32l5xx_nucleo.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32L5xx_Nucleo

