################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure/secure_context.c \
C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure/secure_context_port.c \
C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure/secure_heap.c \
C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure/secure_init.c 

OBJS += \
./Middlewares/FreeRTOS/secure_context.o \
./Middlewares/FreeRTOS/secure_context_port.o \
./Middlewares/FreeRTOS/secure_heap.o \
./Middlewares/FreeRTOS/secure_init.o 

C_DEPS += \
./Middlewares/FreeRTOS/secure_context.d \
./Middlewares/FreeRTOS/secure_context_port.d \
./Middlewares/FreeRTOS/secure_heap.d \
./Middlewares/FreeRTOS/secure_init.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FreeRTOS/secure_context.o: C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure/secure_context.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L552xx -DDEBUG -c -I../../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Application/User/Crypto/hmac-sha256 -I../../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../../Secure_nsclib -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/non_secure -I../../../Secure/Inc -I../../../Drivers/BSP/STM32L5xx_Nucleo -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure -I../../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../../Drivers/CMSIS/Include -I../Application/User -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/aes-gcm" -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/Aesnew" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/secure_context_port.o: C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure/secure_context_port.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L552xx -DDEBUG -c -I../../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Application/User/Crypto/hmac-sha256 -I../../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../../Secure_nsclib -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/non_secure -I../../../Secure/Inc -I../../../Drivers/BSP/STM32L5xx_Nucleo -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure -I../../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../../Drivers/CMSIS/Include -I../Application/User -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/aes-gcm" -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/Aesnew" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/secure_heap.o: C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure/secure_heap.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L552xx -DDEBUG -c -I../../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Application/User/Crypto/hmac-sha256 -I../../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../../Secure_nsclib -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/non_secure -I../../../Secure/Inc -I../../../Drivers/BSP/STM32L5xx_Nucleo -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure -I../../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../../Drivers/CMSIS/Include -I../Application/User -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/aes-gcm" -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/Aesnew" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FreeRTOS/secure_init.o: C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure/secure_init.c Middlewares/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L552xx -DDEBUG -c -I../../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Application/User/Crypto/hmac-sha256 -I../../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../../Secure_nsclib -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/non_secure -I../../../Secure/Inc -I../../../Drivers/BSP/STM32L5xx_Nucleo -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure -I../../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../../Drivers/CMSIS/Include -I../Application/User -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/aes-gcm" -I"C:/Users/laoam/OneDrive/Documents/stm32/workspace/freertostrustzone-master/freertostrustzone-master/STM32CubeIDE/Secure/Application/User/Aesnew" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-FreeRTOS

clean-Middlewares-2f-FreeRTOS:
	-$(RM) ./Middlewares/FreeRTOS/secure_context.cyclo ./Middlewares/FreeRTOS/secure_context.d ./Middlewares/FreeRTOS/secure_context.o ./Middlewares/FreeRTOS/secure_context.su ./Middlewares/FreeRTOS/secure_context_port.cyclo ./Middlewares/FreeRTOS/secure_context_port.d ./Middlewares/FreeRTOS/secure_context_port.o ./Middlewares/FreeRTOS/secure_context_port.su ./Middlewares/FreeRTOS/secure_heap.cyclo ./Middlewares/FreeRTOS/secure_heap.d ./Middlewares/FreeRTOS/secure_heap.o ./Middlewares/FreeRTOS/secure_heap.su ./Middlewares/FreeRTOS/secure_init.cyclo ./Middlewares/FreeRTOS/secure_init.d ./Middlewares/FreeRTOS/secure_init.o ./Middlewares/FreeRTOS/secure_init.su

.PHONY: clean-Middlewares-2f-FreeRTOS

