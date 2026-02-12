################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/Crypto/hmac-sha256/hmac-sha256.c 

OBJS += \
./Application/User/Crypto/hmac-sha256/hmac-sha256.o 

C_DEPS += \
./Application/User/Crypto/hmac-sha256/hmac-sha256.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/Crypto/hmac-sha256/%.o Application/User/Crypto/hmac-sha256/%.su Application/User/Crypto/hmac-sha256/%.cyclo: ../Application/User/Crypto/hmac-sha256/%.c Application/User/Crypto/hmac-sha256/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L552xx -DDEBUG -c -I../Application/User/Crypto/hmac-sha256 -I../Application/User/Aesnew -I../Application/User/Speck -I../../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../Secure_nsclib -I../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/non_secure -I../../../Drivers/BSP/STM32L5xx_Nucleo -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure -I../../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../../NonSecure/Inc -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-Crypto-2f-hmac-2d-sha256

clean-Application-2f-User-2f-Crypto-2f-hmac-2d-sha256:
	-$(RM) ./Application/User/Crypto/hmac-sha256/hmac-sha256.cyclo ./Application/User/Crypto/hmac-sha256/hmac-sha256.d ./Application/User/Crypto/hmac-sha256/hmac-sha256.o ./Application/User/Crypto/hmac-sha256/hmac-sha256.su

.PHONY: clean-Application-2f-User-2f-Crypto-2f-hmac-2d-sha256

