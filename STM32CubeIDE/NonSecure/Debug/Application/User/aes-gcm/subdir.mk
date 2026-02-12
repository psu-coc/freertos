################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/aes-gcm/aes-cbc.c \
../Application/User/aes-gcm/aes-ccm.c \
../Application/User/aes-gcm/aes-ctr.c \
../Application/User/aes-gcm/aes-debug.c \
../Application/User/aes-gcm/aes-eax.c \
../Application/User/aes-gcm/aes-encblock.c \
../Application/User/aes-gcm/aes-gcm-test.c \
../Application/User/aes-gcm/aes-gcm.c \
../Application/User/aes-gcm/aes-internal-dec.c \
../Application/User/aes-gcm/aes-internal-enc.c \
../Application/User/aes-gcm/aes-internal.c \
../Application/User/aes-gcm/aes-omac1.c \
../Application/User/aes-gcm/aes-unwrap.c \
../Application/User/aes-gcm/aes-wrap.c 

OBJS += \
./Application/User/aes-gcm/aes-cbc.o \
./Application/User/aes-gcm/aes-ccm.o \
./Application/User/aes-gcm/aes-ctr.o \
./Application/User/aes-gcm/aes-debug.o \
./Application/User/aes-gcm/aes-eax.o \
./Application/User/aes-gcm/aes-encblock.o \
./Application/User/aes-gcm/aes-gcm-test.o \
./Application/User/aes-gcm/aes-gcm.o \
./Application/User/aes-gcm/aes-internal-dec.o \
./Application/User/aes-gcm/aes-internal-enc.o \
./Application/User/aes-gcm/aes-internal.o \
./Application/User/aes-gcm/aes-omac1.o \
./Application/User/aes-gcm/aes-unwrap.o \
./Application/User/aes-gcm/aes-wrap.o 

C_DEPS += \
./Application/User/aes-gcm/aes-cbc.d \
./Application/User/aes-gcm/aes-ccm.d \
./Application/User/aes-gcm/aes-ctr.d \
./Application/User/aes-gcm/aes-debug.d \
./Application/User/aes-gcm/aes-eax.d \
./Application/User/aes-gcm/aes-encblock.d \
./Application/User/aes-gcm/aes-gcm-test.d \
./Application/User/aes-gcm/aes-gcm.d \
./Application/User/aes-gcm/aes-internal-dec.d \
./Application/User/aes-gcm/aes-internal-enc.d \
./Application/User/aes-gcm/aes-internal.d \
./Application/User/aes-gcm/aes-omac1.d \
./Application/User/aes-gcm/aes-unwrap.d \
./Application/User/aes-gcm/aes-wrap.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/aes-gcm/%.o Application/User/aes-gcm/%.su Application/User/aes-gcm/%.cyclo: ../Application/User/aes-gcm/%.c Application/User/aes-gcm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L552xx -DDEBUG -c -I../Application/User/Crypto/hmac-sha256 -I../Application/User/Aesnew -I../Application/User/Speck -I../../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../../Secure_nsclib -I../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/non_secure -I../../../Drivers/BSP/STM32L5xx_Nucleo -I../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33/secure -I../../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../../NonSecure/Inc -I../../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-aes-2d-gcm

clean-Application-2f-User-2f-aes-2d-gcm:
	-$(RM) ./Application/User/aes-gcm/aes-cbc.cyclo ./Application/User/aes-gcm/aes-cbc.d ./Application/User/aes-gcm/aes-cbc.o ./Application/User/aes-gcm/aes-cbc.su ./Application/User/aes-gcm/aes-ccm.cyclo ./Application/User/aes-gcm/aes-ccm.d ./Application/User/aes-gcm/aes-ccm.o ./Application/User/aes-gcm/aes-ccm.su ./Application/User/aes-gcm/aes-ctr.cyclo ./Application/User/aes-gcm/aes-ctr.d ./Application/User/aes-gcm/aes-ctr.o ./Application/User/aes-gcm/aes-ctr.su ./Application/User/aes-gcm/aes-debug.cyclo ./Application/User/aes-gcm/aes-debug.d ./Application/User/aes-gcm/aes-debug.o ./Application/User/aes-gcm/aes-debug.su ./Application/User/aes-gcm/aes-eax.cyclo ./Application/User/aes-gcm/aes-eax.d ./Application/User/aes-gcm/aes-eax.o ./Application/User/aes-gcm/aes-eax.su ./Application/User/aes-gcm/aes-encblock.cyclo ./Application/User/aes-gcm/aes-encblock.d ./Application/User/aes-gcm/aes-encblock.o ./Application/User/aes-gcm/aes-encblock.su ./Application/User/aes-gcm/aes-gcm-test.cyclo ./Application/User/aes-gcm/aes-gcm-test.d ./Application/User/aes-gcm/aes-gcm-test.o ./Application/User/aes-gcm/aes-gcm-test.su ./Application/User/aes-gcm/aes-gcm.cyclo ./Application/User/aes-gcm/aes-gcm.d ./Application/User/aes-gcm/aes-gcm.o ./Application/User/aes-gcm/aes-gcm.su ./Application/User/aes-gcm/aes-internal-dec.cyclo ./Application/User/aes-gcm/aes-internal-dec.d ./Application/User/aes-gcm/aes-internal-dec.o ./Application/User/aes-gcm/aes-internal-dec.su ./Application/User/aes-gcm/aes-internal-enc.cyclo ./Application/User/aes-gcm/aes-internal-enc.d ./Application/User/aes-gcm/aes-internal-enc.o ./Application/User/aes-gcm/aes-internal-enc.su ./Application/User/aes-gcm/aes-internal.cyclo ./Application/User/aes-gcm/aes-internal.d ./Application/User/aes-gcm/aes-internal.o ./Application/User/aes-gcm/aes-internal.su ./Application/User/aes-gcm/aes-omac1.cyclo ./Application/User/aes-gcm/aes-omac1.d ./Application/User/aes-gcm/aes-omac1.o ./Application/User/aes-gcm/aes-omac1.su ./Application/User/aes-gcm/aes-unwrap.cyclo ./Application/User/aes-gcm/aes-unwrap.d ./Application/User/aes-gcm/aes-unwrap.o ./Application/User/aes-gcm/aes-unwrap.su ./Application/User/aes-gcm/aes-wrap.cyclo ./Application/User/aes-gcm/aes-wrap.d ./Application/User/aes-gcm/aes-wrap.o ./Application/User/aes-gcm/aes-wrap.su

.PHONY: clean-Application-2f-User-2f-aes-2d-gcm

