################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/Encoders/encoder_abs.c \
../Components/Encoders/encoder_inc.c 

OBJS += \
./Components/Encoders/encoder_abs.o \
./Components/Encoders/encoder_inc.o 

C_DEPS += \
./Components/Encoders/encoder_abs.d \
./Components/Encoders/encoder_inc.d 


# Each subdirectory must supply rules for building sources it contributes
Components/Encoders/%.o Components/Encoders/%.su Components/Encoders/%.cyclo: ../Components/Encoders/%.c Components/Encoders/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I"/home/cg/A_DEV/8Motors22NOV/8Motors/Components" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-Encoders

clean-Components-2f-Encoders:
	-$(RM) ./Components/Encoders/encoder_abs.cyclo ./Components/Encoders/encoder_abs.d ./Components/Encoders/encoder_abs.o ./Components/Encoders/encoder_abs.su ./Components/Encoders/encoder_inc.cyclo ./Components/Encoders/encoder_inc.d ./Components/Encoders/encoder_inc.o ./Components/Encoders/encoder_inc.su

.PHONY: clean-Components-2f-Encoders

