################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/Control/drive_control.c \
../Components/Control/pid_ctrl.c \
../Components/Control/steer_control.c 

OBJS += \
./Components/Control/drive_control.o \
./Components/Control/pid_ctrl.o \
./Components/Control/steer_control.o 

C_DEPS += \
./Components/Control/drive_control.d \
./Components/Control/pid_ctrl.d \
./Components/Control/steer_control.d 


# Each subdirectory must supply rules for building sources it contributes
Components/Control/%.o Components/Control/%.su Components/Control/%.cyclo: ../Components/Control/%.c Components/Control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I"/home/cg/A_DEV/8Motors22NOV/8Motors/Components" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-Control

clean-Components-2f-Control:
	-$(RM) ./Components/Control/drive_control.cyclo ./Components/Control/drive_control.d ./Components/Control/drive_control.o ./Components/Control/drive_control.su ./Components/Control/pid_ctrl.cyclo ./Components/Control/pid_ctrl.d ./Components/Control/pid_ctrl.o ./Components/Control/pid_ctrl.su ./Components/Control/steer_control.cyclo ./Components/Control/steer_control.d ./Components/Control/steer_control.o ./Components/Control/steer_control.su

.PHONY: clean-Components-2f-Control

