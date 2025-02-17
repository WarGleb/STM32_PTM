################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TCP_Server/tcpserv.c 

OBJS += \
./TCP_Server/tcpserv.o 

C_DEPS += \
./TCP_Server/tcpserv.d 


# Each subdirectory must supply rules for building sources it contributes
TCP_Server/%.o TCP_Server/%.su TCP_Server/%.cyclo: ../TCP_Server/%.c TCP_Server/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I"D:/PTM/ExtLib" -I"D:/PTM/LWIP/Target" -I"D:/PTM/Middlewares/Third_Party/LwIP/src/include" -I"D:/PTM/Middlewares/Third_Party/LwIP/system" -I"D:/PTM/Middlewares/Third_Party/LwIP/system/arch" -I"D:/PTM/Middlewares/Third_Party/LwIP/system/OS" -I"D:/PTM/Drivers/BSP/Components/lan8742" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TCP_Server

clean-TCP_Server:
	-$(RM) ./TCP_Server/tcpserv.cyclo ./TCP_Server/tcpserv.d ./TCP_Server/tcpserv.o ./TCP_Server/tcpserv.su

.PHONY: clean-TCP_Server

