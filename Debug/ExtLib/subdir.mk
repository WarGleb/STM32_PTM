################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ExtLib/CRCCalc.c \
../ExtLib/Common.c \
../ExtLib/DataExchCAN.c \
../ExtLib/DataExchETH.c \
../ExtLib/DataExchUART.c \
../ExtLib/INA226.c \
../ExtLib/ads867x.c \
../ExtLib/ads867x_dс.c \
../ExtLib/dacx0501.c 

OBJS += \
./ExtLib/CRCCalc.o \
./ExtLib/Common.o \
./ExtLib/DataExchCAN.o \
./ExtLib/DataExchETH.o \
./ExtLib/DataExchUART.o \
./ExtLib/INA226.o \
./ExtLib/ads867x.o \
./ExtLib/ads867x_dс.o \
./ExtLib/dacx0501.o 

C_DEPS += \
./ExtLib/CRCCalc.d \
./ExtLib/Common.d \
./ExtLib/DataExchCAN.d \
./ExtLib/DataExchETH.d \
./ExtLib/DataExchUART.d \
./ExtLib/INA226.d \
./ExtLib/ads867x.d \
./ExtLib/ads867x_dс.d \
./ExtLib/dacx0501.d 


# Each subdirectory must supply rules for building sources it contributes
ExtLib/%.o ExtLib/%.su ExtLib/%.cyclo: ../ExtLib/%.c ExtLib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I"D:/CubeIDE_PRJ/sPTMachine/ExtLib" -I"D:/CubeIDE_PRJ/sPTMachine/LWIP/Target" -I"D:/CubeIDE_PRJ/sPTMachine/Middlewares/Third_Party/LwIP/src/include" -I"D:/CubeIDE_PRJ/sPTMachine/Middlewares/Third_Party/LwIP/system" -I"D:/CubeIDE_PRJ/sPTMachine/Middlewares/Third_Party/LwIP/system/arch" -I"D:/CubeIDE_PRJ/sPTMachine/Middlewares/Third_Party/LwIP/system/OS" -I"D:/CubeIDE_PRJ/sPTMachine/Drivers/BSP/Components/lan8742" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ExtLib

clean-ExtLib:
	-$(RM) ./ExtLib/CRCCalc.cyclo ./ExtLib/CRCCalc.d ./ExtLib/CRCCalc.o ./ExtLib/CRCCalc.su ./ExtLib/Common.cyclo ./ExtLib/Common.d ./ExtLib/Common.o ./ExtLib/Common.su ./ExtLib/DataExchCAN.cyclo ./ExtLib/DataExchCAN.d ./ExtLib/DataExchCAN.o ./ExtLib/DataExchCAN.su ./ExtLib/DataExchETH.cyclo ./ExtLib/DataExchETH.d ./ExtLib/DataExchETH.o ./ExtLib/DataExchETH.su ./ExtLib/DataExchUART.cyclo ./ExtLib/DataExchUART.d ./ExtLib/DataExchUART.o ./ExtLib/DataExchUART.su ./ExtLib/INA226.cyclo ./ExtLib/INA226.d ./ExtLib/INA226.o ./ExtLib/INA226.su ./ExtLib/ads867x.cyclo ./ExtLib/ads867x.d ./ExtLib/ads867x.o ./ExtLib/ads867x.su ./ExtLib/ads867x_dс.cyclo ./ExtLib/ads867x_dс.d ./ExtLib/ads867x_dс.o ./ExtLib/ads867x_dс.su ./ExtLib/dacx0501.cyclo ./ExtLib/dacx0501.d ./ExtLib/dacx0501.o ./ExtLib/dacx0501.su

.PHONY: clean-ExtLib

