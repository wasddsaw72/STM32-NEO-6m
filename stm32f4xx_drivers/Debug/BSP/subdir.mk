################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/lcd_2.c 

OBJS += \
./BSP/lcd_2.o 

C_DEPS += \
./BSP/lcd_2.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o BSP/%.su BSP/%.cyclo: ../BSP/%.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -I"C:/Users/q/STM32CubeIDE/workspace_1.14.1/stm32f4xx_drivers/BSP" -I"C:/Users/q/STM32CubeIDE/workspace_1.14.1/stm32f4xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -finput-charset=UTF-8 -fexec-charset=cp1251 -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP

clean-BSP:
	-$(RM) ./BSP/lcd_2.cyclo ./BSP/lcd_2.d ./BSP/lcd_2.o ./BSP/lcd_2.su

.PHONY: clean-BSP

