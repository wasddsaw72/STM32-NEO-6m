################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main_lcd.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/main_lcd.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/main_lcd.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -I"C:/Users/q/STM32CubeIDE/workspace_1.14.1/stm32f4xx_drivers/BSP" -I"C:/Users/q/STM32CubeIDE/workspace_1.14.1/stm32f4xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -finput-charset=UTF-8 -fexec-charset=cp1251 -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/main_lcd.cyclo ./Src/main_lcd.d ./Src/main_lcd.o ./Src/main_lcd.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

