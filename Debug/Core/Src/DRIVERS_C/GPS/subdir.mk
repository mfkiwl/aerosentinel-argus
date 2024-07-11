################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DRIVERS_C/GPS/GPS.c 

OBJS += \
./Core/Src/DRIVERS_C/GPS/GPS.o 

C_DEPS += \
./Core/Src/DRIVERS_C/GPS/GPS.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/DRIVERS_C/GPS/%.o Core/Src/DRIVERS_C/GPS/%.su Core/Src/DRIVERS_C/GPS/%.cyclo: ../Core/Src/DRIVERS_C/GPS/%.c Core/Src/DRIVERS_C/GPS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-DRIVERS_C-2f-GPS

clean-Core-2f-Src-2f-DRIVERS_C-2f-GPS:
	-$(RM) ./Core/Src/DRIVERS_C/GPS/GPS.cyclo ./Core/Src/DRIVERS_C/GPS/GPS.d ./Core/Src/DRIVERS_C/GPS/GPS.o ./Core/Src/DRIVERS_C/GPS/GPS.su

.PHONY: clean-Core-2f-Src-2f-DRIVERS_C-2f-GPS

