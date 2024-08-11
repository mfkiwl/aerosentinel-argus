################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DRIVERS_C/MS560702BA03/MS5607.c 

OBJS += \
./Core/Src/DRIVERS_C/MS560702BA03/MS5607.o 

C_DEPS += \
./Core/Src/DRIVERS_C/MS560702BA03/MS5607.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/DRIVERS_C/MS560702BA03/%.o Core/Src/DRIVERS_C/MS560702BA03/%.su Core/Src/DRIVERS_C/MS560702BA03/%.cyclo: ../Core/Src/DRIVERS_C/MS560702BA03/%.c Core/Src/DRIVERS_C/MS560702BA03/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-DRIVERS_C-2f-MS560702BA03

clean-Core-2f-Src-2f-DRIVERS_C-2f-MS560702BA03:
	-$(RM) ./Core/Src/DRIVERS_C/MS560702BA03/MS5607.cyclo ./Core/Src/DRIVERS_C/MS560702BA03/MS5607.d ./Core/Src/DRIVERS_C/MS560702BA03/MS5607.o ./Core/Src/DRIVERS_C/MS560702BA03/MS5607.su

.PHONY: clean-Core-2f-Src-2f-DRIVERS_C-2f-MS560702BA03

