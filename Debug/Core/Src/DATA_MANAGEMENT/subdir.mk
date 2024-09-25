################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DATA_MANAGEMENT/telemetry_manager.c 

OBJS += \
./Core/Src/DATA_MANAGEMENT/telemetry_manager.o 

C_DEPS += \
./Core/Src/DATA_MANAGEMENT/telemetry_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/DATA_MANAGEMENT/%.o Core/Src/DATA_MANAGEMENT/%.su Core/Src/DATA_MANAGEMENT/%.cyclo: ../Core/Src/DATA_MANAGEMENT/%.c Core/Src/DATA_MANAGEMENT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-DATA_MANAGEMENT

clean-Core-2f-Src-2f-DATA_MANAGEMENT:
	-$(RM) ./Core/Src/DATA_MANAGEMENT/telemetry_manager.cyclo ./Core/Src/DATA_MANAGEMENT/telemetry_manager.d ./Core/Src/DATA_MANAGEMENT/telemetry_manager.o ./Core/Src/DATA_MANAGEMENT/telemetry_manager.su

.PHONY: clean-Core-2f-Src-2f-DATA_MANAGEMENT

