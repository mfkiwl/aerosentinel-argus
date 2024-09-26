################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DRIVERS/BNO086/bno086.c 

OBJS += \
./Core/Src/DRIVERS/BNO086/bno086.o 

C_DEPS += \
./Core/Src/DRIVERS/BNO086/bno086.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/DRIVERS/BNO086/%.o Core/Src/DRIVERS/BNO086/%.su Core/Src/DRIVERS/BNO086/%.cyclo: ../Core/Src/DRIVERS/BNO086/%.c Core/Src/DRIVERS/BNO086/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-DRIVERS-2f-BNO086

clean-Core-2f-Src-2f-DRIVERS-2f-BNO086:
	-$(RM) ./Core/Src/DRIVERS/BNO086/bno086.cyclo ./Core/Src/DRIVERS/BNO086/bno086.d ./Core/Src/DRIVERS/BNO086/bno086.o ./Core/Src/DRIVERS/BNO086/bno086.su

.PHONY: clean-Core-2f-Src-2f-DRIVERS-2f-BNO086

