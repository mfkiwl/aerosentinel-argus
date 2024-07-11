################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DRIVERS_C/BNO055/bno055.c \
../Core/Src/DRIVERS_C/BNO055/bno055_support.c 

OBJS += \
./Core/Src/DRIVERS_C/BNO055/bno055.o \
./Core/Src/DRIVERS_C/BNO055/bno055_support.o 

C_DEPS += \
./Core/Src/DRIVERS_C/BNO055/bno055.d \
./Core/Src/DRIVERS_C/BNO055/bno055_support.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/DRIVERS_C/BNO055/%.o Core/Src/DRIVERS_C/BNO055/%.su Core/Src/DRIVERS_C/BNO055/%.cyclo: ../Core/Src/DRIVERS_C/BNO055/%.c Core/Src/DRIVERS_C/BNO055/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-DRIVERS_C-2f-BNO055

clean-Core-2f-Src-2f-DRIVERS_C-2f-BNO055:
	-$(RM) ./Core/Src/DRIVERS_C/BNO055/bno055.cyclo ./Core/Src/DRIVERS_C/BNO055/bno055.d ./Core/Src/DRIVERS_C/BNO055/bno055.o ./Core/Src/DRIVERS_C/BNO055/bno055.su ./Core/Src/DRIVERS_C/BNO055/bno055_support.cyclo ./Core/Src/DRIVERS_C/BNO055/bno055_support.d ./Core/Src/DRIVERS_C/BNO055/bno055_support.o ./Core/Src/DRIVERS_C/BNO055/bno055_support.su

.PHONY: clean-Core-2f-Src-2f-DRIVERS_C-2f-BNO055

