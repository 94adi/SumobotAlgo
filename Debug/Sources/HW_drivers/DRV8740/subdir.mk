################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/HW_drivers/DRV8740/drv8740.c 

OBJS += \
./Sources/HW_drivers/DRV8740/drv8740.o 

C_DEPS += \
./Sources/HW_drivers/DRV8740/drv8740.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/HW_drivers/DRV8740/%.o: ../Sources/HW_drivers/DRV8740/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -I"../Sources" -I"../Includes" -std=c99 -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


