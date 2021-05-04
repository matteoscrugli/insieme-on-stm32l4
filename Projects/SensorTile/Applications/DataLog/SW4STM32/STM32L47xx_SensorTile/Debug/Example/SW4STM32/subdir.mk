################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/SW4STM32/startup_stm32l476xx.s 

OBJS += \
./Example/SW4STM32/startup_stm32l476xx.o 


# Each subdirectory must supply rules for building sources it contributes
Example/SW4STM32/startup_stm32l476xx.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/SW4STM32/startup_stm32l476xx.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../Inc -IROJ_DIR$/../Inc -I../../../../../../../Drivers/BSP/SensorTile -I../../../../../../../Drivers/BSP/Components/stc3115 -I../../../../../../../CMSIS/Core/Include -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


