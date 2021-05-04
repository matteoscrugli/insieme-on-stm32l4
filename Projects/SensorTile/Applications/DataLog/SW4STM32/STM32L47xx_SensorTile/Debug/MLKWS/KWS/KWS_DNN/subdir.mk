################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/MLKWS/KWS/KWS_DNN/kws_dnn.cpp 

OBJS += \
./MLKWS/KWS/KWS_DNN/kws_dnn.o 

CPP_DEPS += \
./MLKWS/KWS/KWS_DNN/kws_dnn.d 


# Each subdirectory must supply rules for building sources it contributes
MLKWS/KWS/KWS_DNN/kws_dnn.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/MLKWS/KWS/KWS_DNN/kws_dnn.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -D__DSP_PRESENT -D__FPU_PRESENT -DARM_MATH_CM4 -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/MLKWS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


