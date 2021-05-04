################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221/HTS221_Driver.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221/HTS221_Driver_HL.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb/LPS22HB_Driver.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb/LPS22HB_Driver_HL.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr/LSM303AGR_ACC_driver.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr/LSM303AGR_ACC_driver_HL.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr/LSM303AGR_MAG_driver.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr/LSM303AGR_MAG_driver_HL.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm/LSM6DSM_ACC_GYRO_driver.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm/LSM6DSM_ACC_GYRO_driver_HL.c \
/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115/STC3115_Driver.c 

OBJS += \
./Drivers/BSP/Components/HTS221_Driver.o \
./Drivers/BSP/Components/HTS221_Driver_HL.o \
./Drivers/BSP/Components/LPS22HB_Driver.o \
./Drivers/BSP/Components/LPS22HB_Driver_HL.o \
./Drivers/BSP/Components/LSM303AGR_ACC_driver.o \
./Drivers/BSP/Components/LSM303AGR_ACC_driver_HL.o \
./Drivers/BSP/Components/LSM303AGR_MAG_driver.o \
./Drivers/BSP/Components/LSM303AGR_MAG_driver_HL.o \
./Drivers/BSP/Components/LSM6DSM_ACC_GYRO_driver.o \
./Drivers/BSP/Components/LSM6DSM_ACC_GYRO_driver_HL.o \
./Drivers/BSP/Components/STC3115_Driver.o 

C_DEPS += \
./Drivers/BSP/Components/HTS221_Driver.d \
./Drivers/BSP/Components/HTS221_Driver_HL.d \
./Drivers/BSP/Components/LPS22HB_Driver.d \
./Drivers/BSP/Components/LPS22HB_Driver_HL.d \
./Drivers/BSP/Components/LSM303AGR_ACC_driver.d \
./Drivers/BSP/Components/LSM303AGR_ACC_driver_HL.d \
./Drivers/BSP/Components/LSM303AGR_MAG_driver.d \
./Drivers/BSP/Components/LSM303AGR_MAG_driver_HL.d \
./Drivers/BSP/Components/LSM6DSM_ACC_GYRO_driver.d \
./Drivers/BSP/Components/LSM6DSM_ACC_GYRO_driver_HL.d \
./Drivers/BSP/Components/STC3115_Driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/HTS221_Driver.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221/HTS221_Driver.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/HTS221_Driver_HL.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221/HTS221_Driver_HL.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/LPS22HB_Driver.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb/LPS22HB_Driver.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/LPS22HB_Driver_HL.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb/LPS22HB_Driver_HL.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/LSM303AGR_ACC_driver.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr/LSM303AGR_ACC_driver.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/LSM303AGR_ACC_driver_HL.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr/LSM303AGR_ACC_driver_HL.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/LSM303AGR_MAG_driver.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr/LSM303AGR_MAG_driver.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/LSM303AGR_MAG_driver_HL.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr/LSM303AGR_MAG_driver_HL.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/LSM6DSM_ACC_GYRO_driver.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm/LSM6DSM_ACC_GYRO_driver.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/LSM6DSM_ACC_GYRO_driver_HL.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm/LSM6DSM_ACC_GYRO_driver_HL.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/BSP/Components/STC3115_Driver.o: /home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115/STC3115_Driver.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32_SENSORTILE -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DARM_MATH_CM4 -D__DSP_PRESENT -D__FPU_PRESENT -I../../../../../../../Cifar-10/code/m4 -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/SensorTile" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/Common" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/hts221" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Projects/SensorTile/Applications/DataLog/Inc" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/Drivers/BSP/Components/stc3115" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/NN/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/DSP/Include" -I"/home/matteo/Documents/projects/sensortile/insieme-on-stm32l4/CMSIS/Core/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


