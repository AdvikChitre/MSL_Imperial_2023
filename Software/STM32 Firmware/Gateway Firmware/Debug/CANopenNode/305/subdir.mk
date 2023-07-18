################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CANopenNode/305/CO_LSSmaster.c \
../CANopenNode/305/CO_LSSslave.c 

OBJS += \
./CANopenNode/305/CO_LSSmaster.o \
./CANopenNode/305/CO_LSSslave.o 

C_DEPS += \
./CANopenNode/305/CO_LSSmaster.d \
./CANopenNode/305/CO_LSSslave.d 


# Each subdirectory must supply rules for building sources it contributes
CANopenNode/305/%.o CANopenNode/305/%.su CANopenNode/305/%.cyclo: ../CANopenNode/305/%.c CANopenNode/305/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/bens1/Documents/Uni/Robotics/MSL/MSL_Imperial_2023/Software/STM32 Firmware/Gateway Firmware/CANopenNode_STM32" -I"C:/Users/bens1/Documents/Uni/Robotics/MSL/MSL_Imperial_2023/Software/STM32 Firmware/Gateway Firmware/CANopenNode" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CANopenNode-2f-305

clean-CANopenNode-2f-305:
	-$(RM) ./CANopenNode/305/CO_LSSmaster.cyclo ./CANopenNode/305/CO_LSSmaster.d ./CANopenNode/305/CO_LSSmaster.o ./CANopenNode/305/CO_LSSmaster.su ./CANopenNode/305/CO_LSSslave.cyclo ./CANopenNode/305/CO_LSSslave.d ./CANopenNode/305/CO_LSSslave.o ./CANopenNode/305/CO_LSSslave.su

.PHONY: clean-CANopenNode-2f-305

