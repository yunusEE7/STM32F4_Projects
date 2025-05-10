################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/USB_Device_Library/Core/Src/usbd_desc.c 

OBJS += \
./Middlewares/USB_Device_Library/Core/Src/usbd_desc.o 

C_DEPS += \
./Middlewares/USB_Device_Library/Core/Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/USB_Device_Library/Core/Src/%.o Middlewares/USB_Device_Library/Core/Src/%.su Middlewares/USB_Device_Library/Core/Src/%.cyclo: ../Middlewares/USB_Device_Library/Core/Src/%.c Middlewares/USB_Device_Library/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Middlewares/USB_Device_Library/Class/Inc -I../Middlewares/USB_Device_Library/Core/Inc -I../Inc -I../Drivers/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-USB_Device_Library-2f-Core-2f-Src

clean-Middlewares-2f-USB_Device_Library-2f-Core-2f-Src:
	-$(RM) ./Middlewares/USB_Device_Library/Core/Src/usbd_desc.cyclo ./Middlewares/USB_Device_Library/Core/Src/usbd_desc.d ./Middlewares/USB_Device_Library/Core/Src/usbd_desc.o ./Middlewares/USB_Device_Library/Core/Src/usbd_desc.su

.PHONY: clean-Middlewares-2f-USB_Device_Library-2f-Core-2f-Src

