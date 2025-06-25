################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/TFT_ST7735/fonc_tft.c 

OBJS += \
./Drivers/TFT_ST7735/fonc_tft.o 

C_DEPS += \
./Drivers/TFT_ST7735/fonc_tft.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/TFT_ST7735/%.o Drivers/TFT_ST7735/%.su Drivers/TFT_ST7735/%.cyclo: ../Drivers/TFT_ST7735/%.c Drivers/TFT_ST7735/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xE -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/7Seg_MAX7219 -I../Drivers/TFT_ST7735 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-TFT_ST7735

clean-Drivers-2f-TFT_ST7735:
	-$(RM) ./Drivers/TFT_ST7735/fonc_tft.cyclo ./Drivers/TFT_ST7735/fonc_tft.d ./Drivers/TFT_ST7735/fonc_tft.o ./Drivers/TFT_ST7735/fonc_tft.su

.PHONY: clean-Drivers-2f-TFT_ST7735

