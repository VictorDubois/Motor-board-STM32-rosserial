################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f3xx_hal.c \
../Drivers/Src/stm32f3xx_hal_cortex.c \
../Drivers/Src/stm32f3xx_hal_dma.c \
../Drivers/Src/stm32f3xx_hal_exti.c \
../Drivers/Src/stm32f3xx_hal_flash.c \
../Drivers/Src/stm32f3xx_hal_flash_ex.c \
../Drivers/Src/stm32f3xx_hal_gpio.c \
../Drivers/Src/stm32f3xx_hal_i2c.c \
../Drivers/Src/stm32f3xx_hal_i2c_ex.c \
../Drivers/Src/stm32f3xx_hal_pwr.c \
../Drivers/Src/stm32f3xx_hal_pwr_ex.c \
../Drivers/Src/stm32f3xx_hal_rcc.c \
../Drivers/Src/stm32f3xx_hal_rcc_ex.c \
../Drivers/Src/stm32f3xx_hal_spi.c \
../Drivers/Src/stm32f3xx_hal_spi_ex.c \
../Drivers/Src/stm32f3xx_hal_tim.c \
../Drivers/Src/stm32f3xx_hal_tim_ex.c \
../Drivers/Src/stm32f3xx_hal_uart.c \
../Drivers/Src/stm32f3xx_hal_uart_ex.c 

OBJS += \
./Drivers/Src/stm32f3xx_hal.o \
./Drivers/Src/stm32f3xx_hal_cortex.o \
./Drivers/Src/stm32f3xx_hal_dma.o \
./Drivers/Src/stm32f3xx_hal_exti.o \
./Drivers/Src/stm32f3xx_hal_flash.o \
./Drivers/Src/stm32f3xx_hal_flash_ex.o \
./Drivers/Src/stm32f3xx_hal_gpio.o \
./Drivers/Src/stm32f3xx_hal_i2c.o \
./Drivers/Src/stm32f3xx_hal_i2c_ex.o \
./Drivers/Src/stm32f3xx_hal_pwr.o \
./Drivers/Src/stm32f3xx_hal_pwr_ex.o \
./Drivers/Src/stm32f3xx_hal_rcc.o \
./Drivers/Src/stm32f3xx_hal_rcc_ex.o \
./Drivers/Src/stm32f3xx_hal_spi.o \
./Drivers/Src/stm32f3xx_hal_spi_ex.o \
./Drivers/Src/stm32f3xx_hal_tim.o \
./Drivers/Src/stm32f3xx_hal_tim_ex.o \
./Drivers/Src/stm32f3xx_hal_uart.o \
./Drivers/Src/stm32f3xx_hal_uart_ex.o 

C_DEPS += \
./Drivers/Src/stm32f3xx_hal.d \
./Drivers/Src/stm32f3xx_hal_cortex.d \
./Drivers/Src/stm32f3xx_hal_dma.d \
./Drivers/Src/stm32f3xx_hal_exti.d \
./Drivers/Src/stm32f3xx_hal_flash.d \
./Drivers/Src/stm32f3xx_hal_flash_ex.d \
./Drivers/Src/stm32f3xx_hal_gpio.d \
./Drivers/Src/stm32f3xx_hal_i2c.d \
./Drivers/Src/stm32f3xx_hal_i2c_ex.d \
./Drivers/Src/stm32f3xx_hal_pwr.d \
./Drivers/Src/stm32f3xx_hal_pwr_ex.d \
./Drivers/Src/stm32f3xx_hal_rcc.d \
./Drivers/Src/stm32f3xx_hal_rcc_ex.d \
./Drivers/Src/stm32f3xx_hal_spi.d \
./Drivers/Src/stm32f3xx_hal_spi_ex.d \
./Drivers/Src/stm32f3xx_hal_tim.d \
./Drivers/Src/stm32f3xx_hal_tim_ex.d \
./Drivers/Src/stm32f3xx_hal_uart.d \
./Drivers/Src/stm32f3xx_hal_uart_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Inc -I../Inc/ros -I../Drivers/STM32F3xx_HAL_Driver/Inc -I"C:/Users/pauli/krabi/Motor-board-STM32-rosserial/Drivers/Inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f3xx_hal.cyclo ./Drivers/Src/stm32f3xx_hal.d ./Drivers/Src/stm32f3xx_hal.o ./Drivers/Src/stm32f3xx_hal.su ./Drivers/Src/stm32f3xx_hal_cortex.cyclo ./Drivers/Src/stm32f3xx_hal_cortex.d ./Drivers/Src/stm32f3xx_hal_cortex.o ./Drivers/Src/stm32f3xx_hal_cortex.su ./Drivers/Src/stm32f3xx_hal_dma.cyclo ./Drivers/Src/stm32f3xx_hal_dma.d ./Drivers/Src/stm32f3xx_hal_dma.o ./Drivers/Src/stm32f3xx_hal_dma.su ./Drivers/Src/stm32f3xx_hal_exti.cyclo ./Drivers/Src/stm32f3xx_hal_exti.d ./Drivers/Src/stm32f3xx_hal_exti.o ./Drivers/Src/stm32f3xx_hal_exti.su ./Drivers/Src/stm32f3xx_hal_flash.cyclo ./Drivers/Src/stm32f3xx_hal_flash.d ./Drivers/Src/stm32f3xx_hal_flash.o ./Drivers/Src/stm32f3xx_hal_flash.su ./Drivers/Src/stm32f3xx_hal_flash_ex.cyclo ./Drivers/Src/stm32f3xx_hal_flash_ex.d ./Drivers/Src/stm32f3xx_hal_flash_ex.o ./Drivers/Src/stm32f3xx_hal_flash_ex.su ./Drivers/Src/stm32f3xx_hal_gpio.cyclo ./Drivers/Src/stm32f3xx_hal_gpio.d ./Drivers/Src/stm32f3xx_hal_gpio.o ./Drivers/Src/stm32f3xx_hal_gpio.su ./Drivers/Src/stm32f3xx_hal_i2c.cyclo ./Drivers/Src/stm32f3xx_hal_i2c.d ./Drivers/Src/stm32f3xx_hal_i2c.o ./Drivers/Src/stm32f3xx_hal_i2c.su ./Drivers/Src/stm32f3xx_hal_i2c_ex.cyclo ./Drivers/Src/stm32f3xx_hal_i2c_ex.d ./Drivers/Src/stm32f3xx_hal_i2c_ex.o ./Drivers/Src/stm32f3xx_hal_i2c_ex.su ./Drivers/Src/stm32f3xx_hal_pwr.cyclo ./Drivers/Src/stm32f3xx_hal_pwr.d ./Drivers/Src/stm32f3xx_hal_pwr.o ./Drivers/Src/stm32f3xx_hal_pwr.su ./Drivers/Src/stm32f3xx_hal_pwr_ex.cyclo ./Drivers/Src/stm32f3xx_hal_pwr_ex.d ./Drivers/Src/stm32f3xx_hal_pwr_ex.o ./Drivers/Src/stm32f3xx_hal_pwr_ex.su ./Drivers/Src/stm32f3xx_hal_rcc.cyclo ./Drivers/Src/stm32f3xx_hal_rcc.d ./Drivers/Src/stm32f3xx_hal_rcc.o ./Drivers/Src/stm32f3xx_hal_rcc.su ./Drivers/Src/stm32f3xx_hal_rcc_ex.cyclo ./Drivers/Src/stm32f3xx_hal_rcc_ex.d ./Drivers/Src/stm32f3xx_hal_rcc_ex.o ./Drivers/Src/stm32f3xx_hal_rcc_ex.su ./Drivers/Src/stm32f3xx_hal_spi.cyclo ./Drivers/Src/stm32f3xx_hal_spi.d ./Drivers/Src/stm32f3xx_hal_spi.o ./Drivers/Src/stm32f3xx_hal_spi.su ./Drivers/Src/stm32f3xx_hal_spi_ex.cyclo ./Drivers/Src/stm32f3xx_hal_spi_ex.d ./Drivers/Src/stm32f3xx_hal_spi_ex.o ./Drivers/Src/stm32f3xx_hal_spi_ex.su ./Drivers/Src/stm32f3xx_hal_tim.cyclo ./Drivers/Src/stm32f3xx_hal_tim.d ./Drivers/Src/stm32f3xx_hal_tim.o ./Drivers/Src/stm32f3xx_hal_tim.su ./Drivers/Src/stm32f3xx_hal_tim_ex.cyclo ./Drivers/Src/stm32f3xx_hal_tim_ex.d ./Drivers/Src/stm32f3xx_hal_tim_ex.o ./Drivers/Src/stm32f3xx_hal_tim_ex.su ./Drivers/Src/stm32f3xx_hal_uart.cyclo ./Drivers/Src/stm32f3xx_hal_uart.d ./Drivers/Src/stm32f3xx_hal_uart.o ./Drivers/Src/stm32f3xx_hal_uart.su ./Drivers/Src/stm32f3xx_hal_uart_ex.cyclo ./Drivers/Src/stm32f3xx_hal_uart_ex.d ./Drivers/Src/stm32f3xx_hal_uart_ex.o ./Drivers/Src/stm32f3xx_hal_uart_ex.su

.PHONY: clean-Drivers-2f-Src

