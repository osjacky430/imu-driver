#pragma once

#include <stdint.h>

#include "../../imu_driver.h"

#include "stm32f4xx_hal.h"		// this is included to make the error disappear, not used
#include "stm32f4xx_hal_dma.h"	// this is included to make the error disappear, not used
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"

ImuInterfacePtr HAL_ImuInterfaceCreate(GPIO_TypeDef* t_reset_port, uint16_t t_reset_pin);

void HAL_ImuInterfaceSetupSpi(ImuInterfacePtr t_interface, SPI_HandleTypeDef* spi, GPIO_TypeDef* t_drdy_port,
							  uint16_t t_drdy_pin, GPIO_TypeDef* t_ss_port, uint16_t t_ss_pin);
