#pragma once

#include <stdint.h>

#include "../imu_driver.h"

#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"

ImuInterfacePtr LL_ImuInterfaceCreate(GPIO_Typedef *t_reset_port, uint16_t t_reset_pin);

void LL_ImuInterfaceSetupSpi(ImuInterfacePtr t_interface, SPI_Typedef *spi, GPIO_Typedef *t_drdy_port,
							 uint16_t t_drdy_pin, GPIO_Typedef *t_ss_port, uint16_t t_ss_pin);
