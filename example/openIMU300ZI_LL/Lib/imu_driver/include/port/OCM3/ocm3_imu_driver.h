#pragma once

#include <stdint.h>
#include "../../imu_driver.h"

#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/spi.h"

ImuInterfacePtr OCM3_ImuInterfaceCreate(uint32_t t_reset_port, uint16_t t_reset_pin);

void OCM3_ImuInterfaceSetupSpi(ImuInterfacePtr t_interface, uint32_t spi, uint32_t t_drdy_port, uint16_t t_drdy_pin,
							   uint32_t t_ss_port, uint16_t t_ss_pin);
