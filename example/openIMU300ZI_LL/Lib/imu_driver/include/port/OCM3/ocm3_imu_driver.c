#include "ocm3_imu_driver.h"

static inline uint32_t ocm3_gpio_read_pin(void* ocm3_gpio_port, uint32_t t_pin) {
	uint32_t port = (uint32_t)ocm3_gpio_port;
	return gpio_get(port, t_pin);
}

static inline uint16_t ocm3_spi_xfer(void* t_spi, uint16_t t_cmd) {
	uint32_t spi = (uint32_t)t_spi;
	spi_send(spi, t_cmd);
	return spi_read(spi);
}

static inline uint32_t ocm3_gpio_set(void* ocm3_gpio_port, uint32_t t_pin) {
	uint32_t port = (uint32_t)ocm3_gpio_port;
	gpio_set(port, t_pin);

	return 0;
}

static inline uint32_t ocm3_gpio_clear(void* ocm3_gpio_port, uint32_t t_pin) {
	uint32_t port = (uint32_t)ocm3_gpio_port;
	gpio_clear(port, t_pin);

	return 0;
}

ImuInterfacePtr OCM3_ImuInterfaceCreate(uint32_t t_reset_port, uint16_t t_reset_pin) {
	Gpio rst = {(uint32_t*)t_reset_port, t_reset_pin};

	ImuInterfacePtr obj = imuInterfaceCreate(rst, ocm3_gpio_clear, ocm3_gpio_set);

	return obj;
}

void OCM3_ImuInterfaceSetupSpi(ImuInterfacePtr t_interface, uint32_t spi, uint32_t t_drdy_port, uint16_t t_drdy_pin,
							   uint32_t t_ss_port, uint16_t t_ss_pin) {
	Gpio drdy = {(uint32_t*)t_drdy_port, t_drdy_pin};
	Gpio ss = {(uint32_t*)t_ss_port, t_ss_pin};

	imuInterfaceSetupSpi(t_interface, (uint32_t*)spi, ocm3_spi_xfer, drdy, ss, ocm3_gpio_read_pin);
}
