#include "ll_imu_driver.h"

static inline void ll_spi_wait_tx_complete(SPI_Typedef *SPIx) {
	while (!LL_SPI_IsActiveFlag_TXE(SPIx)) {
	}
}

static inline void ll_spi_wait_rx_buffer_ready(SPI_Typedef *SPIx) {
	while (!LL_SPI_IsActiveFlag_RXNE(SPIx)) {
	}
}

static inline uint16_t ll_spi_xfer16(void *t_spi, uint16_t t_cmd) {
	SPI_Typedef *SPIx = t_spi;

	ll_spi_wait_tx_complete(SPIx);
	LL_SPI_TransmitData16(SPIx, t_cmd);
	ll_spi_wait_rx_buffer_ready(SPIx);
	return LL_SPI_ReceiveData16(SPIx);
}

static inline uint32_t ll_gpio_clear(void *t_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef *port = (GPIO_TypeDef *)(t_gpio_port);
	LL_GPIO_ResetOutputPin(port, t_pin);

	return 0U;
}

static inline uint32_t ll_gpio_set(void *t_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef *port = (GPIO_TypeDef *)(t_gpio_port);
	LL_GPIO_SetOutputPin(port, t_pin);

	return 0U;
}

static inline uint32_t ll_gpio_read_pin(void *t_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef *port = (GPIO_TypeDef *)(t_gpio_port);
	return LL_GPIO_IsInputPinSet(port, t_pin);
}

ImuInterfacePtr LL_ImuInterfaceCreate(GPIO_Typedef *t_reset_port, uint16_t t_reset_pin) {
	Gpio reset = {t_reset_port, t_reset_pin};

	ImuInterfacePtr obj = imuInterfaceCreate(reset, ll_gpio_clear, ll_gpio_set);

	return obj;
}

void LL_ImuInterfaceSetupSpi(ImuInterfacePtr t_interface, SPI_Typedef *spi, GPIO_Typedef *t_drdy_port,
							 uint16_t t_drdy_pin, GPIO_Typedef *t_ss_port, uint16_t t_ss_pin) {
	Gpio drdy = {t_drdy_port, t_drdy_pin};
	Gpio ss = {t_ss_port, t_ss_pin};

	imuInterfaceSetupSpi(t_interface, spi, ll_spi_xfer16, drdy, ss, ll_gpio_read_pin);
}
