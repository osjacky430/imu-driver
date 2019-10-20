#include "hal_imu_driver.h"

static inline uint16_t hal_spi_xfer16(void* t_spi, uint16_t cmd) {
	SPI_HandleTypeDef* spi = t_spi;
	uint16_t rx_data;

	HAL_SPI_TransmitReceive(spi, (uint8_t*)&cmd, (uint8_t*)&rx_data, 1, HAL_MAX_DELAY);

	return rx_data;
}

static inline uint32_t hal_gpio_read_pin(void* hal_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef* port = hal_gpio_port;
	return HAL_GPIO_ReadPin(port, t_pin);
}

static inline uint32_t hal_gpio_set(void* hal_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef* port = hal_gpio_port;
	HAL_GPIO_WritePin(port, t_pin, GPIO_PIN_SET);

	return 0;
}

static inline uint32_t hal_gpio_clear(void* hal_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef* port = hal_gpio_port;
	HAL_GPIO_WritePin(port, t_pin, GPIO_PIN_RESET);

	return 0;
}

ImuInterfacePtr HAL_ImuInterfaceCreate(GPIO_TypeDef* t_reset_port, uint16_t t_reset_pin) {
	Gpio reset = {t_reset_port, t_reset_pin};

	ImuInterfacePtr obj = imuInterfaceCreate(reset, hal_gpio_clear, hal_gpio_set);

	return obj;
}

void HAL_ImuInterfaceSetupSpi(ImuInterfacePtr t_interface, SPI_HandleTypeDef* spi, GPIO_TypeDef* t_drdy_port,
							  uint16_t t_drdy_pin, GPIO_TypeDef* t_ss_port, uint16_t t_ss_pin) {
	Gpio drdy = {t_drdy_port, t_drdy_pin};
	Gpio ss = {t_ss_port, t_ss_pin};

	imuInterfaceSetupSpi(t_interface, spi, hal_spi_xfer16, drdy, ss, hal_gpio_read_pin);
}
