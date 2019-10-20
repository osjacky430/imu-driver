#include <gtest/gtest.h>

#include "../include/imu_driver.h"
#include "imu_mock/imu_mock.h"

// Hardware Typedef
static SPI_TypeDef spi3;
static GPIO_TypeDef gpioA;
static uint8_t gpio_pin_8 = 8U;
static uint8_t gpio_pin_9 = 9U;
static uint8_t gpio_pin_10 = 10U;

// Software bridge to hardware

static uint32_t ll_gpio_clear_pin(void *t_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef *port = (GPIO_TypeDef *)(t_gpio_port);
	LL_GPIO_ResetOutputPin(port, t_pin);

	return 0U;
}

static uint32_t ll_gpio_set_pin(void *t_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef *port = (GPIO_TypeDef *)(t_gpio_port);
	LL_GPIO_SetOutputPin(port, t_pin);

	return 0U;
}

static uint32_t ll_gpio_read_input_pin(void *t_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef *port = (GPIO_TypeDef *)(t_gpio_port);
	return LL_GPIO_IsInputPinSet(port, t_pin);
}

static uint16_t ll_spi_xfer(void *t_spi, uint16_t t_cmd) {
	SPI_TypeDef *spix = (SPI_TypeDef *)(t_spi);
	return LL_SPI_Xfer(spix, t_cmd);
}

TEST(ImuInterfaceIntialization, null_interface_member_will_trow_assertion_error) {
	Gpio bad_gpio_setup = {NULL, gpio_pin_10};
	Gpio good_gpio_setup = {&gpioA, gpio_pin_10};

	ASSERT_DEATH(imuInterfaceCreate(bad_gpio_setup, ll_gpio_clear_pin, ll_gpio_set_pin), "");
	ASSERT_DEATH(imuInterfaceCreate(good_gpio_setup, NULL, ll_gpio_set_pin), "");
	ASSERT_DEATH(imuInterfaceCreate(good_gpio_setup, ll_gpio_clear_pin, 0), "");

	ImuInterfacePtr spi_interface = imuInterfaceCreate(good_gpio_setup, ll_gpio_clear_pin, ll_gpio_set_pin);
	ASSERT_DEATH(imuInterfaceSetupSpi(spi_interface, NULL, ll_spi_xfer, good_gpio_setup, good_gpio_setup,
									  ll_gpio_read_input_pin),
				 "");
	ASSERT_DEATH(
		imuInterfaceSetupSpi(spi_interface, &spi3, NULL, good_gpio_setup, good_gpio_setup, ll_gpio_read_input_pin), "");
	ASSERT_DEATH(imuInterfaceSetupSpi(spi_interface, &spi3, ll_spi_xfer, bad_gpio_setup, good_gpio_setup,
									  ll_gpio_read_input_pin),
				 "");
	ASSERT_DEATH(imuInterfaceSetupSpi(spi_interface, &spi3, ll_spi_xfer, good_gpio_setup, bad_gpio_setup,
									  ll_gpio_read_input_pin),
				 "");
	ASSERT_DEATH(imuInterfaceSetupSpi(spi_interface, &spi3, ll_spi_xfer, good_gpio_setup, good_gpio_setup, NULL), "");

	imuInterfaceDestroy(&spi_interface);
}
