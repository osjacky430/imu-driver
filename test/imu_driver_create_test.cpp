#include <gmock/gmock.h>

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

static uint32_t __ll_gpio_clear_pin(void *t_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef *port = (GPIO_TypeDef *)(t_gpio_port);
	LL_GPIO_ResetOutputPin(port, t_pin);

	return 0U;
}

static uint32_t __ll_gpio_set_pin(void *t_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef *port = (GPIO_TypeDef *)(t_gpio_port);
	LL_GPIO_SetOutputPin(port, t_pin);

	return 0U;
}

static uint32_t __ll_gpio_read_input_pin(void *t_gpio_port, uint32_t t_pin) {
	GPIO_TypeDef *port = (GPIO_TypeDef *)(t_gpio_port);
	return LL_GPIO_IsInputPinSet(port, t_pin);
}

static uint16_t __ll_spi_xfer(void *t_spi, uint16_t t_cmd) {
	SPI_TypeDef *spix = (SPI_TypeDef *)(t_spi);
	return LL_SPI_Xfer(spix, t_cmd);
}

TEST(ImuDriverInitialization, variable_is_initialized_correctly_will_not_throw_any_assertion) {
	Gpio reset_pin = {&gpioA, gpio_pin_10};
	Gpio drdy_pin = {&gpioA, gpio_pin_8};
	Gpio ss_pin = {&gpioA, gpio_pin_9};

	ImuInterfacePtr spi_interface = imuInterfaceCreate(reset_pin, __ll_gpio_clear_pin, __ll_gpio_set_pin);
	imuInterfaceSetupSpi(spi_interface, &spi3, __ll_spi_xfer, drdy_pin, ss_pin, __ll_gpio_read_input_pin);
	ImuDriverPtr obj = imuDriverCreate(FrameworkSpi, spi_interface);

	EXPECT_TRUE(obj);
	EXPECT_EQ(imuDriverGetFramework(obj), FrameworkSpi);

	imuDriverDestroy(&obj);
	imuInterfaceDestroy(&spi_interface);
}

TEST(ImuDriverInitialization, null_interface_will_throw_assertion_error) {
	ASSERT_DEATH(imuDriverCreate(FrameworkSpi, 0), "");
}
