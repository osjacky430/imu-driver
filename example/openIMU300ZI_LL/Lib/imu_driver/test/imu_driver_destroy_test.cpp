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
static Gpio reset_pin = {&gpioA, gpio_pin_10};
static Gpio drdy_pin = {&gpioA, gpio_pin_8};
static Gpio ss_pin = {&gpioA, gpio_pin_9};

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

TEST(ImuDriverDestruction, destruction_should_null_object_and_always_return_OK) {
	ImuInterfacePtr spi_interface = imuInterfaceCreate(reset_pin, __ll_gpio_clear_pin, __ll_gpio_set_pin);
	imuInterfaceSetupSpi(spi_interface, &spi3, __ll_spi_xfer, drdy_pin, ss_pin, __ll_gpio_read_input_pin);
	ImuDriverPtr obj = imuDriverCreate(FrameworkSpi, spi_interface);

	auto status = imuDriverDestroy(&obj);

	EXPECT_TRUE(obj == NULL);
	EXPECT_EQ(status, ImuDriverStatusOk);

	imuInterfaceDestroy(&spi_interface);
}
