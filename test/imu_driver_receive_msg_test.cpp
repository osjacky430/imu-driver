#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../include/imu_driver.h"
#include "imu_mock/imu_mock.h"

using testing::Return;

// Hardware driver struct, these structs and numbers are usually defined globally
static SPI_TypeDef spi_3;
static GPIO_TypeDef gpio_a;
static uint8_t gpio_pin_8 = 8U;	// arbitrary pin number for data ready(DRDY) pin
static uint8_t gpio_pin_9 = 9U;	// arbitrary pin number for slave select(SS) pin
static uint8_t gpio_pin_10 = 10U;  // arbitrary pin number for reset (nRST) pin

// Software bridge to hardware
static Gpio imu_drdy = {&gpio_a, gpio_pin_8};
static Gpio imu_ss = {&gpio_a, gpio_pin_9};
static Gpio imu_rst = {&gpio_a, gpio_pin_10};

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

// Test
TEST_F(ImuDriverReceive, imuDriverReceiveMsg_return_ImuDriverDataNotReady_if_DRDY_pin_not_set) {
	EXPECT_CALL(*ImuDriverReceive::m_imuMock, m_LL_GPIO_IsInputPinSet(&gpio_a, gpio_pin_8)).WillOnce(Return(0));

	ImuInterfacePtr spi_interface = imuInterfaceCreate(imu_rst, __ll_gpio_clear_pin, __ll_gpio_set_pin);
	imuInterfaceSetupSpi(spi_interface, &spi_3, __ll_spi_xfer, imu_drdy, imu_ss, __ll_gpio_read_input_pin);
	ImuDriverPtr imu = imuDriverCreate(FrameworkSpi, spi_interface);

	auto status = imuDriverReceiveBurstMsg(imu, false);

	EXPECT_EQ(status, ImuDriverDataNotReady);

	imuDriverDestroy(&imu);
	imuInterfaceDestroy(&spi_interface);
}

TEST_F(ImuDriverReceive, imuDriverReceiveMsg_return_ImuDriverStatusOk_if_no_error) {
	EXPECT_CALL(*ImuDriverReceive::m_imuMock, m_LL_GPIO_IsInputPinSet(&gpio_a, gpio_pin_8)).WillOnce(Return(1));
	EXPECT_CALL(*ImuDriverReceive::m_imuMock, m_LL_SPI_Xfer(&spi_3, 0x3E00))
		.Times(1)
		.WillOnce(Return(0));  // First packet of Read operation is N/A
	EXPECT_CALL(*ImuDriverReceive::m_imuMock, m_LL_SPI_Xfer(&spi_3, 0x0000))
		.Times(8)
		.WillOnce(Return(0))  // status
		.WillOnce(Return(0))
		.WillOnce(Return(0))
		.WillOnce(Return(0))  // rate
		.WillOnce(Return(0))
		.WillOnce(Return(0))
		.WillOnce(Return(1))	// accel
		.WillOnce(Return(26));  // temperature
	EXPECT_CALL(*ImuDriverReceive::m_imuMock, m_LL_GPIO_ResetOutputPin(&gpio_a, gpio_pin_9));
	EXPECT_CALL(*ImuDriverReceive::m_imuMock, m_LL_GPIO_SetOutputPin(&gpio_a, gpio_pin_9));

	// Imu driver interface struct
	Gpio imu_drdy = {(void *)(&gpio_a), gpio_pin_8};
	Gpio imu_ss = {(void *)(&gpio_a), gpio_pin_9};

	ImuInterfacePtr spi_interface = imuInterfaceCreate(imu_rst, __ll_gpio_clear_pin, __ll_gpio_set_pin);
	imuInterfaceSetupSpi(spi_interface, &spi_3, __ll_spi_xfer, imu_drdy, imu_ss, __ll_gpio_read_input_pin);

	ImuDriverPtr imu = imuDriverCreate(FrameworkSpi, spi_interface);

	auto status = imuDriverReceiveBurstMsg(imu, false);

	EXPECT_EQ(status, ImuDriverStatusOk);

	imuDriverDestroy(&imu);
}
