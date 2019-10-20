#include "imu_mock.h"
#include "ll_mock.h"

std::unique_ptr<MockImuInterface> ImuDriverReceive::m_imuMock;

void LL_GPIO_ResetOutputPin(GPIO_TypeDef *gpio, uint32_t mask) {
	ImuDriverReceive::m_imuMock->m_LL_GPIO_ResetOutputPin(gpio, mask);
}

void LL_GPIO_SetOutputPin(GPIO_TypeDef *gpio, uint32_t mask) {
	ImuDriverReceive::m_imuMock->m_LL_GPIO_SetOutputPin(gpio, mask);
}

uint32_t LL_GPIO_IsInputPinSet(GPIO_TypeDef *gpio, uint32_t mask) {
	return ImuDriverReceive::m_imuMock->m_LL_GPIO_IsInputPinSet(gpio, mask);
}

uint16_t LL_SPI_Xfer(SPI_TypeDef *spi, uint16_t t_reg) {
	// Assuming that 
	// 
	//	1) this function will wait for transfer complete
	//	2) this function will make sure the buffer is received

	return ImuDriverReceive::m_imuMock->m_LL_SPI_Xfer(spi, t_reg);
}