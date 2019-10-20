#pragma once

#include <gmock/gmock.h>
#include <stdint.h>

#include "ll_mock.h"

struct MockImuInterface {
public:
	virtual ~MockImuInterface() {}

	MOCK_METHOD2(m_LL_GPIO_ResetOutputPin, void(GPIO_TypeDef*, uint32_t));
	MOCK_METHOD2(m_LL_GPIO_SetOutputPin, void(GPIO_TypeDef*, uint32_t));
	MOCK_METHOD2(m_LL_SPI_Xfer, uint16_t(SPI_TypeDef*, uint16_t));
	MOCK_METHOD2(m_LL_GPIO_IsInputPinSet, uint32_t(GPIO_TypeDef*, uint32_t));
};

class ImuDriverReceive : public ::testing::Test {
public:
	ImuDriverReceive() {
		m_imuMock.reset(new ::testing::NiceMock<MockImuInterface>());
	}

	~ImuDriverReceive() {
		m_imuMock.reset();
	}
public:
	static std::unique_ptr<MockImuInterface> m_imuMock;
};

