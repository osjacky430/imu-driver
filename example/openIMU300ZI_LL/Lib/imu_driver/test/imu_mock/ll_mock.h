#pragma once

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

	typedef struct {

	} GPIO_TypeDef;

	typedef struct {

	} SPI_TypeDef;

	void LL_GPIO_ResetOutputPin(GPIO_TypeDef* t_gpio, uint32_t t_mask);
	void LL_GPIO_SetOutputPin(GPIO_TypeDef* t_gpio, uint32_t t_mask);
	uint32_t LL_GPIO_IsInputPinSet(GPIO_TypeDef* t_gpio, uint32_t t_mask);
	uint16_t LL_SPI_Xfer(SPI_TypeDef* t_spi, uint16_t t_reg);

#if defined __cplusplus
}
#endif