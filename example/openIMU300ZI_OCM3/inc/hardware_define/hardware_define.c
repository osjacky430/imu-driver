/*******************************************************************************
 * Copyright (C) 2019 osjacky430
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/
#include "hardware_define/hardware_define.h"

CTOR(101) static inline void init_rcc(void) {
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_180MHZ]);
}

CTOR(102) static inline void setup_led(void) {
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(LED_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_GPIO_Pin);
}

CTOR(103) static inline void setup_imu_spi(void) {
	rcc_periph_clock_enable(RCC_SPI3);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);

	// spi MOSI, MISO, SCK setup
	gpio_mode_setup(IMU_MOSI_Port, GPIO_MODE_AF, GPIO_PUPD_NONE,
					IMU_MISO_Pin | IMU_MOSI_Pin | IMU_SCK_Pin);
	gpio_set_af(IMU_MOSI_Port, GPIO_AF6, IMU_MISO_Pin | IMU_MOSI_Pin | IMU_SCK_Pin);

	// spi SS setup
	gpio_mode_setup(IMU_SS_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, IMU_SS_Pin | IMU_RST_Pin);

	spi_init_master(SPI3, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
					SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_16BIT, SPI_CR1_MSBFIRST);
	spi_enable_ss_output(SPI3);
}

CTOR(104) static inline void setup_vcp(void) {
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(VCP_TX_Port, GPIO_MODE_AF, GPIO_PUPD_NONE, VCP_TX_Pin | VCP_RX_Pin);
	gpio_set_af(VCP_TX_Port, GPIO_AF7, VCP_TX_Pin | VCP_RX_Pin);

	usart_set_baudrate(USART2, 115200);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_mode(USART2, USART_MODE_TX_RX);

	usart_enable(USART2);
}

CTOR(105) static inline void setup_imu_exti(void) {
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(IMU_DRDY_Port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, IMU_DRDY_Pin);
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);
	exti_select_source(EXTI8, GPIOA);
	exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
}
