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
#pragma once

#include "libopencm3/cm3/nvic.h"
#include "libopencm3/stm32/exti.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/spi.h"
#include "libopencm3/stm32/usart.h"

#define LED_GPIO_Port GPIOA
#define LED_GPIO_Pin GPIO5

#define IMU_MISO_Port GPIOB
#define IMU_MISO_Pin GPIO4
#define IMU_MOSI_Port GPIOB
#define IMU_MOSI_Pin GPIO5
#define IMU_SCK_Port GPIOB
#define IMU_SCK_Pin GPIO3

#define IMU_RST_Port GPIOA
#define IMU_RST_Pin GPIO10
#define IMU_DRDY_Port GPIOA
#define IMU_DRDY_Pin GPIO8
#define IMU_SS_Port GPIOA
#define IMU_SS_Pin GPIO9

#define VCP_TX_Port GPIOA
#define VCP_TX_Pin GPIO2
#define VCP_RX_Port GPIOA
#define VCP_RX_Pin GPIO3
