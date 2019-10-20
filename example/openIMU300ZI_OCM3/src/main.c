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
#include <stdio.h>

#include "hardware_define/hardware_define.h"
#include "imu_driver/include/port/OCM3/ocm3_imu_driver.h"

ImuInterfacePtr imu_interface;
ImuDriverPtr imu_driver;

static inline void setup_imu_driver(void) {
	imu_interface = OCM3_ImuInterfaceCreate(IMU_RST_Port, IMU_DRDY_Pin);
	OCM3_ImuInterfaceSetupSpi(imu_interface, SPI3, IMU_DRDY_Port, IMU_DRDY_Pin, IMU_SS_Port,
							  IMU_SS_Pin);

	imu_driver = imuDriverCreate(FrameworkSpi, imu_interface);
}

static inline void start_sampling(void) {
	spi_enable(SPI3);
	exti_enable_request(EXTI8);
}

int main() {
	setup_imu_driver();
	start_sampling();

	ImuData data;
	while (true) {
		for (int i = 0; i < 100000; ++i) {
		}

		data = imuDriverGetImuData(imu_driver);

		float x_rate = (data.rate[0]) / 200.0f;
		float y_rate = (data.rate[1]) / 200.0f;
		float z_rate = (data.rate[2]) / 200.0f;

		float x_accel = (data.accel[0]) / 4000.0f;
		float y_accel = (data.accel[1]) / 4000.0f;
		float z_accel = (data.accel[2]) / 4000.0f;

		printf("% .3f, % .3f, % .3f, % .3f, % .3f, % .3f, % d\r\n", x_rate, y_rate, z_rate, x_accel,
			   y_accel, z_accel, data.temp);
	}

	return 0;
}

void exti9_5_isr(void) {
	exti_reset_request(EXTI8);

	gpio_toggle(LED_GPIO_Port, LED_GPIO_Pin);

	imuDriverReceiveBurstMsg(imu_driver, false);
}
