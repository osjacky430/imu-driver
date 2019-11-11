#include "../include/imu_driver.h"

#include <stdbool.h>
#include <assert.h>
#include <stddef.h>
#include <stdlib.h>

// abstract data type definition
typedef struct ImuInterface {
	/* USART & SPI */
	void* comInterfaceType;
	GpioFuncPtr clearPin, setPin;
	Gpio reset;

	// @TODO consider using union to pack two different communication type

	/* SPI */
	SpiFuncPtr spiXfer;
	GpioFuncPtr readInputPin;
	Gpio dataReady, slaveSelect;

	/* USART */
	UsartFuncPtr usartXfer;
} ImuInterface;

typedef struct ImuDriver {
	ImuRawData m_imuRawData;
	AhrsRawData m_ahrsRawData;

	ImuInterface m_msgInterface;
	MessageFramework m_msgFramework;

	float accelScaler[3], gyroScaler[3], attitudeScaler[3];
	ImuData m_imuProcessedData;
	AhrsData m_ahrsProcessedData;
} ImuDriver;

// private function
static inline uint16_t m_imuDriverSpiGenWriteCmd(const uint8_t t_reg, const uint8_t t_cmd) {
	return (((t_reg | (1 << 8U)) << 8U) | t_cmd);
}

static inline uint16_t m_imuDriverSpiGenReadCmd(const uint8_t t_reg) { return (t_reg << 8U); }

static inline bool m_imuDriverSpiCheckDataReady(const ImuDriverPtr t_imu) {
	Gpio ready_io = t_imu->m_msgInterface.dataReady;

	return t_imu->m_msgInterface.readInputPin(ready_io.port, ready_io.pin) == 0;
}

static inline void m_imuDriverProcessImuRawData(const ImuDriverPtr t_imu) {
	for (int i = 0; i < 3; ++i) {
		t_imu->m_imuProcessedData.accel[i] = t_imu->m_imuRawData.accel[i] / t_imu->accelScaler[i];
		t_imu->m_imuProcessedData.rate[i] = t_imu->m_imuRawData.rate[i] / t_imu->gyroScaler[i];
	}
}

static inline void m_imuDriverProcessAhrsRawData(const ImuDriverPtr t_imu) {
	for (int i = 0; i < 3; ++i) {
		t_imu->m_ahrsProcessedData.attitude[i] = t_imu->m_ahrsRawData.attitude[i] / t_imu->attitudeScaler[i];
	}
}

// Imu interface member function
ImuInterfacePtr imuInterfaceCreate(Gpio t_rst, GpioFuncPtr t_clear, GpioFuncPtr t_set) {
	ImuInterfacePtr obj = calloc(1, sizeof(ImuInterface));

	if (obj != NULL) {
		assert(t_clear != NULL);
		assert(t_set != NULL);
		assert(t_rst.port != NULL);

		obj->reset = t_rst;
		obj->clearPin = t_clear;
		obj->setPin = t_set;
	}

	return obj;
}

void imuInterfaceSetupSpi(ImuInterfacePtr interface, void* t_comm, SpiFuncPtr t_xfer, Gpio t_drdy, Gpio t_ss,
						  GpioFuncPtr t_read_pin) {
	assert(interface != NULL);
	assert(t_comm != NULL);
	assert(t_xfer != NULL);
	assert(t_drdy.port != NULL);
	assert(t_ss.port != NULL);
	assert(t_read_pin != NULL);

	interface->comInterfaceType = t_comm;
	interface->spiXfer = t_xfer;
	interface->dataReady = t_drdy;
	interface->slaveSelect = t_ss;
	interface->readInputPin = t_read_pin;
}

ImuDriverStatus imuInterfaceDestroy(ImuInterfacePtr* interface_ptr) {
	if (interface_ptr == NULL) {
		return ImuDriverStatusOk;
	}

	if (*interface_ptr != NULL) {
		free(*interface_ptr);
		*interface_ptr = NULL;
	}

	return ImuDriverStatusOk;
}

// Imu driver member function
ImuDriverPtr imuDriverCreate(const MessageFramework t_fw, const ImuInterfacePtr t_interface) {
	ImuDriverPtr obj = calloc(1, sizeof(ImuDriver));

	if (obj != NULL) {
		assert(t_interface != NULL);

		obj->m_msgFramework = t_fw;
		obj->m_msgInterface = *t_interface;	 // copy

		for (int i = 0; i < 3; ++i) {
			obj->accelScaler[i] = DEFAULT_IMU_ACCEL_SCALER;
			obj->gyroScaler[i] = DEFAULT_IMU_GYRO_SCALER;
			obj->attitudeScaler[i] = DEFAULT_AHRS_ATTITUDE_SCALER;
		}
	}

	return obj;
}

ImuDriverStatus imuDriverDestroy(ImuDriverPtr* t_imu_ptr) {
	if (t_imu_ptr == NULL) {
		return ImuDriverStatusOk;
	}

	if (*t_imu_ptr != NULL) {
		free(*t_imu_ptr);
		*t_imu_ptr = NULL;
	}

	return ImuDriverStatusOk;
}

ImuDriverStatus imuDriverReceiveBurstMsg(ImuDriverPtr t_imu, bool extended) {
	Gpio slave_io = t_imu->m_msgInterface.slaveSelect;

	if (m_imuDriverSpiCheckDataReady(t_imu)) {
		void* comm_interface = t_imu->m_msgInterface.comInterfaceType;
		uint8_t max_data = 0U;
		uint16_t burst_read_reg = 0U;

		if (!extended) {
			max_data = 8U;
			burst_read_reg = m_imuDriverSpiGenReadCmd(ReadBurstDataRegister);
		} else {
			max_data = 11U;
			burst_read_reg = m_imuDriverSpiGenReadCmd(ReadExtBurstDataRegister);
		}

		t_imu->m_msgInterface.clearPin(slave_io.port, slave_io.pin);  // start transmission
		t_imu->m_msgInterface.spiXfer(comm_interface, burst_read_reg);
		static int data_count = 0;

		while (data_count < max_data) {	 // polling, @TODO interrupt-base method?
			uint16_t data = t_imu->m_msgInterface.spiXfer(comm_interface, 0x0000);
			switch (data_count) {
				case 0:
					t_imu->m_imuRawData.status = data;
					break;
				case 1:
				case 2:
				case 3:
					t_imu->m_imuRawData.rate[data_count - 1] = data;
					break;
				case 4:
				case 5:
				case 6:
					t_imu->m_imuRawData.accel[data_count - 4] = data;
					break;
				case 7:
					t_imu->m_imuRawData.temp = data;
					break;
				default:
					break;
			}

			++data_count;
		}

		t_imu->m_msgInterface.setPin(slave_io.port, slave_io.pin);	// end transmission

		m_imuDriverProcessImuRawData(t_imu);

		return ImuDriverStatusOk;
	} else {
		return ImuDriverDataNotReady;
	}
}

ImuDriverStatus imuDriverReceiveAhrsBurstMsg(ImuDriverPtr t_imu) {
	Gpio slave_io = t_imu->m_msgInterface.slaveSelect;

	if (m_imuDriverSpiCheckDataReady(t_imu)) {
		void* comm_interface = t_imu->m_msgInterface.comInterfaceType;
		uint8_t max_data = 5U;
		uint16_t burst_read_reg = m_imuDriverSpiGenReadCmd(ReadAhrsBurstDataRegister);

		t_imu->m_msgInterface.clearPin(slave_io.port, slave_io.pin);  // start transmission
		t_imu->m_msgInterface.spiXfer(comm_interface, burst_read_reg);

		static int data_count = 0;

		while (data_count < max_data) {
			uint16_t data = t_imu->m_msgInterface.spiXfer(comm_interface, 0x0000);
			switch (data_count) {
				case 0:
					t_imu->m_ahrsRawData.status = data;
					break;
				case 1:
				case 2:
				case 3:
					t_imu->m_ahrsRawData.attitude[data_count - 1] = data;
					break;
				case 4:
					t_imu->m_ahrsRawData.temp = data;
					break;
			}

			++data_count;
		}
		t_imu->m_msgInterface.setPin(slave_io.port, slave_io.pin);	// end transmission

		m_imuDriverProcessAhrsRawData(t_imu);

		return ImuDriverStatusOk;
	} else {
		return ImuDriverDataNotReady;
	}
}

MessageFramework imuDriverGetFramework(const ImuDriverPtr t_imu) {
	assert(t_imu);

	return t_imu->m_msgFramework;
}

ImuRawData imuDriverGetImuRawData(const ImuDriverPtr t_imu) {
	assert(t_imu);

	return t_imu->m_imuRawData;
}

ImuData imuDriverGetImuData(const ImuDriverPtr t_imu) {
	assert(t_imu);
	return t_imu->m_imuProcessedData;
}

AhrsRawData imuDriverGetAhrsRawData(const ImuDriverPtr t_imu) {
	assert(t_imu);

	return t_imu->m_ahrsRawData;
}

AhrsData imuDriverGetAhrsProcessedData(const ImuDriverPtr t_imu) {
	assert(t_imu);

	return t_imu->m_ahrsProcessedData;
}
