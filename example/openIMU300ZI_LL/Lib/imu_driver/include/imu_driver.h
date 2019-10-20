#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef uint16_t (*SpiFuncPtr)(void*, uint16_t);
typedef uint32_t (*GpioFuncPtr)(void*, uint32_t);
typedef void (*UsartFuncPtr)(void*, uint32_t);

typedef enum ImuDriverRegister { ReadBurstDataRegister = 0x3E, ReadExtBurstDataRegister = 0x3F } ImuDriverRegister;

typedef enum ImuDriverStatus { ImuDriverStatusOk, ImuDriverDataNotReady } ImuDriverStatus;

typedef enum MessageFramework { FrameworkSpi, FrameworkUsart, FrameworkNone } MessageFramework;

typedef struct ImuData {
	uint16_t status;
	int16_t rate[3];
	int16_t accel[3];
	int16_t temp;
} ImuData;

typedef struct Gpio {
	void* port;
	uint32_t pin;
} Gpio;

typedef struct ImuInterface* ImuInterfacePtr;
typedef struct ImuDriver* ImuDriverPtr;

#if defined __cplusplus
extern "C" {
#endif

ImuInterfacePtr imuInterfaceCreate(Gpio t_rst, GpioFuncPtr t_clear, GpioFuncPtr t_set);
void imuInterfaceSetupSpi(ImuInterfacePtr interface, void* t_comm, SpiFuncPtr t_xfer, Gpio t_drdy, Gpio ss,
						  GpioFuncPtr t_read_input);
ImuDriverStatus imuInterfaceDestroy(ImuInterfacePtr* t_interface_ptr);

ImuDriverPtr imuDriverCreate(MessageFramework t_fw, const ImuInterfacePtr t_interface);
ImuDriverStatus imuDriverDestroy(ImuDriverPtr* t_imu_ptr);

ImuDriverStatus imuDriverReceiveBurstMsg(ImuDriverPtr t_imu, bool extended);
ImuData imuDriverGetImuData(const ImuDriverPtr t_imu);
MessageFramework imuDriverGetFramework(const ImuDriverPtr t_imu);

#if defined __cplusplus
}
#endif
