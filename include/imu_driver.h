#pragma once

#include <stdbool.h>
#include <stdint.h>

#define DEFAULT_IMU_ACCEL_SCALER 4000.0f
#define DEFAULT_IMU_GYRO_SCALER 200.0f
#define DEFAULT_AHRS_ATTITUDE_SCALER 1000.0f

typedef uint16_t (*SpiFuncPtr)(void*, uint16_t);
typedef uint32_t (*GpioFuncPtr)(void*, uint32_t);
typedef void (*UsartFuncPtr)(void*, uint32_t);

typedef enum ImuDriverRegister {
	ReadAhrsBurstDataRegister = 0x3D,
	ReadBurstDataRegister = 0x3E,
	ReadExtBurstDataRegister = 0x3F
} ImuDriverRegister;

typedef enum ImuDriverStatus { ImuDriverStatusOk, ImuDriverDataNotReady } ImuDriverStatus;

typedef enum MessageFramework { FrameworkSpi, FrameworkUsart, FrameworkNone } MessageFramework;

typedef struct ImuRawData {
	uint16_t status;
	int16_t rate[3];
	int16_t accel[3];
	int16_t temp;
} ImuRawData;

typedef struct ImuData {
	float rate[3];
	float accel[3];
} ImuData;

typedef struct AhrsRawData {
	uint16_t status;
	int16_t attitude[3];
	int16_t temp;
} AhrsRawData;

typedef struct AhrsData {
	float attitude[3];
} AhrsData;

typedef struct Gpio {
	void* port;
	uint32_t pin;
} Gpio;

typedef struct ImuInterface* ImuInterfacePtr;
typedef struct ImuDriver* ImuDriverPtr;

#if defined __cplusplus
extern "C" {
#endif

/**
 *	@brief	This function handles the construction of imu communication interface
 *
 *	@param	t_rst	reset pin Gpio struct
 *	@param	t_clear	gpio function clear pin
 *	@param	t_set	gpio function set pin
 *
 *	@return	an ImuInterfacePtr
 */
ImuInterfacePtr imuInterfaceCreate(Gpio t_rst, GpioFuncPtr t_clear, GpioFuncPtr t_set);

/**
 *	@brief	This function handles the setup of spi communication
 *
 *	@param	interface		ImuInterfacePtr struct
 *	@param	t_comm			pointer to communication interface type
 *	@param	t_xfer			spi function transfer/receive
 *	@param	t_drdy			data ready pin Gpio struct
 *	@param	t_ss			slave select pin Gpio struct
 *	@param	t_read_input	gpio function read pin
 */
void imuInterfaceSetupSpi(ImuInterfacePtr interface, void* t_comm, SpiFuncPtr t_xfer, Gpio t_drdy, Gpio t_ss,
						  GpioFuncPtr t_read_input);

/**
 *	@brief	This function handles the destruction of imu interface struct
 *
 *	@param	t_interface_ptr	Pointer to ImuInterfacePtr struct
 *
 *	@return	ImuDriverStatus to indicate the result of destruction
 *
 *	@note The interface ptr passed through will be nulled
 */
ImuDriverStatus imuInterfaceDestroy(ImuInterfacePtr* t_interface_ptr);

/**
 *	@brief	This function handles the construction of imu driver struct
 *
 *	@param	t_fw			MessageFramework to indicate the current communication type
 *	@param	t_interface		Interface struct
 *
 *	@return An imuDriverPtr struct
 */
ImuDriverPtr imuDriverCreate(MessageFramework t_fw, const ImuInterfacePtr t_interface);

/**
 *	@brief	This function handles the destruction of imu driver struct
 *
 *	@param	t_imu_ptr	Pointer to imuDriverPtr struct
 *
 *	@return ImuDriverStatus to indicate the result of the destruction
 *
 *	@note The imu ptr passed through will be nulled
 */
ImuDriverStatus imuDriverDestroy(ImuDriverPtr* t_imu_ptr);

/**
 *	@brief	This function handles the receive of burst message function of openIMU
 *
 *	@pararm t_imu		ImuDriverPtr struct
 *	@param	t_extended	bool to indicate the message type is extended or not
 *
 *	@return ImuDriverStatus to indicate the result of the receive operation
 */
ImuDriverStatus imuDriverReceiveBurstMsg(ImuDriverPtr t_imu, bool t_extended);

/**
 *	@brief	This function handles the receive of AHRS burst message function
 *
 *	@param t_imu		ImuDriverPtr struct
 *
 *	@return ImuDriverStatus to indicate the result of the receive operation
 *
 *	@note	This is only suitable for AHRS app, and additional procedure needs to be done
 *			in order to make the work
 */
ImuDriverStatus imuDriverReceiveAhrsBurstMsg(ImuDriverPtr t_imu);

/**
 *	@brief	This function is getter function to get current framework
 *
 *	@param	t_imu	ImuDriverPtr struct
 *
 *	@return the message framework of ImuDriver passed to the function
 */
MessageFramework imuDriverGetFramework(const ImuDriverPtr t_imu);

/**
 *	@brief This function is getter function to get the raw imu data
 *
 *	@param	t_imu	ImuDriverPtr struct
 *
 *	@return the raw imu data copy
 */
ImuRawData imuDriverGetImuRawData(const ImuDriverPtr t_imu);

/**
 *	@brief This function is getter function to get the processed imu data
 *
 *	@param t_imu	ImuDriverPtr struct
 *
 *	@return the processed imu data copy
 */
ImuData imuDriverGetImuData(const ImuDriverPtr t_imu);

/**
 *	@brief This function is getter function to get the raw AHRS data
 *
 *	@param	t_imu	ImuDriverPtr struct
 *
 *	@return the raw ahrs data copy
 */
AhrsRawData imuDriverGetAhrsRawData(const ImuDriverPtr t_imu);

/**
 *	@brief	This function is getter function to get the processed AHRS data
 *
 *	@param t_imu	ImuDriverPtr struct
 *
 *	@return the processed ahrs data copy
 */
AhrsData imuDriverGetAhrsProcessedData(const ImuDriverPtr t_imu);

#if defined __cplusplus
}
#endif
