#pragma once

#include "math_routines.h"
#include "hardware_interface.h"
#include "crc16.h"
#include "leds.h"
#include "global_parameters.h"

class I2cCommunicator
{
	void HardwareInit();
public:
	void Init()
	{
		HardwareInit();
	}

	void TransmitData(Uint16 TxCount, unsigned char *txarray, Uint16 RxCount, unsigned char *rxarray, Uint16 slave);
	void I2C_Wait(void);
	void I2cHandler();
};
