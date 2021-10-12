#pragma once

#include "math_routines.h"
#include "hardware_interface.h"
#include "crc16.h"
#include "leds.h"
#include "global_parameters.h"


class SerialCommunicator
{
	volatile struct SCI_REGS * sci;

	static const uint16_t FRAME_HEADER 		= 0xAA;
	static const uint16_t FRAME_MAX_SIZE 	= 64;
	static const uint16_t FRAME_MIN_SIZE 	= 8;

	static const uint16_t timeOut = 10;

	enum rxState {IDLE, HEADER_READ, SIZE_READ, GOOD_FRAME_RECEIVED};

	uint16_t txIndex;
	uint16_t txTotalPacketsNumber;
	uint16_t txLastPacketSize;

	uint16_t rxIndex;
	uint16_t rxIndexHead;
	uint16_t rxIndexTail;

	uint16_t rxTotalPacketsNumber;
	uint16_t rxLastPacketSize;
	uint16_t rxFifoSize;

    volatile uint16_t rxState;

    volatile uint16_t waitLastTxPacket 	: 1;
    volatile uint16_t waitLastRxPacket 	: 1;

	uint16_t time;

	void HardwareInit(uint32_t baudrate);

public:
	uint16_t txBuf[128];
	uint16_t rxFrame[128];
	uint16_t rxBuf[128];

    volatile uint16_t isTransmitted 	: 1;
    volatile uint16_t isReceived 		: 1;

	void Init(volatile struct SCI_REGS * sci, uint32_t baudrate)
	{
		this->sci = sci;
		time = 0;

		isReceived = false;
		rxIndex = rxIndexTail = 0;
		rxState = IDLE;

		HardwareInit(baudrate);
	}

	void StartReceive(void)
	{
		isReceived = false;
		waitLastRxPacket = false;
		rxIndex = rxIndexTail = 0;
		rxState = IDLE;

		rxFifoSize = FRAME_MIN_SIZE;
		sci->SCIFFRX.bit.RXFFIL = rxFifoSize;
		sci->SCIFFRX.bit.RXFIFORESET=1;
	}

	void StartSend();
	void SendIrq();
	void ReceiveIrq();

	void CheckErrors()
	{
		if (sci->SCIRXST.bit.RXERROR == 1)	// transmission error
		{
			Led::Blink(3, 200);
			sci->SCICTL1.bit.SWRESET = 0;	// clear error
			sci->SCICTL1.bit.SWRESET = 1;	// go to normal mode
			sci->SCIFFTX.bit.TXFIFOXRESET=1;
			sci->SCIFFRX.bit.RXFIFORESET=1;
		}
	}

	void CheckTimeOut(void)
	{
		if (time != 0)
		{
			time--;
			if (time == 0)
			{
				StartReceive();
				Led::Blink(3, 750);
			}
		}
	}

};
