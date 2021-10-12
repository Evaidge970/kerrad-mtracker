#include "serial_communication.h"

void SerialCommunicator::HardwareInit(uint32_t baudrate)
{

	uint16_t PRD=(LSPCLK_FREQ/(baudrate*8))-1;

	// 1 stop bit, no loopback, no parity, 8 char bits, async mode, idle-line protocol
	sci->SCICCR.all = 0x0007;

	// enable TX, RX, internal SCICLK, disable RX ERR, SLEEP, TXWAKE
	sci->SCICTL1.all = 0x0003;

	// set baudrate
	sci->SCIHBAUD = (PRD >> 8);
	sci->SCILBAUD = (PRD & 0x00FF);

	sci->SCIFFTX.all = 0xC008;

	// TXFFIL4–0 Transmit FIFO interrupt level bits, transmit FIFO will generate interrupt when the FIFO status bits (TXFFST4–0)
	// and FIFO level bits (TXFFIL4–0 ) match (less than or equal to).
	sci->SCIFFTX.bit.TXFFIL = 0;

	sci->SCIFFRX.all= 0x0028;
	sci->SCIFFRX.bit.RXFFIL = FRAME_MIN_SIZE;
	sci->SCIFFCT.all= 0x00;

	//Free run. Continues SCI operation regardless of suspend
	sci->SCIPRI.bit.FREE = 1;
	// Relinquish SCI from Reset
	sci->SCICTL1.all = 0x0023;
}

void SerialCommunicator::StartSend()
{
	isTransmitted = false;
	waitLastTxPacket = false;

	uint16_t size = txBuf[0];

	txIndex = 0;

	uint16_t crc = CRC16Init();
	for (int i = 0; i< size + 1; i++)
		crc = CRC16Calc(crc, txBuf[txIndex++]);

	txBuf[txIndex++] = crc >> 8;
	txBuf[txIndex++] = crc & 0x00FF;

	// total frame size (crc + length char)
	size += 3;
	txTotalPacketsNumber = size >> 4;
	txLastPacketSize = size & 0x000F;

	if(txLastPacketSize == 0)
	{
		txTotalPacketsNumber--;
		txLastPacketSize = 16;
	}

	txIndex = 0;
	sci->SCIFFTX.bit.TXFIFOXRESET = 1;			// Re-enable transmit FIFO operation
	sci->SCITXBUF = FRAME_HEADER;				// start -> send header
	sci->SCIFFTX.bit.TXFFIENA = 1;				// TX FIFO interrupt based on TXFFIVL match (less than or equal to) is enabled.
}

void SerialCommunicator::SendIrq()
{
	if(waitLastTxPacket == true)
	{
		waitLastTxPacket = false;
		sci->SCIFFTX.bit.TXFIFOXRESET=0;	// Reset the FIFO pointer to zero and hold in reset
		sci->SCIFFTX.bit.TXFFIENA = 0;		// TX FIFO interrupt based on TXFFIVL match (less than or equal to) is disabled

		isTransmitted = true;
	}
	else
	{
		if (txTotalPacketsNumber == 0)
		{
			for(uint16_t i=0; i < txLastPacketSize; i++)
				sci->SCITXBUF = txBuf[txIndex++];


			waitLastTxPacket = true;
		}
		else
		{
			for(uint16_t i=0; i < 16 ; i++)
				sci->SCITXBUF= txBuf[txIndex++];     // Send data

			txTotalPacketsNumber--;
		}
	}
}

void SerialCommunicator::ReceiveIrq(void)
{
    // copy data from UART FIFO to cyclic buffer
    for(uint16_t i = 0; i < rxFifoSize; i++)
    {
        rxBuf[rxIndexTail++] = sci->SCIRXBUF.all;
        rxIndexTail &= 128-1;
    }

	if (waitLastRxPacket == false)
	{

		uint16_t index;
		time = timeOut;

		switch (rxState)
		{
			case HEADER_READ:
				if (rxBuf[rxIndex] < FRAME_MAX_SIZE)
				{
					index = (uint16_t)(-1);
					rxState = SIZE_READ;
				}
			break;

			default:		// frame header is not detected yet -> look for it
				rxState = IDLE;
				for(index = 0; index < rxFifoSize; index++)
				{
					if (rxBuf[rxIndex++] == FRAME_HEADER)
					{
						rxState = HEADER_READ;									//wykryto potencjalny poczatek pakietu
						rxIndex &= 128-1;
						rxIndexHead = rxIndex;							//ustawienie wskaŸnika na potencjalny poczatek pakietu - na bajt dlugoœci

						if (index < rxFifoSize - 1)
						{
							if (rxBuf[rxIndex] < FRAME_MAX_SIZE) 			//czy bajt d³ugoœci mniejszy od maksymalnej dlugoœci pakietu
							{
								rxState = SIZE_READ;
								break;
							}
						}
					}
				}
			break;
		}

		switch (rxState)
		{
			case SIZE_READ:
			{
				uint16_t size = rxBuf[rxIndex] - 4 + index;		//ile bajtów zosta³o do odczytania
				if (size == 0)
				{
					time = 0;
					isReceived = true;
					rxState = GOOD_FRAME_RECEIVED; // odebrano  kompletn¹ ramkê
				}
				else
				{
					rxTotalPacketsNumber = size >> 3;       // each packet is 8 bytes long
					rxLastPacketSize = size & 0x0007;		//d³ugoœæ ostatniego bloku do odebrania
					if (rxLastPacketSize == 0)				//iloœæ bajtow do odebrania jes wielokrotnoscia 8
					{
						rxLastPacketSize = 8;
						rxTotalPacketsNumber--;
					}
					if (rxTotalPacketsNumber == 0)
					{
						rxFifoSize = rxLastPacketSize;
						sci->SCIFFRX.bit.RXFFIL = rxLastPacketSize;
					}
					waitLastRxPacket = true;
				}
			}
			break;

			case IDLE:
				// data reset
				rxIndex = rxIndexTail = 0;
			break;

			default:
			break;
		}
	}
	else
    {
        if (rxTotalPacketsNumber == 0)
        {
            time = 0;
            rxFifoSize = FRAME_MIN_SIZE;
            sci->SCIFFRX.bit.RXFFIL = rxFifoSize;   // set interrupt condition

            uint16_t index = rxIndexHead;
            uint16_t size = 0xFF & rxBuf[index];

            uint16_t crc1 = ComputeCRC16(size + 1, &rxBuf[index]);
            uint16_t crc2 = (rxBuf[index + size + 1] << 8) | (rxBuf[index + size + 2] & 0xFF);

            if (crc1 == crc2)
            {
                for (uint16_t i=0; i < size+1; i++)
                {
                    rxFrame[i] = rxBuf[rxIndexHead++];
                    rxIndexHead &= 128-1;
                }

                rxState = GOOD_FRAME_RECEIVED;
                isReceived = true;
            }
            else
            {
                sci->SCIFFRX.bit.RXFIFORESET = 0;       //Write 0 to reset the FIFO pointer to zero, and hold in reset.
                StartReceive();
            }

            waitLastRxPacket = false;
        }
        else
        {
            time = timeOut;

            rxTotalPacketsNumber--;

            // update the last packet size
            if (rxTotalPacketsNumber == 0)
                rxFifoSize = rxLastPacketSize;

            sci->SCIFFRX.bit.RXFFIL = rxFifoSize;
        }
    }
}


void InitSciGpio()
{
   InitSciaGpio();
#if DSP28_SCIB
   InitScibGpio();
#endif // if DSP28_SCIB
#if DSP28_SCIC
   InitScicGpio();
#endif // if DSP28_SCIC
}

void InitSciaGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;	   // Enable pull-up for GPIO29 (SCITXDA)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)

/* Configure SCI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation

    EDIS;
}

void InitScibGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;     // Enable pull-up for GPIO9  (SCITXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up for GPIO14 (SCITXDB)
	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;	   // Enable pull-up for GPIO18 (SCITXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;    // Enable pull-up for GPIO22 (SCITXDB)


//  GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up for GPIO11 (SCIRXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up for GPIO15 (SCIRXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	   // Enable pull-up for GPIO19 (SCIRXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up for GPIO23 (SCIRXDB)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3;  // Asynch input GPIO11 (SCIRXDB)
//  GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (SCIRXDB)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDB)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)

/* Configure SCI-B pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2;    // Configure GPIO9 for SCITXDB operation
//  GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;   // Configure GPIO14 for SCITXDB operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDB operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;   // Configure GPIO22 for SCITXDB operation

//  GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2;   // Configure GPIO11 for SCIRXDB operation
//  GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2;   // Configure GPIO15 for SCIRXDB operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDB operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO23 for SCIRXDB operation

    EDIS;
}

void InitScicGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

/* Configure SCI-C pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation

    EDIS;
}


