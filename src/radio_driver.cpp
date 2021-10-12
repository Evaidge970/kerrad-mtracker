#include <radio_driver.h>
#include "crc16.h"

const uint16_t RadioDriver::paTable[1] = {0x00FF};   //{0xFB};


void RadioDriver::GpioInit()
{
    EALLOW;

    // SPI lines

    // Pull-up lines
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;

    // Asynch inputs
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 3;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 3;

    // Configure multiplexer
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1;    // SPISIMOA
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1;    // SPISOMIA
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1;    // SPICLKA
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;    // GPIO - manual CS

    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;     // Output
    GpioDataRegs.GPBSET.bit.GPIO57 = 1;     // Load output latch

    // GPIO60(DSP) <-  GDO0(radio) INT_B
    // GPIO59(DSP) <-  GDO2(radio) INT_A

    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;     // Enable pull-up on GPIO60
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;    // GPIO mode
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 0;   //  Xint Synch to SYSCLKOUT only
    GpioIntRegs.GPIOXINT6SEL.bit.GPIOSEL = 60 - 32; // XINT6 = GDO0
    XIntruptRegs.XINT6CR.bit.POLARITY = 0;          // Event on falling edge

    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;   // Enable pull-up on GPIO55
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;  // GPIO59 = GPIO59
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 0; //  Xint? Synch to SYSCLKOUT only
    GpioIntRegs.GPIOXINT7SEL.bit.GPIOSEL = 59 - 32;// XINT7 = GDO2
    XIntruptRegs.XINT7CR.bit.POLARITY = 0;  // Event on falling edge

    EDIS;
}
void RadioDriver::PowerupReset(void)
{
    DisableCS();
    Wait(30);
    EnableCS();
    Wait(30);
    DisableCS();
    Wait(45);

    SpiaRegs.SPIFFRX.bit.RXFFIL = 1;    // flag is set after 1 char is received
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;  //RX FIFO interrupt disabled
    EnableCS();
    SpiaRegs.SPITXBUF = TI_CCxxx0_SRES << 8;      // Send strobe
    while(!(SpiaRegs.SPIFFRX.bit.RXFFINT)); // wait until transmission is completed
    chipStatus = SpiaRegs.SPIRXBUF;
    Wait(10);
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // clear interrupt flag
    DisableCS();
}

void RadioDriver::SpiInit(void)
{
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;   // Reset SPI

    SpiaRegs.SPICCR.all = 0x0007;         //8-bit character

    SpiaRegs.SPICTL.all = 0x0017;          //Interrupt enabled, Master/Slave XMIT enabled
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;   // for CC2500
    SpiaRegs.SPISTS.all = 0x0000;		    //SPISTS (SPI status register).
								   	    //Contains two receive buffer status bits and one transmit buffer status bit
									    //– RECEIVER OVERRUN
									    //– SPI INT FLAG
									    //– TX BUF FULL FLAG

    SpiaRegs.SPIBRR = 0x000B;           // Baud rate - nie chce dzia³aæ z podzielnikiem mniejszym niŸ B (a powinno) !!!!!
    SpiaRegs.SPIFFTX.all = 0xC028;      // Enable FIFO's, set TX FIFO level to 8
    SpiaRegs.SPIFFRX.all = 0x0028;      // Set RX FIFO level to 8

	
    SpiaRegs.SPIFFCT.all = 0x02;		// FIFO transmit delay bits
    SpiaRegs.SPIPRI.all = 0x0008;

    SpiaRegs.SPICCR.bit.SPISWRESET=1;   // Enable SPI

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
}

void RadioDriver::ChipInit(void)
{
    isQueuePurified = true;
	isTransmissionCompleted = false;
	isReceived = false;
	isError = false;
	isRadioInitialized = false;

	PowerupReset();
	Wait(10000);

	WriteChipSettings();
	Wait(10000);

	SpiaRegs.SPIBRR = 0x0006;           // Baud rate - zwiekszenie czêstotliwoœci SPI

	WriteReg(TI_CCxxx0_PATABLE, paTable[0]);//Write PATABLE - wersja zmodyfikowana

	Strobe(TI_CCxxx0_SRX);             // Initialize CCxxxx in RX mode.
                                                // When a pkt is received, it will
                                                // signal on GDO0 and wake CPU

	uint16_t numberOfTrials = 1000;
	uint16_t status;
	do
	{
	    numberOfTrials--;
	    status = ReadStatus(TI_CCxxx0_MARCSTATE);
	}
	while (((status != 13) || (chipStatus != 16)) && (numberOfTrials));

	if (numberOfTrials)
	    isRadioInitialized = true;

	// clear interrupt flags
    PieCtrlRegs.PIEIFR12.bit.INTx6 = 0;
    PieCtrlRegs.PIEIFR12.bit.INTx7 = 0;

    if (isRadioInitialized)
    {
        // connect lines to the interrupt controller
        XIntruptRegs.XINT6CR.bit.ENABLE = 1;
        XIntruptRegs.XINT7CR.bit.ENABLE = 1;
    }

}


void RadioDriver::TriggerBurstWrite()
{
	isTransmissionCompleted = false;

	EnableCS();

	txBufIndex = 0;

	// check if LSB is not zero
	uint16_t crc = CRC16Init();
	// read size
	uint16_t size = txBuf[0];

	for (uint16_t i = 0; i < size + 1; i++)
	    crc = CRC16Calc(crc, txBuf[txBufIndex++]);

	txBuf[txBufIndex++] = crc >> 8;
	txBuf[txBufIndex] = crc & 0x00FF;
	if (txBuf[txBufIndex] == 0)
	{
		// modify the corresponding char
	    // ??
	}

	txBufIndex = 0;

	size = txBuf[txBufIndex] + 3;       // compute size

	UpdateFrameCounters(size);

   	SpiaRegs.SPIFFRX.bit.RXFFIL = currentPacketSize;

   	// Go to transmit mode
   	SpiaRegs.SPITXBUF = TI_CCxxx0_STX << 8;
  	// Set register address
	SpiaRegs.SPITXBUF = (TI_CCxxx0_TXFIFO | TI_CCxxx0_WRITE_BURST) << 8;
	// and write user data
	for (uint16_t i = 0; i < currentPacketSize-2; i++)
		SpiaRegs.SPITXBUF = txBuf[txBufIndex++] << 8;

    spiIrqState = WRITE_PACKETS;
   	// Enable RX FIFO interrupt
	SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;
}

void RadioDriver::BurstWriteSuccesivePackets(void)
{		
    BurstDummyReadFromSpi(currentPacketSize);

    if (packetsNumber > 0)
    {
        packetsNumber--;
        currentPacketSize = SpiFIFOSize;

        // set FIFO size
	    SpiaRegs.SPIFFRX.bit.RXFFIL = currentPacketSize;

	    // Send data
		for(uint16_t i=0; i < currentPacketSize; i++)
			SpiaRegs.SPITXBUF = txBuf[txBufIndex++] << 8;
	}
    else
    {
        isTransmissionCompleted = true;

        DisableCS();
        EnableRadioSignalsInterrupts();
    }
}


void RadioDriver::TriggerBurstRead(uint16_t addr, uint16_t count)
{
  	EnableCS();

  	currentPacketSize = count;

	// current FIFO size
    SpiaRegs.SPIFFRX.bit.RXFFIL = currentPacketSize + 1;

    rxBufIndex = 0;

    SpiaRegs.SPITXBUF = (addr | TI_CCxxx0_READ_BURST) << 8;   // Send address
    BurstWriteZerosToSpi(currentPacketSize);

	spiIrqState = READ_FIRST_PACKET;

	SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;
}

void RadioDriver::ReadBurstFirstPacket(void)
{		
    chipStatus = SpiaRegs.SPIRXBUF;

	for(uint16_t i = 0; i < currentPacketSize; i++)
	    rxBuf[rxBufIndex++] = SpiaRegs.SPIRXBUF;

	uint16_t size = rxBuf[0] & 0x3F;

	if (size > 0)
	    size += 3 - currentPacketSize;	   // include size of the fist packet
	                                       // add chip_status, two crc chars

	if (size == 0)
	{
	    // A short frame is received
        isReceived = true;
        // RX FIFO interrupt disabled
        SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;

        DisableCS();
        EnableRadioSignalsInterrupts();
	}
	else
	{
	    UpdateFrameCounters(size);

	    spiIrqState = READ_SUCCESIVE_PACKETS;

	    // Write number of chars when the interrupt will be triggered
	    SpiaRegs.SPIFFRX.bit.RXFFIL = currentPacketSize;
	    // Push dummy bytes to read data from the device
	    BurstWriteZerosToSpi(currentPacketSize);
	}
}


void RadioDriver::ReadBurstSuccesivePackets(void)
{
    for(uint16_t i=0; i < currentPacketSize; i++)
        rxBuf[rxBufIndex++] = SpiaRegs.SPIRXBUF;

    if (packetsNumber > 0)
    {
        packetsNumber --;
        currentPacketSize = SpiFIFOSize;

        SpiaRegs.SPIFFRX.bit.RXFFIL = currentPacketSize;

        // Push dummy bytes to read data from the device
        BurstWriteZerosToSpi(currentPacketSize);
    }
    else
	{
   		isReceived = true;
		SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;  //RX FIFO interrupt disabled

		DisableCS();
		EnableRadioSignalsInterrupts();
	}
}

void RadioDriver::ClearQueueStage1()
{		
    chipStatus = SpiaRegs.SPIRXBUF;
    // Read how many data should be removed
    uint16_t size = SpiaRegs.SPIRXBUF & TI_CCxxx0_NUM_RXBYTES;
	// Add extra char (to control the device)
	size++;

    UpdateFrameCounters(size);

    // Set interrupt condition
    SpiaRegs.SPIFFRX.bit.RXFFIL = currentPacketSize;

    // Send address RXFIFO
    SpiaRegs.SPITXBUF = (uint16_t)(TI_CCxxx0_RXFIFO | TI_CCxxx0_READ_BURST) << 8;
    // Push dummy bytes to read data from the device
    BurstWriteZerosToSpi(currentPacketSize-1);

    spiIrqState = QUEUE_PURIFICATION_B;
}	

// odczyt kolejnych pakietów z FIFO - po ostatnim pakiecie sp³ukanie kolejki
void RadioDriver::ClearQueueStage2()
{
    BurstDummyReadFromSpi(currentPacketSize);

    if (packetsNumber > 0)
    {
        packetsNumber--;
        currentPacketSize = SpiFIFOSize;

        SpiaRegs.SPIFFRX.bit.RXFFIL = currentPacketSize;    // przerwanie od RX-FFIRO po okreœlonej iloœci ba

        BurstWriteZerosToSpi(currentPacketSize);
    }
    else
    {
		DisableCS();
		Wait(10);
		EnableCS();

		SpiaRegs.SPIFFRX.bit.RXFFIL = 2;
		SpiaRegs.SPITXBUF = (TI_CCxxx0_SFRX) << 8;      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
		SpiaRegs.SPITXBUF = (TI_CCxxx0_SRX) << 8;       // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.

		spiIrqState = QUEUE_PURIFICATION_C;
	}
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
}

void RadioDriver::ClearQueueStage3()
{
	volatile uint16_t c = SpiaRegs.SPIRXBUF;
	c = SpiaRegs.SPIRXBUF;

	isQueuePurified = true;
	isError = false;
	DisableCS();

	EnableRadioSignalsInterrupts();
}

void RadioDriver::WriteReg(uint16_t addr, uint16_t value)
{
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;  //RX FIFO interrupt disabled
    EnableCS();
    // RXFFINT should be set when 2 bytes are received
    SpiaRegs.SPIFFRX.bit.RXFFIL = 2;

    SpiaRegs.SPITXBUF = addr << 8;
    SpiaRegs.SPITXBUF = value << 8;

    // wait for data in FIFO
    while(!(SpiaRegs.SPIFFRX.bit.RXFFINT));

    chipStatus = SpiaRegs.SPIRXBUF;
    volatile uint16_t dump = SpiaRegs.SPIRXBUF;

    Wait(10);

    // clear Interrupt flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    DisableCS();
}

uint16_t RadioDriver::ReadReg(uint16_t addr)
{
    EnableCS();

    // RXFFINT should be set when 2 bytes are received
    SpiaRegs.SPIFFRX.bit.RXFFIL=2;
    //RX FIFO interrupt disabled
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;

    // Send address
    SpiaRegs.SPITXBUF = (addr | TI_CCxxx0_READ_SINGLE) << 8;
    // Dummy write so we can read data
    SpiaRegs.SPITXBUF = 0 << 8;
    // wait for data if FIFO
    while(!(SpiaRegs.SPIFFRX.bit.RXFFINT));
    chipStatus = SpiaRegs.SPIRXBUF;
    // Read data
    uint16_t data = SpiaRegs.SPIRXBUF;

    Wait(10);

    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;
    DisableCS();

    return data;
}

// For status/strobe addresses, the BURST bit selects between status registers
// and command strobes.
uint16_t RadioDriver::ReadStatus(uint16_t addr)
{
    EnableCS();
    SpiaRegs.SPIFFRX.bit.RXFFIL=2;      // flaga RXFFINT ustawi siê na 1 po odebraniu 2 bajtów
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;  //RX FIFO interrupt disabled
    SpiaRegs.SPITXBUF = (addr | TI_CCxxx0_READ_BURST) << 8;  // Send address
    SpiaRegs.SPITXBUF = 0 << 8;                              // Dummy write so we can read data
    while(!(SpiaRegs.SPIFFRX.bit.RXFFINT)); //oczekiwanie na zape³nieni bufora odbiornika

    chipStatus = SpiaRegs.SPIRXBUF; // odzczyt chip_status
    volatile uint16_t status = 0xFF & SpiaRegs.SPIRXBUF;            // Read data
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // Clear Interrupt flag
    DisableCS();
    Wait(10);
    return status;
}

void RadioDriver::Strobe(uint16_t strobe)
{
    EnableCS();
    SpiaRegs.SPIFFRX.bit.RXFFIL=1;          // flaga RXFFINT ustawi siê na 1 po odebraniu 1 bajtów
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;      //RX FIFO interrupt disabled
    SpiaRegs.SPITXBUF = strobe << 8;        // Send strobe
    while(!(SpiaRegs.SPIFFRX.bit.RXFFINT));
    chipStatus = SpiaRegs.SPIRXBUF;
    Wait(10);
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;      // Clear Interrupt flag
    DisableCS();
}


// Product = CC2500
// Crystal accuracy = 40 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 540.000000 kHz
// Deviation = 0.000000
// Return state:  Return to RX state upon leaving either TX or RX
// Datarate = 250.000000 kbps
// Modulation = (7) MSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 2433.000000 MHz
// Channel spacing = 199.950000 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (11) Serial Clock
void RadioDriver::WriteChipSettings(void)
{
    WriteReg(TI_CCxxx0_IOCFG2,   (0x07 | 0x40));    // Event - falling edge
    WriteReg(TI_CCxxx0_IOCFG0,   (0x04 | 0x40));    // Event - falling edge

    WriteReg(TI_CCxxx0_PKTLEN,   0xFF);  // Packet length.
    WriteReg(TI_CCxxx0_PKTCTRL1, 0x0D); //0x05);  // Packet automation control.
    WriteReg(TI_CCxxx0_PKTCTRL0, 0x45); //0x05);  // Packet automation control. Turn data whitening on

    WriteReg(TI_CCxxx0_ADDR,     0x01);  // Device address.
    WriteReg(TI_CCxxx0_CHANNR,   0x00); // Channel number.
    WriteReg(TI_CCxxx0_FSCTRL1,  0x12);
    WriteReg(TI_CCxxx0_FSCTRL0,  0x00); // Freq synthesizer control. OK
    WriteReg(TI_CCxxx0_FREQ2,    0x5D); // Freq control word, high byte OK
    WriteReg(TI_CCxxx0_FREQ1,    0x93); // Freq control word, mid byte. OK
    WriteReg(TI_CCxxx0_FREQ0,    0xB1); // Freq control word, low byte. OK
    WriteReg(TI_CCxxx0_MDMCFG4,  0x2D); // Modem configuration. OK
    WriteReg(TI_CCxxx0_MDMCFG3,  0x3B); // Modem configuration. OK
    WriteReg(TI_CCxxx0_MDMCFG2,  0x73); //0x73); // Modem configuration. OK
    WriteReg(TI_CCxxx0_MDMCFG1,  0x22); // Modem configuration. OK
    WriteReg(TI_CCxxx0_MDMCFG0,  0xF8); // Modem configuration. OK
    WriteReg(TI_CCxxx0_DEVIATN,  0x00); // Modem dev (when FSK mod en) OK
    WriteReg(TI_CCxxx0_MCSM1 ,   0x0F); //0x3F); //MainRadio Cntrl State Machine
    WriteReg(TI_CCxxx0_MCSM0 ,   0x18); //MainRadio Cntrl State Machine

    WriteReg(TI_CCxxx0_FOCCFG,   0x1D); // Freq Offset Compens. Config OK

    WriteReg(TI_CCxxx0_BSCFG,    0x1C); //  Bit synchronization config. OK
    WriteReg(TI_CCxxx0_AGCCTRL2, 0xC7); // AGC control. OK
    WriteReg(TI_CCxxx0_AGCCTRL1, 0x00); // AGC control. OK
    WriteReg(TI_CCxxx0_AGCCTRL0, 0xB2); // AGC control.

    WriteReg(TI_CCxxx0_FREND1,   0xB6); // Front end RX configuration.
    WriteReg(TI_CCxxx0_FREND0,   0x10); // Front end RX configuration.
    WriteReg(TI_CCxxx0_FSCAL3,   0xEA); // Frequency synthesizer cal.
    WriteReg(TI_CCxxx0_FSCAL2,   0x0A); // Frequency synthesizer cal.
    WriteReg(TI_CCxxx0_FSCAL1,   0x00); // Frequency synthesizer cal.
    WriteReg(TI_CCxxx0_FSCAL0,   0x11); // Frequency synthesizer cal.
    WriteReg(TI_CCxxx0_FSTEST,   0x59); // Frequency synthesizer cal.
    WriteReg(TI_CCxxx0_TEST2,    0x88); // Various test settings.
    WriteReg(TI_CCxxx0_TEST1,    0x31); // Various test settings.
    WriteReg(TI_CCxxx0_TEST0,    0x0B);  // Various test settings.
}


