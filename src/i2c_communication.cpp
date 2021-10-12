#include "i2c_communication.h"

static volatile unsigned char *I2CMaster_ReceiveField;
static volatile unsigned char *I2CMaster_TransmitField;

void I2cCommunicator::HardwareInit()
{
    EALLOW;
    I2caRegs.I2CMDR.bit.IRS = 0;
    I2caRegs.I2CSAR = 0x002C;
    I2caRegs.I2COAR = 0x002D;
    I2caRegs.I2CPSC.all = 0x12;
    I2caRegs.I2CCLKL = 5;
    I2caRegs.I2CCLKH = 5;
    I2caRegs.I2CIER.all = 0x2C;
    I2caRegs.I2CMDR.bit.IRS = 1;
    //I2caRegs.I2CFFTX.all = 0x6000;
    EDIS;
}

void I2cCommunicator::TransmitData(Uint16 TxCount, unsigned char *txarray, Uint16 RxCount, unsigned char *rxarray, Uint16 slaveAddress)
{
    I2caRegs.I2CSAR = slaveAddress;
	if (TxCount != 0)
	{
	    I2caRegs.I2CCNT = TxCount;
	    I2CMaster_TransmitField = txarray;
	    while (TxCount > 0)
	    {
	        I2caRegs.I2CDXR = *I2CMaster_TransmitField;
	        I2CMaster_TransmitField++;
	        TxCount--;
	    }
        I2caRegs.I2CMDR.all = 0x2E20; // Master TX Mode
	    I2C_Wait();
    }

    if (RxCount != 0)
    {
        I2CMaster_ReceiveField = rxarray;
        I2caRegs.I2CCNT = RxCount;
        I2caRegs.I2CMDR.all = 0x2C20; // Master RX Mode
        I2C_Wait();
    }
}

void I2cCommunicator::I2C_Wait(void)
{
    DELAY_US(12);
    while (I2caRegs.I2CMDR.bit.STP == 1);
}

void I2cCommunicator::I2cHandler()
{
    Uint16 I2CMaster_IntSource;
    I2CMaster_IntSource = I2caRegs.I2CISRC.bit.INTCODE;

    switch(I2CMaster_IntSource)
    {
        case I2C_NO_ISRC:   // =0
            break;

        case I2C_ARB_ISRC:  // =1
            break;

        case I2C_NACK_ISRC: // =2
            break;

        case I2C_ARDY_ISRC: // =3
            break;

        case I2C_RX_ISRC:   // =4
            *I2CMaster_ReceiveField = I2caRegs.I2CDRR;
            I2CMaster_ReceiveField++;
            break;

        case I2C_TX_ISRC:   // =5
            break;

        case I2C_SCD_ISRC:  // =6
            break;

        case I2C_AAS_ISRC:  // =7
            break;

        default:
            asm(" ESTOP0"); // Halt on invalid number.
    }
}
