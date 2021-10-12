#include "app.h"

interrupt void IrqReceiveSciC(void)
{
	serialUsbPort.ReceiveIrq();

	ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	PieCtrlRegs.PIEACK.all |= PIEACK_GROUP8;

}

interrupt void IrqTransmitSciC(void)
{
	serialUsbPort.SendIrq();
	ScicRegs.SCIFFTX.bit.TXFFINTCLR=1;
	PieCtrlRegs.PIEACK.all |= PIEACK_GROUP8;
}

interrupt void IrqReceiveSciA(void)
{
    serialAuxPort.ReceiveIrq();

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;

}

interrupt void IrqTransmitSciA(void)
{
    serialAuxPort.SendIrq();
    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;
}

interrupt void IrqCpuTimer0(void)
{
    servo.Irq();
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

interrupt void IrqMainClock(void)                  // EPWM1 Interrupts once every 4 QCLK counts (one period)
{
	Application::PeriodUpdate();

	EQep1Regs.QCLR.bit.UTO = 1;
	EQep1Regs.QCLR.bit.INT = 1;

	PieCtrlRegs.PIEACK.all |= PIEACK_GROUP5;
}

interrupt void IrqSpiRx(void)
{
    radio.SpiRxIrq();
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP6;
}

interrupt void IrqRadioReceived(void)
{
    radio.ReceivedPacketIrq();
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP12;
}

interrupt void IrqRadioOverflow(void)
{
    radio.ReceiverOverflowIrq();
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP12;
}


void SystemSetIntVectors()
{
	EALLOW;
	PieVectTable.SCIRXINTC = (PINT) &IrqReceiveSciC;
	PieVectTable.SCITXINTC = (PINT) &IrqTransmitSciC;
	PieVectTable.SCIRXINTA = (PINT) &IrqReceiveSciA;
	PieVectTable.SCITXINTA = (PINT) &IrqTransmitSciA;
	PieVectTable.EQEP1_INT= (PINT) &IrqMainClock;
	PieVectTable.TINT0 = (PINT) &IrqCpuTimer0;



	PieVectTable.XINT7 = (PINT) &IrqRadioReceived;     //od radia GDO2 = GPIO59
	PieVectTable.XINT6 = (PINT) &IrqRadioOverflow;     //od radia GDO0 = GPIO61
    PieVectTable.SPIRXINTA = (PINT) &IrqSpiRx;
	EDIS;
}

void SystemSetInterrupts()
{
   	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;


   	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;   // enabling PIEIER1 - INT1, bit7. = Timer0 interrupt
   	IER |= M_INT1;

    PieCtrlRegs.PIEIER5.bit.INTx1=1;     // Enable PIE Group 5, INT 1
    IER |= M_INT5;

   	PieCtrlRegs.PIEIER5.bit.INTx1=1;     // Enable PIE Group 5, INT 1 (EQEP1_INT)
   	IER |= M_INT5;

	PieCtrlRegs.PIEIER8.bit.INTx5=1;     // Enable PIE Group 8, INT5 -  SCI_RX_INTC_ISR
	PieCtrlRegs.PIEIER8.bit.INTx6=1;     // Enable PIE Group 8, INT6 -  SCI_TX_INTC_ISR

	IER |= M_INT8;

	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1.
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;   // PIE Group 9, INT2.

    IER |= M_INT9;                       // SCITXINTA and SCIRXINTA.


    PieCtrlRegs.PIEIER6.bit.INTx1=1;     // Enable PIE Group 6, INT 1 (spiRxFifoIsr)
//  PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2 (spiTxFifoIsr)
    IER |= M_INT6;

    PieCtrlRegs.PIEIER12.bit.INTx5 = 1;   //Enable PIE Gropu 12 INT5 (Ext.Int.7 - od radia)
    PieCtrlRegs.PIEIER12.bit.INTx4 = 1;   //Enable PIE Gropu 12 INT4 (Ext.Int.6 - od radia)

    IER |= M_INT12;

    PieCtrlRegs.PIEIER5.bit.INTx1=1;     // Enable PIE Group 5, INT 1 (EQEP1_INT)
    IER |= M_INT5;

}


