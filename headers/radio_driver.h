#pragma once

#include "hardware_interface.h"
#include <stdint.h>
#include "CC1100-CC2500.h"

class RadioDriver
{
    uint16_t txBufIndex;
    uint16_t rxBufIndex;

    uint16_t lastPacketSize;
    uint16_t currentPacketSize;
    uint16_t packetsNumber;

    static const uint16_t SpiFIFOSize = 16;

    volatile uint16_t chipStatus;

    bool isQueuePurified;

    enum spiIrqState {WRITE_PACKETS, READ_FIRST_PACKET, READ_SUCCESIVE_PACKETS, QUEUE_PURIFICATION_A, QUEUE_PURIFICATION_B, QUEUE_PURIFICATION_C};

    volatile uint16_t spiIrqState;

    void GpioInit();
    void SpiInit(void);
    void PowerupReset(void);
    void ChipInit(void);

    static const uint16_t paTable[1];
    static const uint16_t paTableLen = 1;

    void WriteChipSettings(void);
    void WriteReg(uint16_t addr, uint16_t value);
    uint16_t ReadReg(uint16_t addr);
    uint16_t ReadStatus(uint16_t addr);
    void Strobe(uint16_t strobe);

    void BurstWriteZerosToSpi(uint16_t size)
    {
        for(uint16_t i = 0; i < size; i++)
            SpiaRegs.SPITXBUF = 0;
    }

    void BurstDummyReadFromSpi(uint16_t size)
    {
        for(uint16_t i = 0; i < size; i++)
            volatile uint16_t c = SpiaRegs.SPIRXBUF;
    }

    void TriggerBurstRead(uint16_t addr, uint16_t count);
    void ReadBurstFirstPacket(void);
    void ReadBurstSuccesivePackets(void);
    void TriggerBurstWrite();
    void BurstWriteSuccesivePackets();

    void ClearQueueStage1();
    void ClearQueueStage2();
    void ClearQueueStage3();

    void Wait(volatile uint32_t time)
    {
        DELAY_US(time);
    }

    void EnableRadioSignalsInterrupts()
    {
        // Ext.Int.7 - frame received
        PieCtrlRegs.PIEIER12.bit.INTx5 = 1;
        // Ext.Int.6 - overflow
        PieCtrlRegs.PIEIER12.bit.INTx4 = 1;
    }

    void DisableRadioSignalsInterrupts()
    {
        // Ext.Int.7 - frame received
        PieCtrlRegs.PIEIER12.bit.INTx5 = 0;
        // Ext.Int.6 - overflow
        PieCtrlRegs.PIEIER12.bit.INTx4 = 0;
    }

    void DisableCS()
    {
        GpioDataRegs.GPBSET.bit.GPIO57 = 1;      // /CS=1
    }

    void EnableCS()
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;    // /CS=0
    }

    void UpdateFrameCounters(uint16_t size)
    {
        packetsNumber = size / SpiFIFOSize;
        currentPacketSize = size % SpiFIFOSize;

        if ((packetsNumber > 0) && (currentPacketSize == 0))
        {
            packetsNumber--;
            currentPacketSize = SpiFIFOSize;
        }
    }

public:
    uint16_t txBuf[64];
    uint16_t rxBuf[256];

    bool isRadioInitialized;
    bool isTransmissionCompleted;
    bool isReceived;
    bool isError;

    bool Init()
    {
        GpioInit();
        SpiInit();
        ChipInit();
        return isRadioInitialized;
    }

    void SpiRxIrq()
    {

        switch (spiIrqState)
        {
            case WRITE_PACKETS:
                BurstWriteSuccesivePackets();
            break;

            case READ_FIRST_PACKET:
                ReadBurstFirstPacket();
            break;

            case READ_SUCCESIVE_PACKETS:
                ReadBurstSuccesivePackets();
            break;

            case QUEUE_PURIFICATION_A:
                ClearQueueStage1();
            break;

            case QUEUE_PURIFICATION_B:
                ClearQueueStage2();
            break;

            case QUEUE_PURIFICATION_C:
                ClearQueueStage3();
            break;

            default:
            break;
        }

        SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;            // Clear Interrupt flag - przerwanie wymaga programowego czyszczenia
    }

    void ReceivedPacketIrq()
    {
        // xint7_isr
        // Radio device GDO2 - received a good frame

        DisableRadioSignalsInterrupts();

        TriggerBurstRead(TI_CCxxx0_RXFIFO, 4);
    }

    void ReceiverOverflowIrq()
    {
        // xint6_isr
        // Radio device GDO0 - overflow is detected

        DisableRadioSignalsInterrupts();

        isError = true;

        EnableCS();
        isQueuePurified = false;
        SpiaRegs.SPIFFRX.bit.RXFFIL = 2;
        SpiaRegs.SPITXBUF = (uint16_t)(TI_CCxxx0_RXBYTES | TI_CCxxx0_READ_BURST) << 8;
        SpiaRegs.SPITXBUF = 0 << 8;

        spiIrqState = QUEUE_PURIFICATION_A;
        SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;      //RX FIFO interrupt enabled
    }

    void TriggerSend()
    {
        isReceived = false;
        DisableRadioSignalsInterrupts();
        TriggerBurstWrite();
    }

};


