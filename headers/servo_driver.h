#pragma once

#include "hardware_interface.h"

class ServoDriver
{
    uint16_t updatePeriod;
    uint16_t updateTick;

    static void StartPulse()
    {
        CpuTimer0.RegsAddr->TCR.bit.TRB = 1;
        GpioDataRegs.GPASET.bit.GPIO30  = 1;
        CpuTimer0.RegsAddr->TCR.bit.TSS = 0;
    }

    void StopPulse()
    {
        GpioDataRegs.GPACLEAR.bit.GPIO30  = 1;
    }

    void TimerInitialise()
    {
        // Initialize timer period:
        CpuTimer0.RegsAddr->PRD.all = 0;
        CpuTimer0.RegsAddr->TCR.bit.TSS = 1;      // 1 = Stop timer, 0 = Start/Restart Timer
        CpuTimer0.RegsAddr->TPR.all  = 0;
        CpuTimer0.RegsAddr->TPRH.all  = 0;
        CpuTimer0.RegsAddr->TCR.bit.TRB = 1;      // 1 = reload timer
        CpuTimer0.RegsAddr->TCR.bit.SOFT = 1;
        CpuTimer0.RegsAddr->TCR.bit.FREE = 1;     // Timer Free Run
        CpuTimer0.RegsAddr->TCR.bit.TIE = 1;      // 0 = Disable/ 1 = Enable Timer Interrupt

        CpuTimer0.InterruptCount = 0;
    }

public:

    float setValue;

    ServoDriver(uint16_t updatePeriod)
    {
        updateTick = 0;
        setValue = 0;
        this->updatePeriod = updatePeriod;
    }

	void Init()
	{
		EALLOW;

		GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;   // Enable pullup
		GpioDataRegs.GPASET.bit.GPIO30 = 1;   // Load output latch
		GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;  // GPIO mode
		GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;   // Output

		TimerInitialise();

		EDIS;
	}

    void Update()
    {
        if (!(--updatePeriod))
        {
           uint32_t counter = (uint32_t) ((150*1500.0f) + (150*500.0f) * setValue);
           CpuTimer0.RegsAddr->PRD.all = counter;
           StartPulse();
           updatePeriod = 20;
        }
    }

	void Update(float input)
	{
	    if (!(--updatePeriod))
	    {
	       uint32_t counter = (uint32_t) ((150*1500.0f) + (150*500.0f) * input);
	       CpuTimer0.RegsAddr->PRD.all = counter;
	       StartPulse();
	       updatePeriod = 20;
	    }
	}

	void Irq()
	{
	    CpuTimer0.RegsAddr->TCR.bit.TSS = 1;
	    StopPulse();
	}
};
