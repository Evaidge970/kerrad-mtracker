#pragma once

#include "stdint.h"
#include "hardware_interface.h"
#include "math_routines.h"
#include "global_parameters.h"

extern float BufTemp0[];

class WheelMeasurements
{
	static const float EQEP_FREQ_CAP;
	static const float SPEED_CONST;

	volatile EQEP_REGS * eqep;

public:
	static const float RAD2IMP;
	static const float IMP2RAD;

	volatile int32_t    pos;        // wheel pos [imp]
    volatile float      vel;        // wheel velocity [rad/s]
    bool                negativeVelocity;       // sign of the current velocity

    uint16_t indexBuf;
    volatile float timeExact;
    volatile float timeRough;

    char name;


	WheelMeasurements(volatile EQEP_REGS * eqep, char name)
	{
		this->eqep = eqep;
		this->name = name;

		indexBuf = 0;
		pos = 0;
		vel = 0;
		timeExact = 0;
		timeRough = 0;
		negativeVelocity = false;
	}


	void HardwareInit()
	{
		eqep->QPOSCNT = 0;
		eqep->QUPRD = CPU_FREQ/APP_MAIN_CLK;	// Unit Timer for 100Hz at 150 MHz SYSCLKOUT
		eqep->QDECCTL.bit.QSRC = 0;		// QEP quadrature count mode

		eqep->QEPCTL.bit.FREE_SOFT = 2;	// Position counter is unaffected by emulation suspend
		eqep->QEPCTL.bit.PCRM = 0;		// PCRM=00 mode - QPOSCNT reset on index event
		eqep->QEPCTL.bit.UTE = 1; 		// Unit Timeout Enable
		eqep->QEPCTL.bit.QCLM = 1; 		// Latch on unit time out
		eqep->QPOSMAX = 0xffffffff;
		eqep->QEPCTL.bit.QPEN = 1; 		// QEP enable

		eqep->QCAPCTL.bit.UPPS = 2;   	// 1/4 for unit position
		eqep->QCAPCTL.bit.CCPS = 7;		// 1/128 for CAP clock
		eqep->QCAPCTL.bit.CEN = 1; 		// QEP Capture Enable
		eqep->QEINT.bit.UTO = 1;		// enable Unit Timer IRQ
	}

	void ComputePosVel()
	{
	    float v = vel;

	    // position (impulses)
        pos = eqep->QPOSCNT;
        bool overflow = eqep->QEPSTS.bit.COEF;

        timeRough += APP_MAIN_PERIOD;

        if (overflow)
        {
            eqep->QEPSTS.bit.COEF = 1;  // clear flag
            timeExact += (1.0/EQEP_FREQ_CAP) * 0x10000;
        }

		// compute velocity based on period measurement (between impulses)
		if(eqep->QEPSTS.bit.UPEVNT==1)  // new impulse detected
		{
            // direction
            negativeVelocity = (eqep->QEPSTS.bit.QDF==0);

			// check if the time is computed properly
            float t = (1.0/EQEP_FREQ_CAP)*(float)eqep->QCPRDLAT + timeExact;
            v = SPEED_CONST / t;

            // start a new period
			timeExact = 0;
			timeRough = 0;

            if (negativeVelocity)
                vel = -v;
            else
                vel = v;

            eqep->QEPSTS.bit.UPEVNT = 1;    // clear flag
		}
		else
		{
		    v = SPEED_CONST/timeRough;
	        if (fabs(v) < fabs(vel))
	            if (negativeVelocity)
	                vel = -v;
	            else
	                vel = v;
		}
	}
};
