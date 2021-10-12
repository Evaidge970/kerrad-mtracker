#pragma once

#include "hardware_interface.h"
#include "global_data.h"
#include "global_parameters.h"
#include "interrupts.h"
#include "commands_interpreter.h"
#include "i2c_communication.h"

class Application
{

	static const uint16_t ODOMETRY_UPDATE_PERIOD =	3;

	static uint16_t clock1;

	static uint32_t time;


	static volatile bool isMainClockEvent;
	static int16_t extControlGuardTime;

public:

	Application()
	{
		clock1 = 0;
		time = 0;
	}

	static void InitHardware()
	{
		InitSysCtrl();
		InitCpuTimers();
		InitEPwmGpio();
		InitSciGpio();
		InitEQepGpio();

		Led::InitHardware();

		InitZone7();

		DINT;
		IER = 0x0000;
		IFR = 0x0000;

		InitPieCtrl();
		InitPieVectTable();
		SystemSetIntVectors();

	#ifdef FLASH
		MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
		InitFlash();
	#endif

		SystemSetInterrupts();

        if (!radio.Init())
            Led::Blink(3, 500);

	   	serialUsbPort.Init(&ScicRegs, 921600); // UART <-> USB
	   	serialUsbPort.StartReceive();

    #ifdef AUX_UART_ENABLED
	   	serialAuxPort.Init(&SciaRegs, 115200); // UART
	   	serialAuxPort.StartReceive();
    #endif

		InitAdc();
		AdcConverter::SetSampler();
		InitEPwm();

        drive.Init();
		odometry.Init();
		servo.Init();

		time = 0;

		drive.motorEnable = false;
        drive.regEnable = false;

		isMainClockEvent = false;
	}

	static void PeriodUpdate()
	{

	    drive.ComputePosVel();
	    drive.Update();

		AdcConverter::StartConversion();

		isMainClockEvent = true;

		clock1++;

		if (clock1 == ODOMETRY_UPDATE_PERIOD)
		{
			odometry.ComputeIncrements(drive.wheelL.pos, drive.wheelR.pos);
			clock1 = 0;
		}

		// check if external control data are present
		if (--extControlGuardTime < 0)
		{
		    hlController.Stop();
			extControlGuardTime = EXT_CONTROL_TIMEOUT;
		}
	}

	static void Run();
};
