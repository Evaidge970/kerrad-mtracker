#pragma once


#include "hardware_interface.h"
#include "math_routines.h"
#include "filter.h"
#include "leds.h"
#include <stdlib.h>
#include "wheel_measurements.h"
#include "power_bridge_driver.h"
#include "pid_drive_controller.h"
#include "messages.h"


class Drive
{
    float refWr, refWl;
public:

    float refWrear;

    float wR, wL;
    float wRear;

	volatile uint16_t   regEnable:1;
	volatile uint16_t   motorEnable:1;

	PidDriveController  regL;
	PidDriveController  regR;

	WheelMeasurements	wheelL;
	WheelMeasurements	wheelR;

	LP1Filter 		filterVelL;
	LP1Filter 		filterVelR;

	PowerBridgeDriver   power;

	Drive(): filterVelL(50.0, APP_MAIN_PERIOD), filterVelR(50.0, APP_MAIN_PERIOD), wheelR(&EQep1Regs, 'R'), wheelL(&EQep2Regs, 'L'), regL(APP_MAIN_CLK), regR(APP_MAIN_CLK)
	{
	}

	void SetVelocities(WheelsVelocities vel)
	{
	    refWr = vel.rightWheel;
	    refWl = vel.leftWheel;
	}

	void Init()
	{
		wheelL.HardwareInit();
		wheelR.HardwareInit();
#ifdef DIFF_VEHICLE
		power.SetMode(PowerBridgeDriver::DOUBLE_MODE_UNI_VOLT);
#endif

#ifdef CAR_VEHICLE
		power.SetMode(PowerBridgeDriver::SINGLE_MODE_BIP_VOLT);
#endif
		regEnable = 0;
		motorEnable = 1;
		wheelL.pos = wheelR.pos = 0;

		// bipolar mode: kp = 0.03, ki = 1.0
		// unipolar mode: kp = 0.1, ki = 5.0, kc1 = 6, kc2 = 0.02

#ifdef DIFF_VEHICLE
		// set default parameters
		regL.Kp = regR.Kp = 0.12;
		regL.Ki = regR.Ki = 2.0;
		regL.Kc1 = regR.Kc1 = 10.0;
		regL.Kc2 = regR.Kc2 = 0.02;
		regL.uc = regR.uc = 0;
#endif

#ifdef CAR_VEHICLE
		// set default parameters
		regL.Kp = regR.Kp = 0.075;
		regL.Ki = regR.Ki = 2.0;
		regL.Kc1 = regR.Kc1 = 10.0;
		regL.Kc2 = regR.Kc2 = 0.02;
		regL.uc = regR.uc = 0;
#endif

	}

	void Update();

	void ComputePosVel()
	{
	    wheelL.ComputePosVel();
	    wheelR.ComputePosVel();
	}

	void Stop()
	{
	    refWr = refWl = refWrear = 0;
	}

	void DisableOutput(){}
	void EnableOutput(){}

	void IntegralReset(int32_t posL, int32_t posR)
	{
#ifdef DIFF_VEHICLE
        regR.phir = posL;
        regL.phir = posR;
        regR.phirFrac = regL.phirFrac = 0;

#endif

#ifdef CAR_VEHICLE
        float phi = 0.5*(wheelR.pos + wheelL.pos);
        regL.phir = phi;
        regL.phirFrac = 0;
#endif

	}
};

