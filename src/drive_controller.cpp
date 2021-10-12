#include "drive_controller.h"


void Drive::Update()
{

#ifdef DIFF_VEHICLE
    wL = filterVelL.Compute(wheelL.vel);
	float phiL = wheelL.pos;
	wR = filterVelR.Compute(wheelR.vel);
	float phiR = wheelR.pos;

	if (regEnable)
	{
		regL.Compute(phiL, wL, refWl);
		regR.Compute(phiR, wR, refWr);
	}
	else
	{
		regR.u = 0;
		regL.u = 0;
	}
	power.SetOutput(regR.u, regL.u);
#endif

#ifdef CAR_VEHICLE
	wRear =  0.5*(-filterVelL.Compute(wheelL.vel) + filterVelR.Compute(wheelR.vel));
	int32_t phi = (-wheelL.pos + wheelR.pos); // why no 0.5 factor?

    if (regEnable)

        regL.Compute(phi, wRear, refWrear);
    else
        regL.u = 0;

    power.SetOutput(regL.u);
    //power.SetOutput(0.5);

#endif
}


