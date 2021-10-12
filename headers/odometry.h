#pragma once

#include "math_routines.h"
#include "hardware_interface.h"
#include "global_parameters.h"
#include "robot.h"

class Odometry
{
	volatile int32_t wheelPosL, wheelPosR;
	int32_t wheelPosDeltaL, wheelPosDeltaR;
	volatile int32_t xRaw, yRaw, thRaw;

	static const float IMP2RAD;
	static const float ANG_IMP_MAX;
	static const float IMP2METER;


public:
	bool isReady;
	Posture2D posture;

	Odometry()
	{
		posture.x = 0.0;
		posture.y = 0.0;
		posture.th = 0.0;

		isReady = false;
		xRaw = yRaw = thRaw = 0;
	}

	void Init()
	{
	}


	void ComputeIncrements(int32_t pr, int32_t pl)
	{
		wheelPosDeltaL = pl - wheelPosL;
		wheelPosDeltaR = pr - wheelPosR;

		wheelPosL = pl;
		wheelPosR = pr;

		isReady = true;
	}

	void Set(Posture2D &pos)
	{
		posture.x = pos.x;
		posture.y = pos.y;
		posture.th = pos.th;

		xRaw = (int32_t) (256.0/IMP2METER * pos.x);
		yRaw = (int32_t) (256.0/IMP2METER * pos.y);
		thRaw = (int32_t) (1.0/IMP2RAD * pos.th);
	}


	void Compute(void)
	{
		DINT;
		// sum (sm) and difference (sp) of increments
		int16_t sm = wheelPosDeltaL + wheelPosDeltaR;
		int16_t sp = wheelPosDeltaL - wheelPosDeltaR;
#ifdef CAR_VEHICLE
		sm = - sm;
		sp = - sp;
#endif

		EINT;

		if (sm |sp)
		{
			float dth = IMP2RAD * sm;
			float th2 = posture.th + dth/2.0;
			// Increase precision
			sp <<= 8;

			// position increments
			int32_t dx = (int32_t) (cos(th2) * sp);
			int32_t dy = (int32_t) (sin(th2) * sp);

			xRaw += dx;
			yRaw += dy;
			thRaw += sm;

			// correction of the orientation
			if (thRaw > ANG_IMP_MAX)
				thRaw -= 2.0*ANG_IMP_MAX;
			else if (thRaw < -ANG_IMP_MAX)
				thRaw += 2.0*ANG_IMP_MAX;

			// conversion to float
			DINT;
			posture.th = IMP2RAD * thRaw;
			posture.x = IMP2METER/256.0 * xRaw;
			posture.y = IMP2METER/256.0 * yRaw;
			EINT;

			isReady = false;
		}
	}

};





