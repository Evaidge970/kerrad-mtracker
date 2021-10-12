#pragma once

#include "global_parameters.h"
#include "math_routines.h"
#include <stdlib.h>

class PidDriveController
{
    static const float   RAD2IMP;
    static const float   IMP2RAD;

public:

	PidDriveController(float clk)
	{
		Ts = 1/clk;
		phir = 0;
		phirFrac = 0;
	}


	float 	phirFrac;
	int32_t	phir;

	float	e_phi, e_w;

	float	u;
	float	uc;					// anty wind-up correction

	float 	Ts;

	// parameters
	float	Kp;
	float	Ki;
	float	Kc1;
	float	Kc2;


	void Compute(int32_t phi, float w, float wr)
	{

	    float wrImp = RAD2IMP * wr;
        phirFrac += wrImp * (1.0 - uc * Kc2 * sign(wrImp)) * Ts - uc * Kc1 * Ts;

	    // fractional part of integral
	    int32_t deltaPhir = (int32_t) phirFrac;
	    phirFrac -= deltaPhir;

	    // reference angular path
	    phir += deltaPhir;

	    // velocity error
        //volatile float
	    e_w = wr - w;

	    // angular error
	    e_phi = (phir - phi) * IMP2RAD;

	    //float kp = (1 + fabs(r->wr)/30.0) * r->Kp;    // gain schedulling

	    // PI control law
	    u = Kp * e_w + Ki * e_phi;

	    // saturate control output
	    if (fabs(u) > 1.0)
	    {
	        if (u > 0)
	        {
	            uc = u - 1.0;
	            u = 1.0;
	        }
	        else
	        {
	            uc = u + 1.0;
	            u = -1.0;
	        }
	    }
	    else
	        uc = 0;
	}
};

