#pragma once

#include "math_routines.h"

// Low pass I order filter
class LP1Filter
{
	float a1;
    float b0;
    float fc;
    float Ts;

public:

    float out;

    LP1Filter(float fc, float Ts)
	{
    	this->fc = fc;
    	this->Ts = Ts;
    	a1 = 1.0-2.0*M_PI*Ts*fc;
    	b0 = 2.0*M_PI*Ts*fc;
    	out = 0;
	}

    float Compute(float u)
    {
    	out = a1 * out + b0 * u;
    	return out;
    }
};
