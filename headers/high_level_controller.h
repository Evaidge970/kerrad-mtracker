#pragma once

#include <stdlib.h>
#include "messages.h"


class HighLevelController
{

public:

    WheelsVelocities wheelsVel;

    bool isTriggered;

    enum Mode {ZERO, VELOCITY, POSITION};

    HighLevelController()
	{
        wheelsVel.leftWheel = wheelsVel.rightWheel = 0;
        isTriggered = true;
	}


    void SetVelocities(float wr, float wl)
    {
        wheelsVel.rightWheel = wr;
        wheelsVel.leftWheel = wl;
        isTriggered = true;
    }

    void Stop()
    {
        wheelsVel.leftWheel = wheelsVel.rightWheel = 0;
        isTriggered = true;
    }

    bool Update()
    {
        if (isTriggered)
        {

            isTriggered = false;
            return true;
        }
        return false;
    }


};

