#pragma once

#include <stdlib.h>
#include "messages.h"
#include "odometry.h"
#include "math_routines.h"
#include "robot.h"

class HighLevelController
{

public:

    WheelsVelocities wheelsVel;
    Posture2D targetPos; //zawiera x, y i th
    float error;

    bool isTriggered;
    bool isRunning; //w trakcie regulacji

    float wr_max, wl_max;

    enum Mode {ZERO, VELOCITY, POSITION};

    HighLevelController()
	{
        wheelsVel.leftWheel = wheelsVel.rightWheel = 0;
        isTriggered = true;
        isRunning = false;
        error = 0.1;

        wr_max = 1;
        wl_max = 1;
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
        //regulacja rotacji
        if(isRunning)
        {
            if(abs(odometry.posture.th - targetPos.th) < error)
            {
                isRunning = false;
                this->Stop();
            } else {
                this->SetVelocities(_IQ8toF(-(wr_max/2*M_PI)*(odometry.posture.th - targetPos.th))), _IQ8toF((wl_max/2*M_PI)*(odometry.posture.th - targetPos.th))));
            }
        }

        if (isTriggered) //this will enable updating drive controller velocities
        {

            isTriggered = false;
            return true;
        }
        return false;
    }


};

