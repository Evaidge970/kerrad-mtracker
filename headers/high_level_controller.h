#pragma once

#include <stdlib.h>
#include "messages.h"
#include "odometry.h"
#include "math_routines.h"
#include "robot.h"
//#include "global_data.h"

extern Odometry odometry;

class HighLevelController
{

public:

    WheelsVelocities wheelsVel;
    Posture2D targetPos; //zawiera x, y i th
    float error;

    bool isTriggered;
    bool isRunning; //w trakcie regulacji

    float wr_max, wl_max;

    enum ModeEnum {ZERO, VELOCITY, POSITION, ORIENTATION}; //tryby HLC
    enum SettingEnum {TWO_STEP, P, PI, PID, TEST}; //typy regulatora
    ModeEnum Mode;
    SettingEnum Setting;

    HighLevelController()
	{
        wheelsVel.leftWheel = wheelsVel.rightWheel = 0;
        isTriggered = true;
        isRunning = false;
        //isRunning = true;
        error = 0.2;
        Mode = ORIENTATION;
        Setting = TEST;

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
        if(Mode == ORIENTATION && Setting == TEST)
        {
            if(isRunning)
            {
                //this->SetVelocities(1,1);
                if(abs(odometry.posture.th - targetPos.th) < error)
                {
                    isRunning = false;
                    this->Stop();
                } else {
                   // this->SetVelocities(-(wr_max/(2*M_PI))*(targetPos.th - odometry.posture.th), -(wl_max/(2*M_PI))*(targetPos.th - odometry.posture.th));
                    this->SetVelocities(-wr_max,-wl_max);
                }
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

