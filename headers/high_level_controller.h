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
    float error_position, error_orientation;

    bool isTriggered;
    bool isRunning; //w trakcie regulacji

    float wr_max, wl_max;
    float ex, ey, v, w, k, d, D,r; //do regulacji polozenia

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
        error_position = 0.01;
        error_orientation = 0.02;
        Mode = POSITION;
        Setting = TEST;

        //k=0.2; d=0.1; D=0.145, r =0.025;
        k=0.2; d=0.1;

        wr_max = 2;
        wl_max = 2;
	}


    void SetVelocities(float wr, float wl)
    {
        if(abs_float(wr)<25.0 && abs_float(wl)<25.0)
        {
        wheelsVel.rightWheel = wr;
        wheelsVel.leftWheel = wl;
        isTriggered = true;
        }
        else
        {
            wheelsVel.rightWheel = 25.0;
            wheelsVel.leftWheel = 25.0;
        }
    }

    void SetMode(unsigned int modeChoice)
    {
        switch(modeChoice)
        {
            case 0:
            {
                Mode = POSITION;
                Setting = TEST;
            }
            break;
            case 1:
            {
                Mode = ORIENTATION;
                Setting = TEST;
            }
            break;
            case 2:
            {
                Mode = POSITION; //tymczasowo
                Setting = PID;
            }
            break;
            case 3:
            {
                Mode = POSITION; //tymczasowo
                Setting = PI;
            }
            break;
        }
    }

    void Stop()
    {
        wheelsVel.leftWheel = 0;
        wheelsVel.rightWheel = 0;
        isTriggered = true;
    }

    bool Update()
    {
        if(Mode == ORIENTATION && Setting == TEST)
        {
            if(isRunning)
            {

                //this->SetVelocities(1,1);
                if(abs_float(odometry.posture.th - targetPos.th) < error_orientation)
                {
                    isRunning = false;
                    this->Stop();
                } else {
                   //this->SetVelocities((wr_max/(2*M_PI))*(targetPos.th - odometry.posture.th), (wl_max/(2*M_PI))*(targetPos.th - odometry.posture.th));
                   this->SetVelocities(-wr_max*sign(odometry.posture.th - targetPos.th),-wl_max*sign(odometry.posture.th - targetPos.th));
                }
            }
        }
        if(Mode == POSITION && Setting == TEST)
        {
            if(isRunning)
            {
                if(abs_float(odometry.posture.x + d*cos(odometry.posture.th) - targetPos.x) < error_position && abs_float(odometry.posture.y + d*sin(odometry.posture.th) - targetPos.y) < error_position)
                {
                    isRunning = false;
                    this->Stop();
                }
                else {
                                //regulacja polozenia

                                ex = odometry.posture.x + d*cos(odometry.posture.th) - targetPos.x;
                                ey = odometry.posture.y + d*sin(odometry.posture.th) - targetPos.y;
                                v = -k*(cos(odometry.posture.th)*ex + sin(odometry.posture.th)*ey);
                                w = -k*(-sin(odometry.posture.th)*ex/d + cos(odometry.posture.th)*ey/d);

                                this->SetVelocities((v+0.5*WHEEL_BASE*w)/WHEEL_RADIUS, -(v-0.5*WHEEL_BASE*w)/WHEEL_RADIUS);   //pamiêtaj o minusie przy lewym kole

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

