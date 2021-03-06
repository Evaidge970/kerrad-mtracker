#pragma once

#include <stdlib.h>
#include "messages.h"
#include "odometry.h"
#include "math_routines.h"
#include "robot.h"

extern Odometry odometry;
extern Drive drive;


class HighLevelController
{

public:

    WheelsVelocities wheelsVel;
    Posture2D targetPos; //zawiera x, y i th
    float error_position, error_orientation;

    bool isTriggered;
    bool isRunning; //w trakcie regulacji

    float wr_max, wl_max; //globalne nasycenie predkosci
    float wr_max_orientation, wl_max_orientation; //tryb orientacji
    float SlowCoef, SlowThreshold; //wspolczynnik zwolnienia trybu slow, prog zakonczenia trybu slow
    float ex, ey, v, w, k, d, r, v_ax, v_ay, V_const, eps, ex0, ey0, w_x, w_y; //w - omega; w_x i w_y - elementy wektora w

    enum ModeEnum {ZERO, POSITION, ORIENTATION,NONE}; //tryby HLC
    enum SettingEnum {DEFAULT, CONST_VEL}; //ustawienia regulatora
    ModeEnum Mode;
    SettingEnum Setting;


    HighLevelController()
	{
        wheelsVel.leftWheel = wheelsVel.rightWheel = 0;
        isTriggered = true;
        isRunning = false;
        error_position = 0.01;
        error_orientation = 0.02;
        V_const = 0.1; //predkosc ruchu w trybie CONST_VEL
        eps = 0.5;
        ex0 = 0.0; ey0 = 0.0;
        k=0.2; d=0.1;
        wr_max = 15.0;
        wl_max = 15.0;
        wr_max_orientation = 2; //tryb orientacji
        wl_max_orientation = 2;
        SlowCoef = 0.5; //tryb slow
        SlowThreshold = 0.1;
	}


    void SetVelocities(float wr, float wl)
    {
          //!!!!!!!!!!!   dzielenie przez 2 w obu warunkach poni?ej zosta?o wprowadzone dla poprawej realizacji pr?dko?ci
          //!!!!!!!!!!!   i wykres?w. Konieczno?? wprowadzenia wynika z nieznanego b??du, kt?rego nale?y doszuka? si? w innnych plikach
                      //!!!!!!!!!!!   Prawdopodbnie nale?y usun?? wywo?anie drive.Update() w PeriodUpdate() (plik app.h), rozwi?zuje to problem dzielenia przez 2;
                      //              Nie rozwi?zuje to jednak problemu mno?enia pr?dko?ci wysy??nej (comands_interpreter.cpp), ponadto po takiej zmianie pojawiaj? si? szpilki w wykresach pr?dko?ci k??
        if(wr>wr_max) wheelsVel.rightWheel = wr_max/2.0;
        else if(wr<-wr_max) wheelsVel.rightWheel = -wr_max/2.0;
        else wheelsVel.rightWheel = wr/2.0;
        if(wl>wl_max) wheelsVel.leftWheel = wl_max/2.0;
        else if(wl<-wl_max) wheelsVel.leftWheel = -wl_max/2.0;
        else wheelsVel.leftWheel = wl/2.0;
        isTriggered = true;
    }
	
    void SetErrorConstVelMode() //powinno byc wywolane tylko raz na jeden rozkaz
    {
        ex0 = (odometry.posture.x + d*cos(odometry.posture.th)) - targetPos.x;
        ey0 = (odometry.posture.y + d*sin(odometry.posture.th)) - targetPos.y;
    }

    void SetParameters(unsigned int parChoice, float x, float y, float th)
    {
        switch(parChoice)
        {
            case 1:
            {
                wl_max_orientation = x;
                wr_max_orientation = y;
                error_orientation = th;
            }
            break;
            case 2:
            {
                k = x;
                d = y;
                error_position = th;
            }
            break;
            case 3:
            {
                V_const = x;
                eps = y;
                error_position = th;
            }
            break;
            case 4:
            {
                SlowCoef = x;
                SlowThreshold = y;
                error_position = th;
            }
            break;
            case 5:
            {
                wl_max = x;
                wr_max = y;
                error_position = th;
            }
            case 6:
            {
                error_position = 0.01;
                                error_orientation = 0.02;
                            V_const = 0.1; //predkosc ruchu w trybie CONST_VEL
                                eps = 0.5;
                            k=0.2;
                            d=0.1;
                                wr_max = 15.0;
                                wl_max = 15.0;
                                wr_max_orientation = 2; //tryb orientacji
                                wl_max_orientation = 2;
                                SlowCoef = 0.5; //tryb slow
                                SlowThreshold = 0.1;
            }

	    case 7:
		{
			error_position = 0.01;
        		error_orientation = 0.02;	
			V_const = 0.1; //predkosc ruchu w trybie CONST_VEL
        		eps = 0.5;
			k=0.2;
			d=0.1;
        		wr_max = 15.0;
        		wl_max = 15.0;
        		wr_max_orientation = 2; //tryb orientacji
        		wl_max_orientation = 2;
        		SlowCoef = 0.5; //tryb slow
        		SlowThreshold = 0.1;
		}
	    break;
            default:
            break;
        }
    }

    void SetMode(unsigned int modeChoice)
    {
        switch(modeChoice)
        {
            case 0:
            {
                Mode = POSITION;
                Setting = DEFAULT;
            }
            break;
            case 1:
            {
                Mode = ORIENTATION;
                Setting = DEFAULT;
            }
            break;
            case 2:
            {
                Mode = POSITION;
                Setting = CONST_VEL;
            }
            break;
            case 3:
            {
                Mode = NONE;
                Setting = DEFAULT;
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
    void Slow(float a)
    {
        if(a>1) a=0.5;
        if(a<0) a=0.5;
        if(abs_float(drive.wL) > SlowThreshold)
            wheelsVel.leftWheel = wheelsVel.leftWheel*a;
        else
        {
           wheelsVel.leftWheel = 0.0;
           isRunning = false;
        }
        if(abs_float(drive.wR) > SlowThreshold)
            wheelsVel.rightWheel = wheelsVel.rightWheel*a;
        else
        {
            wheelsVel.rightWheel = 0.0;
            isRunning = false;
        }
        isTriggered = true;
    }

    bool Update()
    {
	if(Mode == NONE)
	{
	    this->Slow(SlowCoef);

	}
        if(Mode == ORIENTATION && Setting == DEFAULT)
        {
            if(isRunning)
            {

                //this->SetVelocities(1,1);
                if(abs_float(odometry.posture.th - targetPos.th) < error_orientation)
                {
                    isRunning = false;
                    this->Stop();
                } else {
                   this->SetVelocities(-wr_max_orientation*sign(odometry.posture.th - targetPos.th),-wl_max_orientation*sign(odometry.posture.th - targetPos.th));
                }
            }
        }
        if(Mode == POSITION && Setting == DEFAULT)
        {
            if(isRunning)
            {
		ex = odometry.posture.x + d*cos(odometry.posture.th) - targetPos.x;
                ey = odometry.posture.y + d*sin(odometry.posture.th) - targetPos.y;
                if(abs_float(ex) < error_position && abs_float(ey) < error_position)
                {
                    isRunning = false;
                    this->Stop();
                } else {
                    //regulacja polozenia
                    v = -k*(cos(odometry.posture.th)*ex + sin(odometry.posture.th)*ey);
                    w = -k*(-sin(odometry.posture.th)*ex/d + cos(odometry.posture.th)*ey/d);
                    this->SetVelocities((v+0.5*WHEEL_BASE*w)/WHEEL_RADIUS, -(v-0.5*WHEEL_BASE*w)/WHEEL_RADIUS);   //pamietaj o minusie przy lewym kole
                }
            }
        }
	if(Mode == POSITION && Setting == CONST_VEL)
        {
            if(isRunning)
            {
		ex = odometry.posture.x + d*cos(odometry.posture.th) - targetPos.x;
                ey = odometry.posture.y + d*sin(odometry.posture.th) - targetPos.y;
                if(abs_float(ex) < error_position && abs_float(ey) < error_position)
                {
                     isRunning = false;
                     this->Stop();
                } else {

            v_ax = -ex0/(sqrt(ex0*ex0 + ey0*ey0));
		    v_ay = -ey0/(sqrt(ex0*ex0 + ey0*ey0));
		    if(-k*ey + eps*v_ay == 0 && -k*ex + eps*v_ax == 0)
		    {
		        w_x = -v_ax/(sqrt(v_ax*v_ax + v_ay*v_ay));
		        w_y = -v_ay/(sqrt(v_ax*v_ax + v_ay*v_ay));
		    }
		    else
		    {
		    w_x = ((-k*ex + eps*v_ax)*V_const/(sqrt((-k*ex + eps*v_ax)*(-k*ex + eps*v_ax) + (-k*ey + eps*v_ay)*(-k*ey + eps*v_ay))));
		    w_y = ((-k*ey + eps*v_ay)*V_const/(sqrt((-k*ex + eps*v_ax)*(-k*ex + eps*v_ax) + (-k*ey + eps*v_ay)*(-k*ey + eps*v_ay))));
		    }
                    v = cos(odometry.posture.th)*w_x + sin(odometry.posture.th)*w_y;
                    w = -sin(odometry.posture.th)*w_x/d + cos(odometry.posture.th)*w_y/d;
                    this->SetVelocities((v+0.5*WHEEL_BASE*w)/WHEEL_RADIUS, -(v-0.5*WHEEL_BASE*w)/WHEEL_RADIUS);   //pamietaj o minusie przy lewym kole
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

