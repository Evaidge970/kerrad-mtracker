#include "global_data.h"
#include "math_routines.h"
#include "drive_controller.h"
#include "high_level_controller.h"
#include "serial_communication.h"
#include "odometry.h"
#include "hardware_interface.h"

extern "C"
{
	#include "IQmathLib.h"
}

typedef enum
{
	NONE, SET_WHEEL_VELOCITY, SET_LEDS, SET_WHEEL_VELOCITY_ODOMETRY, SET_DRV_CONTR_PARAMS, SET_CAR_CONTROLS, HIGH_LVL_CONTROL
} CmdName;

struct CmdSetWheelVel
{
	int16_t vl;
	int16_t vr;
};

struct StatusCmdSetWheelVelOdometry
{
	uint16_t drvRegEnable : 1;
	uint16_t motorEnable : 1;
	uint16_t setOdometry : 1;
	uint16_t res : 13;
};

struct CmdWheelVelOdometry
{
	union
	{
		unsigned int all;
		struct
		{
			uint16_t drvRegEnable : 1;
			uint16_t motorEnable : 1;
			uint16_t setOdometry : 1;
			uint16_t res : 13;
		} bit;
	}  status;
	int16_t wl;
	int16_t wr;
	float x;
	float y;
	float th;
};

struct CmdWheelVelOdometryAns
{
	uint16_t status;
	int16_t wl;
	int16_t wr;
	float x;
	float y;
	float th;
	int16_t ul;
	int16_t ur;
};

struct CmdCarControls
{
    union
    {
        unsigned int all;
        struct
        {
            uint16_t drvRegEnable : 1;
            uint16_t motorEnable : 1;
            uint16_t setOdometry : 1;
            uint16_t res : 13;
        } bit;
    }  status;
    int16_t wheelVelocity;
    int16_t servoData;
    float x;
    float y;
    float th;
};

struct CmdDriveCntrParams
{
	int16_t kp;
	int16_t ki;
	int16_t kc1;
	int16_t kc2;
};

struct CmdHLControl
{
    union
    {
        unsigned int all;
        struct
        {
            uint16_t drvRegEnable : 1;
            uint16_t motorEnable : 1;
            uint16_t setOdometry : 1;
            uint16_t requestData : 1;
            uint16_t clearBuffor : 1;
            uint16_t modeChoice : 2;
            uint16_t res : 9;
        } bit;
    }  status;
    int16_t wl;
    int16_t wr;
    float x;
    float y;
    float th;
};

void InitializeFrameHead(uint16_t c, uint16_t s, uint16_t * outBuf)
{
	outBuf[0] = s + 4;	//size
	outBuf[1] = 1;	// group number
	outBuf[2] = c;	// command
	outBuf[3] = 0;	// robot number
	outBuf[4] = 0;	// robot number


}

CmdHLControl cmd_buffor[10]; //kolejka

void InitHLBuffer()
{
    for(int i=0; i<10; i++)
    {
	cmd_buffor[i].status.all = 0;
	cmd_buffor[i].wl = 0;
	cmd_buffor[i].wr = 0;
	cmd_buffor[i].x = 0;
	cmd_buffor[i].y = 0;
	cmd_buffor[i].th = 0;
        cmd_buffor[i].status.bit.requestData = 1;
        //cmd_buffor[i].status.bit.drvRegEnable = 1;
        //cmd_buffor[i].status.bit.motorEnable = 1;
    }
}

void AddToBuffor(CmdHLControl& cmd_buffor, CmdHLControl* cmd) //Add cmd to buffor
{
    cmd_buffor.status.all = cmd->status.all;
    cmd_buffor.wl = cmd->wl;
    cmd_buffor.wr = cmd->wr;
    cmd_buffor.x = cmd->x;
    cmd_buffor.y = cmd->y;
    cmd_buffor.th = cmd->th;
}

bool IsCmdNull(CmdHLControl& cmd) //Check if given cmd is cmd_null
{
    if(cmd.status.all == 8 && cmd.x == 0 && cmd.y == 0 && cmd.th == 0 && cmd.wl == 0 && cmd.wr == 0)
        {
            return true;
        }
    return false;
}

void InterpretCommand(uint16_t *inBuf, uint16_t *outBuf)	//buffer - wska�nik na bufor odbiornika
{
	uint16_t * dataIn;
	uint16_t * dataOut;

	uint16_t bufIn[64];
	uint16_t bufInN[64];

   	dataIn = inBuf + 5;
	dataOut = outBuf + 5;

	CmdName cmdType = (CmdName) *(inBuf + 2);   // cmd code

	switch(cmdType)
	{
		case SET_WHEEL_VELOCITY:
		{
			CmdSetWheelVel * cmd = (CmdSetWheelVel *) bufIn;
			cmd->vl = (int16_t)Get16bitDataFromBuf(dataIn);
			cmd->vr = (int16_t)Get16bitDataFromBuf(dataIn);

			DINT;
            float refWl = (int32)(cmd->vl * (int32)0x28f6);
            float refWr = (int32)(cmd->vr * (int32)0x28f6);

			hlController.SetVelocities(refWr, refWl);

			EINT;
		}

		break;

		case SET_WHEEL_VELOCITY_ODOMETRY:
		{
			CmdWheelVelOdometry * cmd = (CmdWheelVelOdometry *) bufIn;

			volatile CmdWheelVelOdometry cmdTest;

			cmd->status.all = Get16bitDataFromBuf(dataIn);
			cmd->wl = (int16_t) Get16bitDataFromBuf(dataIn);
			cmd->wr = (int16_t) Get16bitDataFromBuf(dataIn);
			cmd->x = GetFloatDataFromBuf(dataIn);
			cmd->y = GetFloatDataFromBuf(dataIn);
			cmd->th = GetFloatDataFromBuf(dataIn);

			DINT;

			// set velocity?
			if(cmd->status.bit.drvRegEnable)	// if bit 0 set velocity
			{
				if (!drive.regEnable)
				{
					drive.regEnable = 1;
					//DriveControllerIntegralReset();
				}

	            hlController.SetVelocities(_IQ8toF(cmd->wr), -_IQ8toF(cmd->wl));
			}
			else
				drive.regEnable = 0;

			// motor amplifiers enable?
			if(cmd->status.bit.motorEnable)
			{
				if (!drive.motorEnable)
					drive.EnableOutput();
					drive.motorEnable = 1;
			}
			else
			{
				if (drive.motorEnable)
					drive.DisableOutput();

                hlController.SetVelocities(0, 0);
				drive.motorEnable = 0;
			}
			EINT;

			InitializeFrameHead(3, 22, outBuf);

			Store16bitDataInBuf(dataOut, 0);
			Store16bitDataInBuf(dataOut, (int16) (-_IQ8(drive.wL)));
			Store16bitDataInBuf(dataOut, (int16) (_IQ8(drive.wR)));
			StoreFloatDataInBuf(dataOut, odometry.posture.x);
			StoreFloatDataInBuf(dataOut, odometry.posture.y);
//            StoreFloatDataInBuf(dataOut, drive.regL.e_phi);
//            StoreFloatDataInBuf(dataOut, drive.regL.e_w);

			StoreFloatDataInBuf(dataOut, odometry.posture.th);
			Store16bitDataInBuf(dataOut, (int16)_IQ12(drive.regL.u));
			Store16bitDataInBuf(dataOut, (int16)_IQ12(drive.regR.u));

			//Update odometry?
			if(cmd->status.bit.setOdometry)	// if bit 1 set then update odometry
			{
				DINT;
				Posture2D pos;
				pos.x = cmd->x;
				pos.y = cmd->y;
				pos.th = cmd->th;
				EINT;
				odometry.Set(pos);
			}
		}
		break;

        case SET_CAR_CONTROLS:
        {
            CmdCarControls * cmd = (CmdCarControls *) bufIn;

            cmd->status.all = Get16bitDataFromBuf(dataIn);
            cmd->wheelVelocity = (int16_t) Get16bitDataFromBuf(dataIn);
            cmd->servoData = -(int16_t) Get16bitDataFromBuf(dataIn);
            cmd->x = GetFloatDataFromBuf(dataIn);
            cmd->y = GetFloatDataFromBuf(dataIn);
            cmd->th = GetFloatDataFromBuf(dataIn);

            DINT;

            // set velocity?
            if(cmd->status.bit.drvRegEnable)    // if bit 0 set velocity
            {
                if (!drive.regEnable)
                {
                    drive.regEnable = 1;
                    //DriveControllerIntegralReset();
                }

                drive.refWrear = -_IQ8toF(cmd->wheelVelocity);
                servo.setValue = _IQ12toF(cmd->servoData);
            }
            else
                drive.regEnable = 0;

            // motor amplifiers enable?
            if(cmd->status.bit.motorEnable)
            {
                if (!drive.motorEnable)
                    drive.EnableOutput();
                drive.motorEnable = 1;

            }
            else
            {
                if (drive.motorEnable)
                    drive.DisableOutput();
                drive.refWrear = 0;
                drive.motorEnable = 0;
            }
            EINT;

            InitializeFrameHead(3, 18, outBuf);

            Store16bitDataInBuf(dataOut, 0);
            Store16bitDataInBuf(dataOut, (int16) (-_IQ8(drive.wRear)));
            StoreFloatDataInBuf(dataOut, odometry.posture.x);
            StoreFloatDataInBuf(dataOut, odometry.posture.y);
            StoreFloatDataInBuf(dataOut, odometry.posture.th);
            Store16bitDataInBuf(dataOut, (int16)_IQ12(drive.regL.u));

            //Update odometry?
            if(cmd->status.bit.setOdometry) // if bit 1 set then update odometry
            {
                DINT;
                Posture2D pos;
                pos.x = cmd->x;
                pos.y = cmd->y;
                pos.th = cmd->th;
                EINT;
                odometry.Set(pos);
            }
        }
        break;

		case SET_DRV_CONTR_PARAMS:
		{

			CmdDriveCntrParams * cmd = (CmdDriveCntrParams *) bufIn;

			cmd->kp = (int16_t) Get16bitDataFromBuf(dataIn);
			cmd->ki = (int16_t) Get16bitDataFromBuf(dataIn);
			cmd->kc1 = (int16_t) Get16bitDataFromBuf(dataIn);
			cmd->kc2 = (int16_t) Get16bitDataFromBuf(dataIn);

			DINT;

			Store16bitDataInBuf(dataOut, (int16_t)_IQ10(drive.regL.Kp));
			Store16bitDataInBuf(dataOut, (int16_t)_IQ10(drive.regL.Ki));
			Store16bitDataInBuf(dataOut, (int16_t)_IQ10(drive.regL.Kc1));
			Store16bitDataInBuf(dataOut, (int16_t)_IQ10(drive.regL.Kc2));

			drive.regL.Kp = drive.regR.Kp = _IQ10toF(cmd->kp);
			drive.regL.Ki = drive.regR.Ki = _IQ10toF(cmd->ki);
			drive.regL.Kc1 = drive.regR.Kc1 = _IQ10toF(cmd->kc1);
			drive.regL.Kc2 = drive.regR.Kc2 = _IQ10toF(cmd->kc2);

			EINT;

			InitializeFrameHead(4, 8, outBuf);
		}
		break;

		case HIGH_LVL_CONTROL:
        {

            CmdHLControl * cmd = (CmdHLControl *) bufIn;
            //CmdHLControl * cmd_last;

            //pusta komenda do wypelnienia kolejki
            CmdHLControl * cmd_null = (CmdHLControl *) bufInN;
            cmd_null->status.bit.drvRegEnable = 0;
            cmd_null->status.bit.motorEnable = 0;
            cmd_null->status.bit.requestData = 1;
            cmd_null->status.bit.setOdometry = 0;
            cmd_null->status.bit.clearBuffor = 0;
            cmd_null->status.bit.modeChoice = 0;
	    cmd_null->status.bit.res = 0;
            cmd_null->wl = 0;
            cmd_null->wr = 0;
            cmd_null->x = 0;
            cmd_null->y = 0;
            cmd_null->th = 0;

            cmd->status.all = Get16bitDataFromBuf(dataIn);
            cmd->wl = (int16_t) Get16bitDataFromBuf(dataIn);
            cmd->wr = (int16_t) Get16bitDataFromBuf(dataIn);
            cmd->x = GetFloatDataFromBuf(dataIn);
            cmd->y = GetFloatDataFromBuf(dataIn);
            cmd->th = GetFloatDataFromBuf(dataIn);

            DINT;

            if(!cmd->status.bit.requestData) //jesli nowy rozkaz
            {
                if(cmd->status.bit.clearBuffor) //jesli nowy rozkaz czysci kolejke i przerywa aktualne zadanie
                {
                    AddToBuffor(cmd_buffor[0], cmd);
                    AddToBuffor(cmd_buffor[1], cmd_null);
                    AddToBuffor(cmd_buffor[2], cmd_null);

                    AddToBuffor(cmd_buffor[3], cmd_null);
                    AddToBuffor(cmd_buffor[4], cmd_null);
                    AddToBuffor(cmd_buffor[5], cmd_null);
                    AddToBuffor(cmd_buffor[6], cmd_null);
                    AddToBuffor(cmd_buffor[7], cmd_null);
                    AddToBuffor(cmd_buffor[8], cmd_null);
                    AddToBuffor(cmd_buffor[9], cmd_null);




                }
                else if(!hlController.isRunning)
                {
                    AddToBuffor(cmd_buffor[0], cmd);

                }
                else //nowy rozkaz na koniec kolejki
                {
                    for(int i=0; i<10; i++) //pierwsza znaleziona pusta komenda w kolejce zostanie zapelniona
                    {
                        if(IsCmdNull(cmd_buffor[i]))
                        {
                            AddToBuffor(cmd_buffor[i], cmd);
                            break; //end for loop
                        }
                    }
                }
                hlController.targetPos.th = cmd_buffor[0].th;
                hlController.targetPos.x = cmd_buffor[0].x;
                hlController.targetPos.y = cmd_buffor[0].y;
                hlController.isRunning = true;
                //ustaw tryb HLControllera przed wykonaniem zadania
                hlController.SetMode((unsigned int)cmd_buffor[0].status.bit.modeChoice);
		hlController.SetErrorConstVelMode(); //ustawia blad w chwili 0 dla trybu CONST_VEL
            }
            else //jesli w trybie wysylania pustych ramek (tylko odczyt danych z robota)
            {
                


            }


            //wykonuj pierwszy rozkaz z kolejki
            // set velocity?
            if(cmd_buffor[0].status.bit.drvRegEnable)    // if bit 0 set velocity
            {
                if (!drive.regEnable)
                {
                    drive.regEnable = 1;
                    //DriveControllerIntegralReset();
                }
            }
            else
                drive.regEnable = 0;

            // motor amplifiers enable?
            if(cmd_buffor[0].status.bit.motorEnable)
            {
                if (!drive.motorEnable)
                    drive.EnableOutput();
                    drive.motorEnable = 1;
            }
            else
            {
                if (drive.motorEnable)
                    drive.DisableOutput();

                hlController.SetVelocities(0, 0);
                drive.motorEnable = 0;
            }

            //przesuwanie kolejki
            if(!hlController.isRunning)
            {
                for(int i=0; i<10-1; i++)
                {
                    cmd_buffor[i] = cmd_buffor[i+1];
                }
                AddToBuffor(cmd_buffor[10-1], cmd_null);

                hlController.targetPos.th = cmd_buffor[0].th;
                hlController.targetPos.x = cmd_buffor[0].x;
                hlController.targetPos.y = cmd_buffor[0].y;
                hlController.isRunning = true;
                //ustaw tryb HLControllera przed wykonaniem zadania
                hlController.SetMode((unsigned int)cmd_buffor[0].status.bit.modeChoice);
		hlController.SetErrorConstVelMode(); //ustawia blad w chwili 0 dla trybu CONST_VEL
            }

            EINT;

            //cmd_last = cmd; //zapisuje ostatnią ramkę w cmd_last

            InitializeFrameHead(3, 22, outBuf);

            Store16bitDataInBuf(dataOut, 0);
            Store16bitDataInBuf(dataOut, (int16) (-_IQ8(drive.wL)));

            Store16bitDataInBuf(dataOut, (int16) (_IQ8(drive.wR)));
            StoreFloatDataInBuf(dataOut, odometry.posture.x);
            StoreFloatDataInBuf(dataOut, odometry.posture.y);

//            StoreFloatDataInBuf(dataOut, drive.regL.e_phi);
//            StoreFloatDataInBuf(dataOut, drive.regL.e_w);
            //StoreFloatDataInBuf(dataOut, (int16) cmd_buffor[0].status.all);
            StoreFloatDataInBuf(dataOut, odometry.posture.th);
            Store16bitDataInBuf(dataOut, (int16)_IQ12(drive.regL.u));
            Store16bitDataInBuf(dataOut, (int16)_IQ12(drive.regR.u));

            //Update odometry?
            if(cmd->status.bit.setOdometry) // if bit 1 set then update odometry
            {
                DINT;
                Posture2D pos;
                pos.x = cmd->x;
                pos.y = cmd->y;
                pos.th = cmd->th;
                EINT;
                odometry.Set(pos);
            }
        }
        break;

		default:
		break;
	}
}
