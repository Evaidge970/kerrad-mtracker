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

/*
void CmdMemCopy(CmdHLControl* cmdSource, CmdHLControl* cmdDestination)
{
    MemCopy(cmdSource->status.all, cmdSource->wl, cmdDestination->status.all);
    MemCopy(cmdSource->wl, cmdSource->wr, cmdDestination->wl);
    MemCopy(cmdSource->wr, cmdSource->x, cmdDestination->wr);
    MemCopy(cmdSource->x, cmdSource->x+16, cmdDestination->x);
    MemCopy(cmdSource->x+16, cmdSource->y, cmdDestination->x+16);
    MemCopy(cmdSource->y, cmdSource->y+16, cmdDestination->y);
    MemCopy(cmdSource->y+16, cmdSource->th, cmdDestination->y+16);
    MemCopy(cmdSource->th, cmdSource->th+16, cmdDestination->th);
    MemCopy(cmdSource->th+16, cmdSource->th+32, cmdDestination->th+16);
}
*/

void CmdMemCopy(CmdHLControl* cmdSource, CmdHLControl* cmdDestination)
{
    while(cmdSource < cmdSource+64*sizeof(uint16_t))
    {
        *cmdDestination++ = *cmdSource++;
    }
}

void InitializeFrameHead(uint16_t c, uint16_t s, uint16_t * outBuf)
{
	outBuf[0] = s + 4;	//size
	outBuf[1] = 1;	// group number
	outBuf[2] = c;	// command
	outBuf[3] = 0;	// robot number
	outBuf[4] = 0;	// robot number
}

CmdHLControl * cmd_buffor[3]; //kolejka

void InitHLBuffer()
{
    uint16_t bufInNull0[64];
    uint16_t bufInNull1[64];
    uint16_t bufInNull2[64];
    cmd_buffor[0] = (CmdHLControl*)bufInNull0;
    cmd_buffor[1] = (CmdHLControl*)bufInNull1;
    cmd_buffor[2] = (CmdHLControl*)bufInNull2;
    cmd_buffor[0]->status.bit.requestData = 1;
    cmd_buffor[1]->status.bit.requestData = 1;
    cmd_buffor[2]->status.bit.requestData = 1;
    cmd_buffor[0]->status.bit.drvRegEnable = 1;
    cmd_buffor[1]->status.bit.drvRegEnable = 1;
    cmd_buffor[2]->status.bit.drvRegEnable = 1;
    cmd_buffor[0]->status.bit.motorEnable = 1;
    cmd_buffor[1]->status.bit.motorEnable = 1;
    cmd_buffor[2]->status.bit.motorEnable = 1;

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
            cmd_null->status.bit.drvRegEnable = 1;
            cmd_null->status.bit.motorEnable = 1;
            cmd_null->status.bit.requestData = 1;
            cmd_null->status.bit.setOdometry = 0;
            cmd_null->status.bit.clearBuffor = 0;
            cmd_null->status.bit.modeChoice = 0;
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
                    /*cmd_buffor[0] = cmd;
                    cmd_buffor[1] = cmd_null;
                    cmd_buffor[2] = cmd_null;*/
                    CmdMemCopy(cmd, cmd_buffor[0]);
                    CmdMemCopy(cmd_null, cmd_buffor[1]);
                    CmdMemCopy(cmd_null, cmd_buffor[2]);
                }
                else if(!hlController.isRunning)
                {
                    //cmd_buffor[0] = cmd; //przypisanie aktualnej komendy do kolejki
                    CmdMemCopy(cmd, cmd_buffor[0]);
                }
                else //nowy rozkaz na koniec kolejki
                {
                    for(int i=0; i<3; i++) //pierwsza znaleziona pusta komenda w kolejce zostanie zapelniona
                    {
                        if(&cmd_buffor[i] == &cmd_null)
                        {
                            //cmd_buffor[i] = cmd;
                            CmdMemCopy(cmd, cmd_buffor[i]);
                            break; //end for loop
                        }
                    }
                }
                hlController.targetPos.th = cmd_buffor[0]->th;
                hlController.targetPos.x = cmd_buffor[0]->x;
                hlController.targetPos.y = cmd_buffor[0]->y;
                hlController.isRunning = true;
            }
            else //jesli w trybie wysylania pustych ramek (tylko odczyt danych z robota)
            {
                
            }

            //ustaw tryb HLControllera przed wykonaniem zadania
            hlController.SetMode((unsigned int)cmd_buffor[0]->status.bit.modeChoice);

            //wykonuj pierwszy rozkaz z kolejki
            // set velocity?
            if(cmd_buffor[0]->status.bit.drvRegEnable)    // if bit 0 set velocity
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
            if(cmd_buffor[0]->status.bit.motorEnable)
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
                for(int i=0; i<3-1; i++)
                {
                    //cmd_buffor[i] = cmd_buffor[i+1];
                    CmdMemCopy(cmd_buffor[i+1], cmd_buffor[i]);
                }
                //cmd_buffor[3-1] = cmd_null;
                CmdMemCopy(cmd_null, cmd_buffor[2]);

                hlController.targetPos.th = cmd_buffor[0]->th;
                hlController.targetPos.x = cmd_buffor[0]->x;
                hlController.targetPos.y = cmd_buffor[0]->y;
                hlController.isRunning = true;
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
            StoreFloatDataInBuf(dataOut, (int16) cmd->status.all);
            //StoreFloatDataInBuf(dataOut, odometry.posture.th);
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
