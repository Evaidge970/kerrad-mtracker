#include "global_data.h"
#include "math_routines.h"
#include "drive_controller.h"
#include "high_level_controller.h"
#include "serial_communication.h"
#include "odometry.h"

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
            uint16_t res : 12;
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

void InterpretCommand(uint16_t *inBuf, uint16_t *outBuf)	//buffer - wska�nik na bufor odbiornika
{
	uint16_t * dataIn;
	uint16_t * dataOut;

	uint16_t bufIn[64];

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
            CmdHLControl * cmd_last;


            //volatile CmdHLControl cmdTest;

            cmd->status.all = Get16bitDataFromBuf(dataIn);
            cmd->wl = (int16_t) Get16bitDataFromBuf(dataIn);
            cmd->wr = (int16_t) Get16bitDataFromBuf(dataIn);
            cmd->x = GetFloatDataFromBuf(dataIn);
            cmd->y = GetFloatDataFromBuf(dataIn);
            cmd->th = GetFloatDataFromBuf(dataIn);

            DINT;

            if(!hlController.isRunning)
            {
                if(cmd != cmd_last) //komenda musi być inna niż poprzednia żeby rozpocząć regulację
                {
                   // hlController.targetPos.th = cmd->th;
                    hlController.targetPos.x = cmd->x;
                    hlController.targetPos.y = cmd->y;
                    hlController.isRunning = true;
                }
            }

            if(cmd->status.bit.requestData) //jesli w trybie wysylania pustych ramek (tylko odczyt danych z robota)
            {
                
            }

            // set velocity?
            if(cmd->status.bit.drvRegEnable)    // if bit 0 set velocity
            {
                if (!drive.regEnable)
                {
                    drive.regEnable = 1;
                    //DriveControllerIntegralReset();
                }

                //hlController.SetVelocities(_IQ8toF(cmd->wr), -_IQ8toF(cmd->wl));
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

            cmd_last = cmd; //zapisuje ostatnią ramkę w cmd_last

            InitializeFrameHead(3, 22, outBuf);

            Store16bitDataInBuf(dataOut, 0);
            Store16bitDataInBuf(dataOut, (int16) (-_IQ8(drive.wL)));
            Store16bitDataInBuf(dataOut, (int16) (_IQ8(drive.wR)));
            StoreFloatDataInBuf(dataOut, odometry.posture.x);
            StoreFloatDataInBuf(dataOut, odometry.posture.y);
            ////StoreFloatDataInBuf(dataOut, hlController.targetPos.th);
//            StoreFloatDataInBuf(dataOut, drive.regL.e_phi);
//            StoreFloatDataInBuf(dataOut, drive.regL.e_w);

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
