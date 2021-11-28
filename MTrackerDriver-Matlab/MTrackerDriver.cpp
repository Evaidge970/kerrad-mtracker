//#include <cstdint>
#include <windows.h>
#include <stdint.h>
#include "WinCom.h"
#include "mex.h"

#define FRAME_HEADER       0xAA
#define FRAME_GROUP_NUMBER 0x01
#define FRAME_ROBOT_NUMBER 0x0100

#define CMD_SET_WHEELS          0x01 //001
#define CMD_SET_LED             0x02 //010
#define CMD_SET_WHEELS_AND_ODOM 0x03 //011
#define CMD_CAR_SET_WHEELS_AND_ODOM 0x04 //100

#define CMD_HIGH_LVL_CONTROL    0x06 //110



#define MODE_MOTORS_OFF    0x00 //0000
#define MODE_MOTORS_ON     0x03 //0011
#define MODE_SET_ODOMETRY  0x04 //0100
#define MODE_REQUEST_DATA  0x08 //1000


typedef enum MatlabCmd {NoneCmd, OpenCmd, CloseCmd, SetVelCmd, SetOdomCmd, ReadDataCmd, SetFreeWheelsCmd, HLControl}; // HLControl

WinCom com;

#pragma pack(1) 
struct CmdFrame 
{
  uint8_t header;
  uint8_t num_data_bytes;
  uint8_t group_number;
  uint8_t command;
  uint16_t robot_number;
  uint16_t mode;
  int16_t w_l;    // Left wheel angular velocity
  int16_t w_r;
  float x;        // X coordinate
  float y;
  float theta;    // Orientation
  uint16_t crc;
};

struct CmdRespFrame 
{
  uint8_t header;
  uint8_t num_data_bytes;
  uint8_t group_number;
  uint8_t command;
  uint16_t robot_number;
  uint16_t mode;
  int16_t w_l;    // Left wheel angular velocity
  int16_t w_r;
  float x;        // X coordinate
  float y;
  float theta;    // Orientation
  int16_t u_l;
  int16_t u_r;
  uint16_t crc;
};

CmdFrame tx_frame;
uint8_t * txBuf = (uint8_t*) &tx_frame;

float saturateVelocity(float w)
{
	const float Wmax = 30;
	if (w>Wmax) w = Wmax;
	else if (w<-Wmax) w = -Wmax;
	return w;
}


uint16_t CRC16(const uint8_t *data, int len)
{
    static const uint16_t crc_table[] = {
      0x0000,0x8005,0x800F,0x000A,0x801B,0x001E,0x0014,0x8011,
      0x8033,0x0036,0x003C,0x8039,0x0028,0x802D,0x8027,0x0022,
      0x8063,0x0066,0x006C,0x8069,0x0078,0x807D,0x8077,0x0072,
      0x0050,0x8055,0x805F,0x005A,0x804B,0x004E,0x0044,0x8041,
      0x80C3,0x00C6,0x00CC,0x80C9,0x00D8,0x80DD,0x80D7,0x00D2,
      0x00F0,0x80F5,0x80FF,0x00FA,0x80EB,0x00EE,0x00E4,0x80E1,
      0x00A0,0x80A5,0x80AF,0x00AA,0x80BB,0x00BE,0x00B4,0x80B1,
      0x8093,0x0096,0x009C,0x8099,0x0088,0x808D,0x8087,0x0082,
      0x8183,0x0186,0x018C,0x8189,0x0198,0x819D,0x8197,0x0192,
      0x01B0,0x81B5,0x81BF,0x01BA,0x81AB,0x01AE,0x01A4,0x81A1,
      0x01E0,0x81E5,0x81EF,0x01EA,0x81FB,0x01FE,0x01F4,0x81F1,
      0x81D3,0x01D6,0x01DC,0x81D9,0x01C8,0x81CD,0x81C7,0x01C2,
      0x0140,0x8145,0x814F,0x014A,0x815B,0x015E,0x0154,0x8151,
      0x8173,0x0176,0x017C,0x8179,0x0168,0x816D,0x8167,0x0162,
      0x8123,0x0126,0x012C,0x8129,0x0138,0x813D,0x8137,0x0132,
      0x0110,0x8115,0x811F,0x011A,0x810B,0x010E,0x0104,0x8101,
      0x8303,0x0306,0x030C,0x8309,0x0318,0x831D,0x8317,0x0312,
      0x0330,0x8335,0x833F,0x033A,0x832B,0x032E,0x0324,0x8321,
      0x0360,0x8365,0x836F,0x036A,0x837B,0x037E,0x0374,0x8371,
      0x8353,0x0356,0x035C,0x8359,0x0348,0x834D,0x8347,0x0342,
      0x03C0,0x83C5,0x83CF,0x03CA,0x83DB,0x03DE,0x03D4,0x83D1,
      0x83F3,0x03F6,0x03FC,0x83F9,0x03E8,0x83ED,0x83E7,0x03E2,
      0x83A3,0x03A6,0x03AC,0x83A9,0x03B8,0x83BD,0x83B7,0x03B2,
      0x0390,0x8395,0x839F,0x039A,0x838B,0x038E,0x0384,0x8381,
      0x0280,0x8285,0x828F,0x028A,0x829B,0x029E,0x0294,0x8291,
      0x82B3,0x02B6,0x02BC,0x82B9,0x02A8,0x82AD,0x82A7,0x02A2,
      0x82E3,0x02E6,0x02EC,0x82E9,0x02F8,0x82FD,0x82F7,0x02F2,
      0x02D0,0x82D5,0x82DF,0x02DA,0x82CB,0x02CE,0x02C4,0x82C1,
      0x8243,0x0246,0x024C,0x8249,0x0258,0x825D,0x8257,0x0252,
      0x0270,0x8275,0x827F,0x027A,0x826B,0x026E,0x0264,0x8261,
      0x0220,0x8225,0x822F,0x022A,0x823B,0x023E,0x0234,0x8231,
      0x8213,0x0216,0x021C,0x8219,0x0208,0x820D,0x8207,0x0202
    };

    uint16_t crc_word = 0xFFFF;

    while (len--)
      crc_word = (crc_word << 8) ^ crc_table[(((crc_word >> 8) & 0x00FF) ^ *data++) & 0x00FF];

    return crc_word;
}

void initFrame()
{
	tx_frame.header = FRAME_HEADER;
	tx_frame.num_data_bytes = 22;
	tx_frame.group_number = FRAME_GROUP_NUMBER;
	tx_frame.robot_number = FRAME_ROBOT_NUMBER;
}

bool readFrame(uint8_t * data)
{
	CmdRespFrame * rx_frame = (CmdRespFrame *) data;
    {
      if ((rx_frame->header == FRAME_HEADER) && (rx_frame->group_number == FRAME_GROUP_NUMBER))
      {
        // Omit header byte and include num_data_bytes byte in crc
        // crc has different endianess
          
        uint16_t crc1 = CRC16((uint8_t*) &rx_frame->num_data_bytes, rx_frame->num_data_bytes + 1);  
        uint8_t c1 = rx_frame->crc >> 8;
        uint8_t c2 = 0xFF & rx_frame->crc;
        uint16_t crc2 = c1 + ((uint16_t)c2 << 8);
        if (crc1 == crc2)
		{
          return true;
		}
      }
    }
	
    return false;
}

void prepareFrame(uint16_t mode, uint8_t command)
{
    tx_frame.command = command;
    tx_frame.mode = mode;

    // Omit header byte and include num_data_bytes byte for crc
    // The crc has different endianess
    uint16_t crc = CRC16(((uint8_t*)&tx_frame) + 1, (int)(tx_frame.num_data_bytes + 1));
    uint8_t crc1 = crc >> 8;
    uint8_t crc2 = 0xFF & crc;
    tx_frame.crc = crc1 + crc2 * 256;
}


void setOdometry(float x, float y, float theta)
{
    tx_frame.x = x;
    tx_frame.y = y;
    tx_frame.theta = theta;

    prepareFrame(MODE_SET_ODOMETRY | MODE_MOTORS_ON, CMD_SET_WHEELS_AND_ODOM);
}

void setTarget(float x, float y, float theta) // nowa funkcja - ustaw cel
{
    tx_frame.x = x;
    tx_frame.y = y;
    tx_frame.theta = theta;

    prepareFrame(MODE_MOTORS_ON, CMD_HIGH_LVL_CONTROL);
}

void requestData() // wysyla puste zapytania, zeby robot wyslal aktualna pozycje
{
    tx_frame.x = 0.0;
    tx_frame.y = 0.0;
    tx_frame.theta = 0.0;

    prepareFrame(MODE_MOTORS_ON | MODE_REQUEST_DATA, CMD_HIGH_LVL_CONTROL);
}

void setVelocity(float w_r, float w_l)
{
   w_l = saturateVelocity(w_l);
   w_r = saturateVelocity(w_r);
   tx_frame.w_l =  (int16_t) (w_l * 256.0f);
   tx_frame.w_r = (int16_t) (w_r * 256.0f);
   prepareFrame(MODE_MOTORS_ON, CMD_SET_WHEELS_AND_ODOM);
}

void switchOffMotors()
{
   tx_frame.w_l = 0;
   tx_frame.w_r = 0;
   prepareFrame(MODE_MOTORS_OFF, CMD_SET_WHEELS_AND_ODOM);
}

void stopWheels()
{
    setVelocity(0.0f, 0.0f);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

    char *cmdName;
    MatlabCmd cmdNum = NoneCmd;
	mxArray *InArray;
	uint8_t *outputData;
    int ErrorParsec = 0;

	double * matrixData;

	double wr, wl, x, y, th, ur, ul;
    
	//Convert the Inputs
    if(nrhs >= 1)
	{
        cmdName = (char *) mxArrayToString(prhs[0]);

		if(strcmp(cmdName,"open")== 0)
        {
			matrixData = mxGetPr(prhs[1]);
			cmdNum = OpenCmd;
        }
			
		if(strcmp(cmdName,"close")== 0)
        {
			cmdNum = CloseCmd;
        }
		
		if(strcmp(cmdName,"sendVelocity")== 0)
        {
            matrixData = mxGetPr(prhs[1]);
			cmdNum = SetVelCmd;
        }

		if(strcmp(cmdName,"read")== 0)
        {
			cmdNum = ReadDataCmd;
        }

		if(strcmp(cmdName,"setOdometry")== 0)
        {
            matrixData = mxGetPr(prhs[1]);
			cmdNum = SetOdomCmd;
        }

        if(strcmp(cmdName,"setFreeWheels")== 0)
        {
			cmdNum = SetFreeWheelsCmd;
        }
        
        if(strcmp(cmdName,"highLevelControl")== 0)
        {
			matrixData = mxGetPr(prhs[1]);
			cmdNum = HLControl;
        }
	}
 
   if((ErrorParsec==0)&&(cmdNum != NoneCmd)) 
   {
      switch(cmdNum)
	  {
  			case OpenCmd:
			{
                // get port number
				auto p = (int)matrixData[0]; 
                auto baudrate = (int)matrixData[1];
                int8_t result;
                
				
                //if (!com.Open(p, 115200)) 
                if (!com.Open(p, baudrate))
                {
					printf("Port COM%d is opened successfully.\n", p);
                    result = 0;
                }
                else
                {
					printf("I cannot open COM%d!\n", p);
                    result = -1;
                }
                plhs[0] = mxCreateNumericMatrix(1,1, mxINT8_CLASS, mxREAL);
                outputData = (uint8_t*)mxGetData(plhs[0]);
                outputData[0] = result;
                
                
			}
			break;

			case CloseCmd:
				com.Close();
			break;

			case SetVelCmd:
				
                initFrame();
				setVelocity((float)matrixData[0], (float)matrixData[1]);
				plhs[0] = mxCreateNumericMatrix(1,26, mxUINT8_CLASS, mxREAL);
				
				com.Send((uint8_t *)&tx_frame, 26);
		        
				outputData = (uint8_t*)mxGetData(plhs[0]);
				for(int i = 0; i < 26; i++)
				{
					uint8_t ch = ((uint8_t *)&tx_frame)[i];
					outputData[i] = ch;
				}
			break;

			case ReadDataCmd:
			{
				size_t size = com.Receive();
                double result = 0.0;
				if (readFrame(com.buffer_in))
				{
					CmdRespFrame * rx_frame = (CmdRespFrame *) com.buffer_in;
					
					wl = rx_frame->w_l / 256.0;
					wr = rx_frame->w_r / 256.0;
					x = rx_frame->x;
					y = rx_frame->y;
					th = rx_frame->theta;
                    ul = rx_frame->u_l / 4096.0;
                    ur = rx_frame->u_r / 4096.0;                   
                    result = 1.0;
				}
           
				
				plhs[0] = mxCreateDoubleMatrix(1,8, mxREAL);
				auto out = mxGetPr(plhs[0]);
                out[0] = result;
				out[1] = x;
				out[2] = y;
				out[3] = th;
				out[4] = wr;
				out[5] = wl;
                out[6] = ur;
				out[7] = ul;
			}
			break;

            case SetOdomCmd:
				
                initFrame();
                setOdometry(matrixData[0], matrixData[1], matrixData[2]);
				plhs[0] = mxCreateNumericMatrix(1,26, mxUINT8_CLASS, mxREAL);
				
				com.Send((uint8_t *)&tx_frame, 26);
	        
				outputData = (uint8_t*)mxGetData(plhs[0]);
				for(int i = 0; i < 26; i++)
				{
					uint8_t ch = ((uint8_t *)&tx_frame)[i];
					outputData[i] = ch;
				}
            break;
                
			case SetFreeWheelsCmd:
				
                initFrame();
				switchOffMotors();
				plhs[0] = mxCreateNumericMatrix(1,26, mxUINT8_CLASS, mxREAL);
				
				com.Send((uint8_t *)&tx_frame, 26);
		        
				outputData = (uint8_t*)mxGetData(plhs[0]);
				for(int i = 0; i < 26; i++)
				{
					uint8_t ch = ((uint8_t *)&tx_frame)[i];
					outputData[i] = ch;
				}

			break;
            
            case HLControl:
                initFrame();

                if((bool)matrixData[3]) //jesli chcemy wyslac zadana wartosc
                {
                    setTarget((float)matrixData[0], (float)matrixData[1], (float)matrixData[2]); //pobiera dane wyslane z pliku .m
                }
                else //jesli wysylamy pusta ramke (chcemy tylko odczytac dane z robota)
                {
                    requestData();
                }
                plhs[0] = mxCreateNumericMatrix(1,26, mxUINT8_CLASS, mxREAL);
                    
                com.Send((uint8_t *)&tx_frame, 26);
                    
                outputData = (uint8_t*)mxGetData(plhs[0]);
                for(int i = 0; i < 26; i++)
                {
                    uint8_t ch = ((uint8_t *)&tx_frame)[i];
                    outputData[i] = ch;
                }
				
            break;
             
			default:
			break;

	  }
   } 
   else 
   {
	    if(nlhs == 1)
			plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
	    printf("                                                      \n");
        printf("--------------        MTracker MATLAB driver v.2.1      -----------------\n");
        printf("        (c) KSIS 2017               \n");
		printf("                                                      \n");
        //help();
   }
   return;
}
