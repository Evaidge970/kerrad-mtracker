#include "global_data.h"

// XMem Pages
#pragma DATA_SECTION("xmem")
float BufTemp0[0x8000];
#pragma DATA_SECTION("xmem")
float BufTemp1[0x8000];
#pragma DATA_SECTION("xmem")
float BufTemp2[0x8000];

Odometry odometry;
HighLevelController hlController;
Drive drive;
SerialCommunicator serialUsbPort;
SerialCommunicator serialAuxPort;
RadioDriver radio;
ServoDriver servo(20);
