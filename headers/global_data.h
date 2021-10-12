#pragma once

#include "hardware_interface.h"
#include "odometry.h"
#include "drive_controller.h"
#include "serial_communication.h"
#include "analog_measurements.h"
#include "radio_driver.h"
#include "servo_driver.h"
#include "high_level_controller.h"

struct Robot
{
	Posture2D pos;
	Velocity2D vel;
};

extern Odometry odometry;
extern Drive drive;
extern SerialCommunicator serialUsbPort;
extern SerialCommunicator serialAuxPort;
extern RadioDriver radio;
extern ServoDriver servo;
extern HighLevelController hlController;


