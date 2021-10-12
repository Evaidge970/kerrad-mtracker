#include "pid_drive_controller.h"

const float PidDriveController::RAD2IMP = ENC_RES_FULL*GEAR_RATIO/(2.0*M_PI);
const float PidDriveController::IMP2RAD = 1.0/RAD2IMP;
