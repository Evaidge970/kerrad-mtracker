#include "odometry.h"

const float Odometry::IMP2RAD = 2.0*M_PI*WHEEL_RADIUS/WHEEL_BASE/GEAR_RATIO/ENC_RES_FULL;		// orientation coeff: imp->rad
const float Odometry::ANG_IMP_MAX = WHEEL_BASE/WHEEL_RADIUS*ENC_RES_FULL*GEAR_RATIO/2.0;		// imp per pi rad
const float Odometry::IMP2METER = 2.0*M_PI*WHEEL_RADIUS/GEAR_RATIO/2/ENC_RES_FULL;				// position coeff: imp -> distance

