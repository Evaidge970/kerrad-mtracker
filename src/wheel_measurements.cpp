#include <wheel_measurements.h>

const float WheelMeasurements::EQEP_FREQ_CAP = (CPU_FREQ/128.0);
const float WheelMeasurements::SPEED_CONST = M_PI/(ENC_RES*GEAR_RATIO);
const float WheelMeasurements::RAD2IMP = ENC_RES_FULL*GEAR_RATIO/(2.0*M_PI);
const float WheelMeasurements::IMP2RAD = 1.0/RAD2IMP;

