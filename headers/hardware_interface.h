#pragma once

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include <stdint.h>

#define WDI_OE_OFF 		GpioDataRegs.GPASET.bit.GPIO17 = 1
#define WDI_OE_ON 		GpioDataRegs.GPACLEAR.bit.GPIO17  = 1
#define WDI_OE_TOGGLE 	GpioDataRegs.GPATOGGLE.bit.GPIO17 = 1

#define SLB_CLK_OFF 	GpioDataRegs.GPASET.bit.GPIO10 = 1
#define SLB_CLK_ON 		GpioDataRegs.GPACLEAR.bit.GPIO10  = 1
#define SLB_CLK_TOGGLE 	GpioDataRegs.GPATOGGLE.bit.GPIO10 = 1
