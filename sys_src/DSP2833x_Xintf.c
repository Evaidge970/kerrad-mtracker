// TI File $Revision: /main/5 $
// Checkin $Date: August 16, 2007   11:06:26 $
//###########################################################################
//
// FILE:   DSP2833x_Xintf.c
//
// TITLE:   DSP2833x Device External Interface Init & Support Functions.
//
// DESCRIPTION:
//
//          Example initialization function for the external interface (XINTF).
//          This example configures the XINTF to its default state.  For an
//          example of how this function being used refer to the
//          examples/run_from_xintf project.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "hardware_interface.h"

// Configure the timing paramaters for Zone 7.
// Notes:
//    This function should not be executed from XINTF
//    Adjust the timing based on the data manual and
//    external device requirements.

void XMemTest(void)
{
	Uint16 *xmemaddr;
	Uint32 j;
	Uint16 i;

test_start:
	xmemaddr = (Uint16*)0x200000U;
	i = 1;
	for(j = 0; j < 0x40000; j++)
	{
		*xmemaddr++ = i++;
	}


	xmemaddr = (Uint16*)0x200000U;
	i = 1;
	for(j = 0; j < 0x40000; j++)
	{
		if(*xmemaddr++ != i++) break;
	}
	if(j != 0x40000)
	{
		goto test_start;	
	}

	xmemaddr = (Uint16*)0x200000U;
	i = 1;
	for(j = 0; j < 0x40000; j++)
	{
		*xmemaddr++ = ~i++;
	}
	xmemaddr = (Uint16*)0x200000U;
	i = 1;
	for(j = 0; j < 0x40000; j++)
	{
		if(*xmemaddr++ != ~i++) break;
	}
	if(j != 0x40000)
	{
		goto test_start;	
	}
}


void InitZone7(void)
{

    // Make sure the XINTF clock is enabled
	SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;

	// Configure the GPIO for XINTF with a 16-bit data bus
	// This function is in DSP2833x_Xintf.c
	InitXintf16Gpio();
    EALLOW;
    // All Zones---------------------------------
    // Timing for all zones based on XTIMCLK = SYSCLKOUT
    XintfRegs.XINTCNF2.bit.XTIMCLK = 0;//ok
    // Buffer up to 3 writes
    XintfRegs.XINTCNF2.bit.WRBUFF = 3;//0 ró¿nica !!!!!
    // XCLKOUT is enabled
    XintfRegs.XINTCNF2.bit.CLKOFF = 0;//ok
    // XCLKOUT = XTIMCLK
    XintfRegs.XINTCNF2.bit.CLKMODE = 0;//ok

    // Zone 7------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING7.bit.XWRLEAD = 1;//ok 
    XintfRegs.XTIMING7.bit.XWRACTIVE = 2;//ok 
    XintfRegs.XTIMING7.bit.XWRTRAIL = 1;//0 ró¿nica !!!!!
    // Zone read timing
    XintfRegs.XTIMING7.bit.XRDLEAD = 1;//ok
    XintfRegs.XTIMING7.bit.XRDACTIVE = 3;//2 ró¿nica !!!!!
    XintfRegs.XTIMING7.bit.XRDTRAIL = 0;//ok

    // don't double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING7.bit.X2TIMING = 0;//ok

    // Zone will not sample XREADY signal
    XintfRegs.XTIMING7.bit.USEREADY = 0;//ok
    XintfRegs.XTIMING7.bit.READYMODE = 1;//

    // 1,1 = x16 data bus
    // 0,1 = x32 data bus
    // other values are reserved
    XintfRegs.XTIMING7.bit.XSIZE = 3;//ok - x16 data bus

   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.
   asm(" RPT #7 || NOP");
	
//	XMemTest();//Test pamiêci zewnêtrznej - przywróciæ !!!!!

}

void InitXintf16Gpio()
{
     EALLOW;
     GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;  // XD15
     GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;  // XD14
     GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3;  // XD13
     GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 3;  // XD12
     GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 3;  // XD11
     GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3;  // XD10
     GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 3;  // XD19
     GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3;  // XD8
     GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 3;  // XD7
     GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 3;  // XD6
     GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 3;  // XD5
     GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 3;  // XD4
     GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 3;  // XD3
     GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 3;  // XD2
     GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 3;  // XD1
     GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 3;  // XD0

     GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 3;  // XA0/XWE1n
     GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 3;  // XA1
     GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 3;  // XA2
     GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 3;  // XA3
     GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 3;  // XA4
     GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 3;  // XA5
     GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 3;  // XA6
     GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 3;  // XA7

     GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 3;  // XA8
     GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 3;  // XA9
     GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 3;  // XA10
     GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 3;  // XA11
     GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 3;  // XA12
     GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 3;  // XA13
     GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 3;  // XA14
     GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 3;  // XA15
     GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 3;  // XA16
     GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 3;  // XA17
     GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 3;  // XA18
     // GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // XA19

//     GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 3;  // XREADY
//	 GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 3;  // XRNW
     GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 3;  // XWE0

//     GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 3;  // XZCS0
     GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 3;  // XZCS7
//     GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // XZCS6
     EDIS;
}

//===========================================================================
// No more.
//===========================================================================
