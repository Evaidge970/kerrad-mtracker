// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:19 $
//###########################################################################
//
// FILE:   DSP2833x_EPwm.c
//
// TITLE:  DSP2833x ePWM Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File


void InitEPwm4Gpio(void);
void InitEPwm5Gpio(void);

// Configure the period for each timer
#define EPWM1_TIMER_TBPRD  1500  // Period register (zakres) !!!!!
//#define EPWM1_MAX_CMPA     375
#define EPWM1_MIN_CMPA       1000 // wype³nienie
//#define EPWM1_MAX_CMPB     650
//#define EPWM1_MIN_CMPB       50

#define EPWM2_TIMER_TBPRD  1500  // Period register
//#define EPWM2_MAX_CMPA     550      
#define EPWM2_MIN_CMPA       1000      
//#define EPWM2_MAX_CMPB     550      
//#define EPWM2_MIN_CMPB       50      

#define EPWM4_TIMER_TBPRD  3752  // Period register
//#define EPWM4_MAX_CMPA      3000  
#define EPWM4_MIN_CMPA      3752  
//#define EPWM4_MAX_CMPB    50 
#define EPWM4_MIN_CMPB     1875  

#define EPWM5_TIMER_TBPRD  3752  // Period register
//#define EPWM5_MAX_CMPA      2000  
//#define EPWM5_MIN_CMPA      1875//50  
#define EPWM5_MIN_CMPA     3752
#define EPWM5_MIN_CMPB     1875  

// To keep track of which way the compare value is moving
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0

//---------------------------------------------------------------------------
// InitEPwm: 
//---------------------------------------------------------------------------
// This function initializes the ePWM(s) to a known state.
//
void InitEPwm(void)
{
   // Initialize ePWM1/2/3/4/5/6

   //tbd...
//InitEPwm1
  // Setup TBCLK
   EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;           // Set timer period 801 TBCLKs
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter
   
   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = EPWM1_MIN_CMPA;     // Set compare A value
 //  EPwm1Regs.CMPB = EPWM1_MAX_CMPB;               // Set Compare B value
   
   // Setup counter mode
//   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Set actions
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;             // Set PWM1A on event A, up count
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;           // Clear PWM1A on event A, down count

//   EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
//   EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM1B on event B, down count

   // Interrupt where we will change the Compare Values
//   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
//   EPwm1Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
//   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;            // Generate INT on 3rd event   
 



//InitEPwm2   
   // Setup TBCLK
   EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;           // Set timer period 801 TBCLKs
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter
   
   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = EPWM2_MIN_CMPA;     // Set compare A value
//   EPwm2Regs.CMPB = EPWM2_MIN_CMPB;               // Set Compare B value
   
   // Setup counter mode
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   


   // Set actions
//   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM2A on event A, up count
//   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM2A on event B, down count
   EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;            // Set PWM2A on event A, up count
   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;          // Clear PWM2A on event B, down count


//   EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;          // Clear PWM2B on zero
//   EPwm2Regs.AQCTLB.bit.PRD = AQ_SET  ;          // Set PWM2B on period

   // Interrupt where we will change the Compare Values
//   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//   EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//   EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event   
   


//InitEPwm4
   // Setup TBCLK
   EPwm4Regs.TBPRD = EPWM4_TIMER_TBPRD;          // Set timer period
   EPwm4Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
   EPwm4Regs.TBCTR = 0x0000;                     // Clear counter

   // Set Compare values
   EPwm4Regs.CMPA.half.CMPA = EPWM4_MIN_CMPA;    // Set compare A value
   EPwm4Regs.CMPB = EPWM4_MIN_CMPB;              // Set Compare B value
   
   // Setup counter mode
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;      // Clock ratio to SYSCLKOUT
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   
   // Setup shadow register load on ZERO
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   


      // Set Actions
/*   EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM4A on period
   EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM4A on event B, down count*/

/*   EPwm4Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM4A on period
   EPwm4Regs.AQCTLB.bit.CAD = AQ_CLEAR;            // Set PWM4A on event A, up count*/

   // Interrupt where we will change the Compare Values
   EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm4Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm4Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event   
   


//InitEPwm5
  // Setup TBCLK
   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm5Regs.TBPRD = EPWM5_TIMER_TBPRD;          // Set timer period
   EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
   EPwm5Regs.TBPHS.half.TBPHS = 0x0000;          // Phase is 0
   EPwm5Regs.TBCTR = 0x0000;                     // Clear counter
   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;      // Clock ratio to SYSCLKOUT
   EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadow register load on ZERO
   EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

  // Set Compare values
/*   EPwm5Regs.CMPA.half.CMPA = EPWM5_MIN_CMPA;    // Set compare A value
   EPwm5Regs.CMPB = EPWM5_MIN_CMPB;    // Set compare B value*/

 //  EPwm5Regs.CMPB = EPWM5_MAX_CMPB;              // Set Compare B value
   
   // Set Actions
/*   EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM5A on period
   EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM5A on event B, down count*/

/*   EPwm5Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM4A on period
   EPwm5Regs.AQCTLB.bit.CAD = AQ_CLEAR;           */

//   EPwm5Regs.AQCTLB.bit.PRD = AQ_CLEAR;          // Clear PWM5A on period
//   EPwm5Regs.AQCTLB.bit.CAU = AQ_SET;            // Set PWM5A on event A, up count

   // Interrupt where we will change the Compare Values
//   EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//   EPwm5Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//   EPwm5Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event   
   

}

//---------------------------------------------------------------------------
// Example: InitEPwmGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ePWM pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 

void InitEPwmGpio(void)
{
   InitEPwm1Gpio();
   InitEPwm2Gpio();
//   InitEPwm3Gpio();
   InitEPwm4Gpio();  
   InitEPwm5Gpio();
//   InitEPwm6Gpio(); 
}

void InitEPwm1Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
 //   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)   
   
/* Configure ePWM-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
   
    EDIS;
}

void InitEPwm2Gpio(void)
{
   EALLOW;
	
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
//    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM3B)

/* Configure ePWM-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
   
    EDIS;
}

void InitEPwm3Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
       
/* Configure ePWM-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
	
    EDIS;
}


#if DSP28_EPWM4
void InitEPwm4Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pull-up on GPIO7 (EPWM4B)

/* Configure ePWM-4 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM4 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B
	
    EDIS;
}
#endif // endif DSP28_EPWM4  


#if DSP28_EPWM5
void InitEPwm5Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pull-up on GPIO9 (EPWM5B)

/* Configure ePWM-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM5 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B
	
    EDIS;
}
#endif // endif DSP28_EPWM5


#if DSP28_EPWM6
void InitEPwm6Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;    // Enable pull-up on GPIO10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up on GPIO11 (EPWM6B)

/* Configure ePWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM6 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B
	
    EDIS;
}
#endif // endif DSP28_EPWM6  

//---------------------------------------------------------------------------
// Example: InitEPwmSyncGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ePWM Synch pins
//

void InitEPwmSyncGpio(void)
{

   EALLOW;

/* Configure EPWMSYNCI  */
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWMSYNCI)
// GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pull-up on GPIO32 (EPWMSYNCI)    

/* Set qualification for selected pins to asynch only */
// This will select synch to SYSCLKOUT for the selected pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;   // Synch to SYSCLKOUT GPIO6 (EPWMSYNCI)
// GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 0;  // Synch to SYSCLKOUT GPIO32 (EPWMSYNCI)    

/* Configure EPwmSync pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPwmSync functional pins.
// Comment out other unwanted lines.   

   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 2;    // Enable pull-up on GPIO6 (EPWMSYNCI)
// GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;   // Enable pull-up on GPIO32 (EPWMSYNCI)    



/* Configure EPWMSYNC0  */

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWMSYNC0)
   GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pull-up on GPIO33 (EPWMSYNC0)    

// GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 3;    // Enable pull-up on GPIO6 (EPWMSYNC0)
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2;   // Enable pull-up on GPIO33 (EPWMSYNC0)    

}



//---------------------------------------------------------------------------
// Example: InitTzGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as Trip Zone (TZ) pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 

void InitTzGpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
   GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)
   GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;    // Enable pull-up on GPIO13 (TZ2)
   GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up on GPIO14 (TZ3)
   GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up on GPIO15 (TZ4)

   GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (TZ5)
// GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up on GPIO28 (TZ5)

   GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (TZ6) 
// GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up on GPIO29 (TZ6)  
   
/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;  // Asynch input GPIO12 (TZ1)
   GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;  // Asynch input GPIO13 (TZ2)
   GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;  // Asynch input GPIO14 (TZ3)
   GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (TZ4)

   GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;  // Asynch input GPIO16 (TZ5)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (TZ5)

   GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;  // Asynch input GPIO17 (TZ6) 
// GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;  // Asynch input GPIO29 (TZ6)  

   
/* Configure TZ pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be TZ functional pins.
// Comment out other unwanted lines.   
   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
   GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2
   GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // Configure GPIO14 as TZ3
   GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // Configure GPIO15 as TZ4

   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 3;  // Configure GPIO16 as TZ5
// GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // Configure GPIO28 as TZ5

   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;  // Configure GPIO17 as TZ6               
// GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // Configure GPIO29 as TZ6  

   EDIS;
}



//===========================================================================
// End of file.
//===========================================================================
