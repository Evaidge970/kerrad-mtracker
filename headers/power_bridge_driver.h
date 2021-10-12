#pragma once

#include "hardware_interface.h"
#include "math_routines.h"
#include <stdlib.h>

class PowerBridgeDriver
{
     static const uint16_t I_MAX_PWM = (48000>>5);
     static const uint16_t REG_BIB_MODE  = (3752/2);

     void SetModeUniVOLT()
     {
         EALLOW;

         EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM4A on period
         EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM4A on event B, down count*/

         EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM5A on period
         EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM5A on event B, down count*/

         OutVoltageL() = OutVoltageR() = 0;

         //DIR-R - Enable an GPIO output on GPIO7, set it high
         GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;   // Enable pullup on GPIO7
         GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;  // GPIO7 = GPIO7
         GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;

         //DIR-L - Enable an GPIO output on GPIO9, set it high
         GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;   // Enable pullup on GPIO9
         GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;  // GPIO9 = GPIO9
         GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;

         //BR-R - BREAK-R an GPIO output on GPIO10, set it high
         GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;   // Enable pullup on GPIO10
         GpioDataRegs.GPASET.bit.GPIO10 = 1;
         GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;  // GPIO10 = GPIO10
         GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;

         //BR-L - BREAK-L an GPIO output on GPIO11, set it high
         GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;   // Enable pullup on GPIO11
         GpioDataRegs.GPASET.bit.GPIO11 = 1;
         GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;  // GPIO11 = GPIO11
         GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;


         GpioDataRegs.GPACLEAR.bit.GPIO10  = 1;
         GpioDataRegs.GPACLEAR.bit.GPIO11  = 1;

         EDIS;
     }

     void SetModeBibVOLT()
     {
         EALLOW;

         EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM4A on period
         EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM4A on event B, down count*/

         EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM4A on period
         EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM4A on event B, down count*/

         OutVoltageL() = OutVoltageR() = REG_BIB_MODE;

         EPwm4Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM4A on period
         EPwm4Regs.AQCTLB.bit.CAD = AQ_CLEAR;            // Set PWM4A on event A, up count*/

         EPwm5Regs.AQCTLB.bit.CAU = AQ_SET;          // Clear PWM4A on period
         EPwm5Regs.AQCTLB.bit.CAD = AQ_CLEAR;            // Set PWM4A on event A, up count*/

         //Set output to PWM unit
         GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;
         GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
         GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
         GpioDataRegs.GPASET.bit.GPIO7 = 0;

         //Set output to PWM unit
         GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;
         GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
         GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;
         GpioDataRegs.GPASET.bit.GPIO9 = 0;

         //BR-R - BREAK-R an GPIO output on GPIO10, set it high
         GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;
         GpioDataRegs.GPASET.bit.GPIO10 = 1;
         GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
         GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;

         //BR-L - BREAK-L an GPIO output on GPIO11, set it high
         GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;
         GpioDataRegs.GPASET.bit.GPIO11 = 1;
         GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
         GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;

         GpioDataRegs.GPACLEAR.bit.GPIO10  = 1;
         GpioDataRegs.GPACLEAR.bit.GPIO11  = 1;

         EDIS;
     }

     volatile uint16_t & OutVoltageL()
     {
         return (uint16_t &) EPwm5Regs.CMPA.half.CMPA;
     }

     volatile uint16_t & OutVoltageR()
     {
         return (uint16_t &) EPwm4Regs.CMPA.half.CMPA;
     }

     volatile uint16_t & OutCurrentL()
     {
         return (uint16_t &) EPwm1Regs.CMPA.half.CMPA;
     }

     volatile uint16_t & OutCurrentR()
     {
         return (uint16_t &) EPwm2Regs.CMPA.half.CMPA;
     }

     void SetDirL()
     {
         GpioDataRegs.GPACLEAR.bit.GPIO9  = 1;
     }

     void ResetDirL()
     {
         GpioDataRegs.GPASET.bit.GPIO9 = 1;
     }

     void SetDirR()
     {
         GpioDataRegs.GPACLEAR.bit.GPIO7  = 1;
     }

     void ResetDirR()
     {
         GpioDataRegs.GPASET.bit.GPIO7 = 1;
     }

public:

     typedef enum
     {
         DOUBLE_MODE_UNI_VOLT, DOUBLE_MODE_BIP_VOLT, DOUBLE_MODE_UNI_CURR, SINGLE_MODE_UNI_VOLT, SINGLE_MODE_BIP_VOLT, SINGLE_MODE_UNI_CURR
     } OutputMode;

     OutputMode outputMode;

     volatile uint16_t   regEnable:1;
     volatile uint16_t   motorEnable:1;

     void SetMode(OutputMode m)
     {
         outputMode = m;
         switch(outputMode)
         {
             case DOUBLE_MODE_UNI_VOLT:
                 SetModeUniVOLT();
                 OutCurrentL() = OutCurrentR() = I_MAX_PWM;
             break;

             case DOUBLE_MODE_BIP_VOLT:
                 SetModeBibVOLT();
                 OutCurrentL() = OutCurrentR() = I_MAX_PWM;
             break;

             case DOUBLE_MODE_UNI_CURR:
             break;

             case SINGLE_MODE_UNI_VOLT:
                 SetModeUniVOLT();
                 OutCurrentL() = OutCurrentR() = I_MAX_PWM;
             break;

             case SINGLE_MODE_BIP_VOLT:
                 SetModeBibVOLT();
                 OutCurrentL() = OutCurrentR() = I_MAX_PWM;
             break;

             case SINGLE_MODE_UNI_CURR:
             break;
         }
     }

     void SetOutput(float uR, float uL)
     {
          switch (outputMode)
          {
              case DOUBLE_MODE_UNI_VOLT:
              {
                  int16_t uOutR = (int16_t)(-uR * REG_BIB_MODE);
                  int16_t uOutL = (int16_t)(-uL * REG_BIB_MODE);

                  if (uOutR < 0)
                      SetDirR();
                  else
                      ResetDirR();

                  if (uOutL < 0)
                      SetDirL();
                  else
                      ResetDirL();

                  OutVoltageR() = abs(uOutR) << 1;
                  OutVoltageL() = abs(uOutL) << 1;
              }
              break;

              case DOUBLE_MODE_BIP_VOLT:
              {
                  int16_t uOutR = (int16_t)(uR * REG_BIB_MODE);
                  int16_t uOutL = (int16_t)(uL * REG_BIB_MODE);

                  OutVoltageR() = uOutR + REG_BIB_MODE;
                  OutVoltageL() = uOutL + REG_BIB_MODE;
              }
              break;

              default:
              break;
          }
      }

     void SetOutput(float u)
     {
          switch (outputMode)
          {
              case SINGLE_MODE_UNI_VOLT:
              {
                  int16_t uOut = (int16_t)(-u * REG_BIB_MODE);

                  if (uOut < 0)
                      SetDirL();
                  else
                      ResetDirL();

                  OutVoltageL() = abs(uOut) << 1;
              }
              break;

              case SINGLE_MODE_BIP_VOLT:
              {
                  int16_t uOut = (int16_t)(u * REG_BIB_MODE);

                  OutVoltageL() = uOut + REG_BIB_MODE;
              }
              break;

              default:
              break;
          }
      }

};

