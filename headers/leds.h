#pragma once

#include "hardware_interface.h"

class Led
{
    static uint16_t cnt1;
    static uint16_t cnt2;
    static uint16_t cnt3;

    static void Led1On()
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO48  = 1;
    }

    static void Led2On()
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO49  = 1;
    }

    static void Led3On()
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO61  = 1;
    }

    static void Led1Off()
    {
        GpioDataRegs.GPBSET.bit.GPIO48  = 1;
    }

    static void Led2Off()
    {
        GpioDataRegs.GPBSET.bit.GPIO49  = 1;
    }

    static void Led3Off()
    {
        GpioDataRegs.GPBSET.bit.GPIO61  = 1;
    }

    static void Led1Toggle()
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO48  = 1;
    }

    static void Led2Toggle()
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO49  = 1;
    }

    static void Led3Toggle()
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO61  = 1;
    }

public:
	static void InitHardware()
	{
		EALLOW;

		//LED_1
		GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;   // Enable pullup on GPIO48
		GpioDataRegs.GPBSET.bit.GPIO48 = 1;   // Load output latch
		GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;  // GPIO48 = GPIO48
		GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;   // GPIO48 = output

		//LED_2
		GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;   // Enable pullup on GPIO49
		GpioDataRegs.GPBSET.bit.GPIO49 = 1;   // Load output latch
		GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;  // GPIO49 = GPIO49
		GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1;   // GPIO49 = output

		//LED_3
		GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;   // Enable pullup on GPIO61
		GpioDataRegs.GPBSET.bit.GPIO61 = 1;   // Load output latch
		GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;  // GPIO61 = GPIO61
		GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;   // GPIO61 = output

		//WDI_OE
		GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pullup on GPIO17
		GpioDataRegs.GPASET.bit.GPIO17 = 1;   // Load output latch
		GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;  // GPIO17 = GPIO17
		GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;   // GPIO17 = output

		//SLB-CLK
		GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;   // Enable pullup on GPIO10
		GpioDataRegs.GPASET.bit.GPIO10 = 1;   // Load output latch
		GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;  // GPIO10 = GPIO10
		GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;   // GPIO10 = output

		EDIS;

		Off(1); Off(2); Off(3);

		cnt1 = cnt2 = cnt3 = 0;
	}

	static void Off(const uint16_t l)
	{
		switch (l)
		{
			case 1:
			    Led1Off();
				break;

			case 2:
			    Led2Off();
				break;

			case 3:
			    Led3Off();
				break;

			default:
				break;
		}
	}

	static void On(const uint16_t l)
	{
		switch (l)
		{
			case 1:
			    Led1On();
				break;

			case 2:
				Led2On();
				break;

			case 3:
                Led3On();
				break;

			default:
				break;
		}
	}

    static void Blink(const uint16_t l, const uint16_t time)
    {
        switch (l)
        {
            case 1:
                Led1On();
                cnt1 = time;
                break;

            case 2:
                Led2On();
                cnt2 = time;
                break;

            case 3:
                Led3On();
                cnt3 = time;
                break;

            default:
                break;
        }
    }

	static void Toggle(const uint16_t l)
	{
		switch (l)
		{
			case 1:
				Led1Toggle();
				break;

			case 2:
			    Led2Toggle();
				break;

			case 3:
			    Led3Toggle();
				break;

			default:
				break;
		}
	}

	static void OffDelay(const uint16_t l)
	{

        switch (l)
        {

            case 1:
                if (cnt1)
                    cnt1--;
                else
                    Led1Off();
                break;

            case 2:
                if (cnt2)
                    cnt2--;
                else
                    Led2Off();
                break;

            case 3:
                if (cnt3)
                    cnt3--;
                else
                    Led3Off();
                break;

            default:
                break;
        }
	}

    static void ToggleDelay(const uint16_t l, const uint16_t halfPeriod)
    {

        switch (l)
        {

            case 1:
                if (cnt1)
                    cnt1--;
                else
                {
                    cnt1 = halfPeriod;
                    Led1Toggle();
                }
            break;

            case 2:
                if (cnt2)
                    cnt2--;
                else
                {
                    cnt2 = halfPeriod;
                    Led2Toggle();
                }
            break;

            case 3:
                if (cnt3)
                    cnt3--;
                else
                {
                    cnt3 = halfPeriod;
                    Led3Toggle();
                }
            break;

            default:
            break;
        }
    }

};
