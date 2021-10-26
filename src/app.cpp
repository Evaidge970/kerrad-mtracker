#include "app.h" //zmiana testowa

uint16_t Application::clock1;
uint32_t Application::time;
int16_t Application::extControlGuardTime; // = EXT_CONTROL_TIMEOUT;
volatile bool Application::isMainClockEvent;

void Application::Run()
{

	EINT;

	for(;;)
	{
		DINT;
		if (serialUsbPort.isReceived)
		{
			EINT;
			Led::Blink(2, 7);
			extControlGuardTime = EXT_CONTROL_TIMEOUT;
			InterpretCommand(serialUsbPort.rxFrame, serialUsbPort.txBuf);
			serialUsbPort.StartSend();
			serialUsbPort.StartReceive();
		}

    #ifdef AUX_UART_ENABLED
		DINT;
        if (serialAuxPort.isReceived)
        {
            EINT;
            Led::Blink(2, 7);
            extControlGuardTime = EXT_CONTROL_TIMEOUT;
            InterpretCommand(serialAuxPort.rxFrame, serialAuxPort.txBuf);
            serialAuxPort.StartSend();
            serialAuxPort.StartReceive();
        }
    #endif

        DINT;

        if (radio.isReceived)
        {
            EINT;
            Led::Blink(2, 7);
            extControlGuardTime = EXT_CONTROL_TIMEOUT;
            InterpretCommand(radio.rxBuf, radio.txBuf);
            radio.TriggerSend();
        }

        DINT;
        if (radio.isError)
        {
            EINT;
            Led::Blink(3, 50);
        }

        DINT;
		if (isMainClockEvent)
		{
		    time++;
		    isMainClockEvent = false;
			EINT;
			WDI_OE_TOGGLE;

            if (hlController.Update())
            {
            #ifdef DIFF_VEHICLE
                drive.SetVelocities(hlController.wheelsVel);
            #endif
            }

			servo.Update();
			drive.Update();

			serialUsbPort.CheckTimeOut();
			serialAuxPort.CheckTimeOut();

			Led::OffDelay(2);
			Led::OffDelay(3);
			Led::ToggleDelay(1, 250);
		}

		DINT;
		if (odometry.isReady)
		{
			EINT;
			odometry.Compute();
		}

		serialUsbPort.CheckErrors();
		serialAuxPort.CheckErrors();
		EINT;
	}
}


