#pragma once
#include <windows.h>

class WinCom  
{
private:
	int port_number;					
	TCHAR port_name[10];					
	HANDLE hPort;						
	HANDLE sent_sync, recv_sync, prepare_sync;
	
	size_t local_len;
	int state;
	unsigned char local_ptr[50];
public:
	static const int buf_size = 1024;
    int error;
	bool received_frame;
	bool is_closed;
	unsigned char buffer_in[buf_size];						// bufor wejťciowy

	WinCom()
	{
        error = 0;
		port_number = -1;
		is_closed = true;
		state = 1;
	}

	bool Send(const unsigned char *ptr, size_t len)
	{
		DWORD dummy;
		if(is_closed) {
			Sleep(10);
			return true;							// jeśli port zaknięty to wychodzimy
		}

		if(state == 2) // exit
		{
			return false;
		}
		else
		{
			received_frame = false;
            FlushFileBuffers(hPort);		// czyszczenie bufora nadawczego
            if( !WriteFile(hPort, ptr, len, &dummy, 0) ) 
				return false;
        }
		return true;
	}

	DWORD Receive()
	{
		DWORD result; // liczba danych
		ReadFile(hPort, buffer_in, buf_size, &result, 0) ;
		return result;
	}

	void Close()
	{
		if( !is_closed )
			if(!CloseHandle(hPort))
			{
			//cout << "close comm error" << endl;
			}
		is_closed = true;
	}

	int Open(int p, int baudrate)
	{
		COMMTIMEOUTS CommTimeOuts;
		DCB dcb;

		if (!is_closed) return 0;

		//	ustalenie nazwy zależnie od systemu
		if(port_number==-1 )
		{
			#ifdef __OS_WIN95__
				wsprintf(port_name, "COM%d", p);
			#else
                wsprintf(port_name, "\\\\.\\COM%d", p);
			#endif
		}


		// parametry transmmisji
		if ( (hPort=CreateFile(port_name, GENERIC_READ | GENERIC_WRITE,
                  0,                    // exclusive access
                  NULL,                 // no security attrs
                  OPEN_EXISTING,
                  FILE_FLAG_NO_BUFFERING,
                  NULL ))==(HANDLE)-1 )
		{
			return -1;
		}
		SetupComm(hPort, 100, 100);

		CommTimeOuts.ReadIntervalTimeout = MAXDWORD;
		CommTimeOuts.ReadTotalTimeoutMultiplier = 0;//MAXDWORD;
		CommTimeOuts.ReadTotalTimeoutConstant = 0;	
		CommTimeOuts.WriteTotalTimeoutMultiplier = 1;
		CommTimeOuts.WriteTotalTimeoutConstant = 0;
		SetCommTimeouts(hPort, &CommTimeOuts);
		GetCommState(hPort, &dcb);

		dcb.BaudRate = baudrate;
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;

		if(!SetCommState(hPort, &dcb))
			return -2;
		is_closed = false;
		return 0;
	}
};
