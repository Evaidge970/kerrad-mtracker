#pragma once

#define CPU_FREQ 	 150E6
#define LSPCLK_FREQ  CPU_FREQ/4

#define APP_MAIN_CLK 	1000.0
#define APP_MAIN_PERIOD	(1.0/APP_MAIN_CLK)

#define EXT_CONTROL_TIMEOUT 500

#define FLASH
#define AUX_UART_ENABLED

// main geometric parameters
#ifdef CAR_VEHICLE
    #define WHEEL_BASE 0.195
    #define WHEEL_RADIUS 0.033
    #define ENC_RES 16.0
    #define GEAR_RATIO 10.0
#endif

#ifdef DIFF_VEHICLE
    #define WHEEL_BASE 0.145
    #define WHEEL_RADIUS 0.025
    #define ENC_RES 32.0
    #define GEAR_RATIO 14.0
#endif

#define ENC_RES_FULL (4*ENC_RES)

