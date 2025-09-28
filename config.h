
// ****************************************************************************
//                                 
//                        Project library configuration
//
// ****************************************************************************
// PicoLibSDK - Alternative SDK library for Raspberry Pico/Pico2 and RP2040/RP2350
// Copyright (c) 2023-2025 Miroslav Nemecek, Panda38@seznam.cz, hardyplotter2@gmail.com
// 	https://github.com/Panda381/PicoLibSDK
//	https://www.breatharian.eu/hw/picolibsdk/index_en.html
//	https://github.com/pajenicko/picopad
//	https://picopad.eu/en/
// License:
//	This source code is freely available for any purpose, including commercial.
//	It is possible to take and modify the code or parts of it, without restriction.

#ifndef _CONFIG_H
#define _CONFIG_H

#define DISPHSTX_PICOSDK 1
#define USE_RAND	 1
#define USE_TEXT	 1

#define USE_DRAWCAN0    1
#define USE_DRAWCAN1    0
#define USE_DRAWCAN2    0
#define USE_DRAWCAN3    0
#define USE_DRAWCAN4    0
#define USE_DRAWCAN6    0
#define USE_DRAWCAN8    1
#define USE_DRAWCAN12   0
#define USE_DRAWCAN16   0

// *********
// At this place you can specify the switches and settings you want
// to change from the default configuration in config_def.h.
// *********

#define USE_DISPHSTX		1		// 1=use HSTX Display driver
#define USE_DRAWCAN		1		// use drawing canvas (lib_drawcan*.c, lib_drawcan*.h)
//#define DISPHSTX_DVI_PINOUT	0		// DVI predefined pinout: 0=DVI breakout board, 1=order D2+..CLK-, 2=order CLK-..D2+
//#define DISPHSTX_DISP_SEL	20		// >=0 GPIO pin with display selection switch, -1=do not use display selection switch

// Enable the USE_VREG_LOCKED switch for experimental use only, not for normal use.
// This will allow higher overclocks to be used, but at the cost of lower chip life
// and the possibility of chip destruction.
//#define USE_VREG_LOCKED	1		// 1=enable vreg locked values > 1.30V from function GetVoltageBySysClock() of RP2350

//#define DISPHSTX_CHECK_LEDIRQ	LED_PIN		// LED_PIN, >0 (use GPIO pin) = use debug LED to indicate that we are alive, interval about 1 sec
//#define DISPHSTX_CHECK_LOAD	1	// 1 = check CPU load during IRQ interrupt (get DispHstxTimeIn, DispHstxTimeOut)

 //#define FONT			FontBold8x8	// default system font
 //#define FONTW			8		// width of system font
 //#define FONTH			8		// height of system font

#endif // _CONFIG_H
