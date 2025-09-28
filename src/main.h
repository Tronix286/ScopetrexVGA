
// ****************************************************************************
//
//      HDMI and VGA display driver for Pico2 RP2350 over HSTX interface
//                                 Main code
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

#ifndef _MAIN_H
#define _MAIN_H

// screen resolution
#define WWIDTH	640	// screen width
#define HHEIGHT	350	// screen height
#define COL_GRAY0	0
#define COL_GRAY1	(B2+B5)
#define COL_GRAY2	(B0+B3+B6)
#define COL_GRAY3	(B0+B2+B3+B5+B6)
#define COL_GRAY4	(B1+B4+B7)
#define COL_GRAY5	(B1+B2+B4+B5+B7)
#define COL_GRAY6	(B0+B1+B3+B4+B6+B7)
#define COL_GRAY7	(B0+B1+B2+B3+B4+B5+B6+B7)


#endif // _MAIN_H
