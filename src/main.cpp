
// ****************************************************************************
//
//  DVI (HDMI) and VGA display driver for Pico2 RP2350, using HSTX peripheral
//                                   Demo
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

#include "../include.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <hardware/structs/dma.h>
#include "hardware/dma.h"
#include "hardware/timer.h"

#define DBG_INFO

#define LED_PIN 25

#define DISPVMODEINFO		0	// 1=display videomode info (debug)
#define DISP_FPS 1

// halt on memory error, blinking internal LED on Pico2 board (not on Wifi version)
#define CHECK_ERR() while (res != DISPHSTX_ERR_OK) { GPIO_Flip(LED_PIN); WaitMs(100); }

// Draw box
ALIGNED u8 Box[WWIDTH*HHEIGHT];
ALIGNED u8 FBuf[WWIDTH*HHEIGHT];
ALIGNED u16 MyPal[256];

#define ADC_CHANNEL_1 0
#define ADC_CHANNEL_2 1
#define ADC_CHANNEL_3 2

#define CAPTURE_DEPTH 1024*8
#define CAPTURE_RING_BITS 14

// Aligned for the DMA ring address warp.
uint16_t capture_buf1[CAPTURE_DEPTH] __attribute__((aligned(CAPTURE_DEPTH*2)));
uint16_t capture_buf2[CAPTURE_DEPTH] __attribute__((aligned(CAPTURE_DEPTH*2)));
uint dma_chan1;
uint dma_chan2;
uint chan32;
char buf[200];
uint32_t cntr;
bool new_data=false;
int irq_counter1 = 0;
int irq_counter2 = 0;
uint16_t *data_buf;


#if DISPVMODEINFO	// 1=display videomode info (debug)
// Display videomode info (debug)
void DispVModeInfo()
{
	// Draw border marks (to check image clipping)
	DrawTextBg("[*]", 0, 0, COL_WHITE, COL_BLACK);			// left-top mark
	DrawTextBg("[*]", pDrawCan->w - 3*8, 0, COL_WHITE, COL_BLACK);	// right-top mark
	DrawTextBg("[*]", pDrawCan->w - 3*8, pDrawCan->h - 16, COL_WHITE, COL_BLACK); // right-bottom mark

	// Draw videomode info
	char tt[100];
	MemPrint(tt, 100, "%s %dx%d/%db %dMHz %'d bytes",
		(DispHstxDispMode == DISPHSTX_DISPMODE_VGA) ? "VGA" : "DVI",	// display mode
		pDrawCan->w, pDrawCan->h, pDrawCan->colbit,			// videomode size and bits
		(ClockGetHz(CLK_SYS)+500000)/1000000,				// system clock
		pDrawCan->wb * pDrawCan->h);					// memory size
	DrawTextBg(tt, 0, pDrawCan->h-2*16, COL_GREEN, COL_BLACK);

	MemPrint(tt, 100, "vreg=%.2fV clkdiv=%d tmp=%d`C",
		VregVoltageFloat(), FlashClkDiv(), (int)(ADC_Temp()+0.5));
	DrawTextBg(tt, 0, pDrawCan->h-1*16, COL_GREEN, COL_BLACK);
}
#endif

// Called when capture_buf1 is full.
void  FASTCODE NOFLASH(dma_handler1()) {
	if (dma_channel_get_irq0_status(dma_chan1)) {
		// Do stuff for dma_chan_
		irq_counter1++;
	data_buf = capture_buf1;
  //dma_hw->ints0 = 1u << dma_chan1;
  dma_channel_acknowledge_irq0(dma_chan1);
		new_data = true;
	}
	
	if (dma_channel_get_irq0_status(dma_chan2)) {
		// Do stuff for display_dma_chan
		irq_counter2++;
	data_buf = capture_buf2;
  dma_channel_acknowledge_irq0(dma_chan2);
  //dma_hw->ints0 = 1u << dma_chan2;
		new_data = true;
	}
  // Toggle debug LED.
  // Clear the interrupt request.
}

void FASTCODE NOFLASH (smooth())
{
		for (uint32_t i=WWIDTH; i<(WWIDTH)*(HHEIGHT)-1-WWIDTH; i++)
		{
			if (Box[i]>3) Box[i]=Box[i]-4; else Box[i] = 0;
		}
}

void FASTCODE NOFLASH (bloor())
{
		//u8 *memptr = DispHstxBuf();//pDrawCan->buf;
		//memptr[1]=1;
		//memset(Box,0,WWIDTH*HHEIGHT);

		for (uint32_t i=WWIDTH; i<(WWIDTH)*(HHEIGHT)-1-WWIDTH; i++)
		{
			//Box[i] = (Box[i-1]+Box[i+1]+Box[i+WWIDTH-1]+Box[i+WWIDTH+1]+Box[i-WWIDTH-1]+Box[i-WWIDTH+1]+Box[i+WWIDTH]+Box[i-WWIDTH])/10;
			Box[i] = (Box[i]+Box[i-1]+Box[i+1]+Box[i+WWIDTH]+Box[i-WWIDTH])/6;
		}
		
}

void FASTCODE NOFLASH (DrawDot)(uint16_t x, uint16_t y, uint8_t col)
{
		//u8 *memptr = DispHstxBuf();
		//DrawRound(x,y,2,col,DRAWCAN_ROUND_ALL);
		DrawPoint(x,y,col);
		//Box[x+WWIDTH*y]	=col;
}

int FASTCODE NOFLASH (main())
{
	pll_init(pll_usb, 1, 1500 * MHZ, 6, 2);

    uint32_t adc_clk_freq_hz = clock_get_hz(clk_usb);
    clock_configure(clk_adc, 0, CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, adc_clk_freq_hz, adc_clk_freq_hz);

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	// ==== Initialize videomode 640x480

		// initialize videomode 640x480
		//int res = DispVMode640x480x8_Fast(0, Box);
		//int res = DispVMode640x480x8(0, Box);

		sDispHstxVModeState* v = &DispHstxVMode; // pointer to default descriptor
		DispHstxVModeInitTime(v, &DispHstxVModeTimeList[vmodetime_640x350_fast]);
		memcpy(MyPal, DispHstxPal8b, 256*2);
		for (int i = 0 ; i < 64; i++) 
		{
			MyPal[i] = ((i*32/64) << 11) | (i << 5) | (i*32/64);
			//MyPal[i] =  ((i*32/64) << 11) | (i << 5);
		}
		int res = DispHstxVModeInitSimple(v, &DispHstxVModeTimeList[vmodetime_640x350_fast],1,1,DISPHSTX_FORMAT_8_PAL,FBuf,MyPal,NULL,-1);
		CHECK_ERR();
// activate videomode
		DispHstxSelDispMode(DISPHSTX_DISPMODE_NONE, v);
		//res = DispHstxVModeStartSimple(0, Box, vmodetime_640x480_fast, 1, 1, DISPHSTX_FORMAT_8_PAL);
		//CHECK_ERR();

// -------ADC init
    // Send core 1 off to start driving the "DAC" whilst we configure the ADC.
    //multicore_launch_core1(core1_main);

  adc_gpio_init(26 + ADC_CHANNEL_1);
  adc_gpio_init(26 + ADC_CHANNEL_2);
  adc_gpio_init(26 + ADC_CHANNEL_3);
  adc_gpio_init(26 + 3);
  adc_init();

  // This determines the first channel that will be scanned in
  // each round robin cycle.
  adc_select_input(ADC_CHANNEL_1);
  // Alternating ADC sampling.
  adc_set_round_robin(1 << ADC_CHANNEL_1 | 1 << ADC_CHANNEL_2| 1 << ADC_CHANNEL_3|1<<3);

  adc_fifo_setup(true,  // Write each completed conversion to the sample FIFO
                 true,  // Enable DMA data request (DREQ)
                 1,  // DREQ (and IRQ) asserted when at least 1 sample present
                 false,  // Collect also the error bit.
                 false  // Do not reduce samples to 8 bits.
  );

  // Determines the ADC sampling rate as a divisor of the basic
  // 48Mhz clock. Set to have 100k sps on each of the two ADC
  // channels.
  //(48000000 / frequency) -1.
  //adc_set_clkdiv(240 - 1);  // Total rate 200k sps.
  //adc_set_clkdiv(416-1);  // Total rate 200k sps.
  //adc_set_clkdiv(125-1);  // Total rate 1M sps.

  // --------------- DMA
  dma_chan1 = dma_claim_unused_channel(true);
  dma_chan2 = dma_claim_unused_channel(true);

  // Chan 1
    dma_channel_config dma_config1 = dma_channel_get_default_config(dma_chan1);
    channel_config_set_transfer_data_size(&dma_config1, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config1, false);  // ADC fifo
    channel_config_set_write_increment(&dma_config1, true);  // RAM buffer.
    // channel_config_set_write_increment(&dma_config1, true);
    // Wrap to begining of buffer. Assuming buffer is well alligned.
    channel_config_set_ring(&dma_config1, true, CAPTURE_RING_BITS);
    // Paced by ADC genered requests.
    channel_config_set_dreq(&dma_config1, DREQ_ADC);
    // When done, start the other channel.
    channel_config_set_chain_to(&dma_config1, dma_chan2);
    // Using interrupt channel 0
    dma_channel_set_irq0_enabled(dma_chan1, true);
    // Set IRQ handler.
    //irq_set_exclusive_handler(DMA_IRQ_0, dma_handler1);
	irq_add_shared_handler(DMA_IRQ_0, dma_handler1,0);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_configure(dma_chan1, &dma_config1,
                          capture_buf1,   // dst
                          &adc_hw->fifo,  // src
                          CAPTURE_DEPTH,  // transfer count
                          true            // start immediately
    );

  // Chan 2
    dma_channel_config dma_config2 = dma_channel_get_default_config(dma_chan2);
    channel_config_set_transfer_data_size(&dma_config2, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config2, false);
    channel_config_set_write_increment(&dma_config2, true);
    channel_config_set_ring(&dma_config2, true, CAPTURE_RING_BITS);
    channel_config_set_dreq(&dma_config2, DREQ_ADC);
    // When done, start the other channel.
    channel_config_set_chain_to(&dma_config2, dma_chan1);
    dma_channel_set_irq0_enabled(dma_chan2, true);
    //irq_set_exclusive_handler(DMA_IRQ_1, dma_handler2);
    //irq_set_enabled(DMA_IRQ_1, true);
    dma_channel_configure(dma_chan2, &dma_config2,
                          capture_buf2,   // dst
                          &adc_hw->fifo,  // src
                          CAPTURE_DEPTH,  // transfer count
                          false           // Do not start immediately
    );

	// Prepare DMA for memcpy_dma
	/* 4-byte DMA transfer width */
	chan32 = dma_claim_unused_channel(true);
	dma_channel_config cfg32 = dma_channel_get_default_config(chan32);
	channel_config_set_read_increment(&cfg32, true);
	channel_config_set_write_increment(&cfg32, true);
	channel_config_set_transfer_data_size(&cfg32, DMA_SIZE_32);
	dma_channel_set_config(chan32, &cfg32, false);

  // Start the ADC free run sampling.
  adc_run(true);

	//DrawClear();
#ifdef DBG_INFO
  	// draw text
	DrawFrame(0,0,WWIDTH-1,HHEIGHT-1,COL_WHITE);

	for (int i = 0; i < 256; i++)
		DrawLine(i+180,4,i+180,12,i);

	//DrawLine(WWIDTH/2,0,WWIDTH/2,HHEIGHT-1,COL_LTGREEN); //vert
	//DrawLine(0,HHEIGHT/2,WWIDTH-1,HHEIGHT/2,COL_LTGREEN); //hor
	//DrawText("Scopetrex!", 540,4,COL_WHITE);
	sprintf(buf,"pll_sys %d pll_usb %d rosc %d clk_sys %d",
										frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY),
										frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY),
										frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC),
										frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS));
	DrawTextBg(buf, 10,14,COL_WHITE,COL_BLACK);
	sprintf(buf,"clk_peri %d clk_usb %d ADC_CLK %d",
										frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI),
										frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB),
										frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC));

	DrawTextBg(buf, 10,24,COL_WHITE,COL_BLACK);
	//sleep_ms(8000);
#endif
	cntr=0;

	// main loop
	uint16_t oldx,oldy,dw,dh;
	uint8_t color;
	oldx = WWIDTH/2;
	oldy = HHEIGHT/2;
	dw = 2048/WWIDTH;
	dh = 2048/HHEIGHT;

	pDrawCan->buf = Box;
	data_buf = capture_buf1;

//	v->strip[0].slot[0].buf = Box;
#if DISP_FPS			// 1=display FPS
	u32 t = time_us_32();
	u32 t2;
#endif

	while (True)
	{
		//DispHstxWaitVSync();
#ifdef DBG_INFO
		sprintf(buf,"%d %d %d drop %d",cntr,irq_counter1,irq_counter2,cntr - (irq_counter1+irq_counter2));
		DrawTextBg(buf, 10,4,COL_WHITE,COL_BLACK);
#else
		//DispHstxWaitVSync();
		sleep_ms(1);
#endif
		//dma_channel_wait_for_finish_blocking(dma_chan1);
		if(new_data)
		{
/*				float k = 0.3;
				uint16_t old_valx = data_buf[0];
				uint16_t old_valy = data_buf[1];
				uint16_t old_valz = data_buf[2];
				for(uint16_t i=0; i < CAPTURE_DEPTH; i=i+4)
				{
					old_valx = k * data_buf[i] + (1.0 - k) * old_valx;
					old_valy = k * data_buf[i+1] + (1.0 - k) * old_valy;
					old_valz = k * data_buf[i+2] + (1.0 - k) * old_valz;
					data_buf[i] = old_valx;
					data_buf[i+1] = old_valy;
					data_buf[i+2] = old_valz;
				}*/
				//DispHstxWaitVSync();
				for(uint16_t i=0; i < CAPTURE_DEPTH; i=i+4)
				{
					#if 1
					if (data_buf[i+2] > 512)
						DrawDot(data_buf[i]/dw-WWIDTH/2,data_buf[i+1]/dh-HHEIGHT/2-90,63);
					//else
					//	DrawPoint(capture_buf1[i]/(2048/WWIDTH)-WWIDTH/2,capture_buf1[i+1]/(2048/HHEIGHT)-HHEIGHT/2-70,COL_GRAY4);
					#else
					if (data_buf[i+2] > 512)
						DrawLine(oldx,oldy,data_buf[i]/dw-WWIDTH/2,data_buf[i+1]/dh-HHEIGHT/2-90,63);
					//else
						//DrawLine(oldx,oldy,capture_buf1[i]/(2048/WWIDTH)-WWIDTH/2,capture_buf1[i+1]/(2048/HHEIGHT)-HHEIGHT/2,COL_BLACK);
					oldx = data_buf[i]/dw-WWIDTH/2;
					oldy = data_buf[i+1]/dh-HHEIGHT/2-90;
					#endif
				//draw flat data
				//DrawPoint(i/(CAPTURE_DEPTH/WWIDTH),200+data_buf[i]/(2048/200)-200/2,COL_WHITE);
				}

		cntr++;
				//DispHstxCore1Exec(bloor);
				//DispHstxWaitVSync();
		//memcpy(FBuf,Box,WWIDTH*HHEIGHT);
		dma_channel_set_read_addr(chan32, Box, false);
		dma_channel_set_write_addr(chan32, FBuf, false);
		dma_channel_set_trans_count(chan32, (HHEIGHT*WWIDTH) >> 2, true);
		//dma_channel_start(chan32);
		//dma_channel_wait_for_finish_blocking(chan32);
						//bloor();
		if (cntr % 5 == 0)
		{
						DispHstxCore1Exec(bloor);
						//bloor();
		}
						//DispHstxCore1Wait();
			new_data = false;
#if DISP_FPS			// 1=display FPS
		{
		// FPS
		t2 = time_us_32();
		//MemPrint(buf, 20, "%.2f ", 1000000.0/(t2-t));
		sprintf(buf, "%.2f ", 1000000.0/(t2-t));
		DrawTextBg(buf, 10, 14, COL_WHITE, COL_BLACK);
		//sleep_ms(100);
		t = time_us_32();
		}
#endif
		} //end new_data
		//if (cntr % 8 == 0)
		//{
				//DispHstxCore1Exec(bloor);
				//bloor();
		//}
		// short delay
		//sleep_ms(1);

	}
}
