/*****************************************************************************
* | File      	:   epd1in54_V2.cpp
* | Author      :   Waveshare team
* | Function    :   1.54inch e-paper V2
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2019-06-24
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include <stdlib.h>
#include "epd1in54_V2.h"
#include <Arduino.h>

/*
 * Set to 1 to enable driver debug output.
 * Set to 0 to compile out all driver debug output.
 */
#define EPD_DEBUG_ENABLE 0
#ifndef EPD_DEBUG_ENABLE
#define EPD_DEBUG_ENABLE 1
#endif

#if EPD_DEBUG_ENABLE
extern HardwareSerial DebugSerial;

#define EPD_DEBUG_PRINT(...)   DebugSerial.print(__VA_ARGS__)
#define EPD_DEBUG_PRINTLN(...) DebugSerial.println(__VA_ARGS__)
#else
#define EPD_DEBUG_PRINT(...)   do { } while (0)
#define EPD_DEBUG_PRINTLN(...) do { } while (0)
#endif

// waveform full refresh
unsigned char WF_Full_1IN54[159] =
{											
0x80,	0x48,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x40,	0x48,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x80,	0x48,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x40,	0x48,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x8,	0x1,	0x0,	0x8,	0x1,	0x0,	0x2,					
0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x22,	0x22,	0x22,	0x22,	0x22,	0x22,	0x0,	0x0,	0x0,			
0x22,	0x17,	0x41,	0x0,	0x32,	0x20
};

// waveform partial refresh(fast)
unsigned char WF_PARTIAL_1IN54_0[159] =
{
0x0,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x80,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x40,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0xF,0x0,0x0,0x0,0x0,0x0,0x0,
0x1,0x1,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x22,0x22,0x22,0x22,0x22,0x22,0x0,0x0,0x0,
0x02,0x17,0x41,0xB0,0x32,0x28,
};

Epd::~Epd()
{
};

Epd::Epd()
{
	reset_pin = RST_PIN;
	dc_pin = DC_PIN;
	cs_pin = CS_PIN;
	busy_pin = BUSY_PIN;
	width = EPD_WIDTH;
	height = EPD_HEIGHT;
};

/**
 *  @brief: basic function for sending commands
 */
void Epd::SendCommand(unsigned char command)
{
	DigitalWrite(dc_pin, LOW);
	SpiTransfer(command);
}

/**
 *  @brief: basic function for sending data
 */
void Epd::SendData(unsigned char data)
{
	DigitalWrite(dc_pin, HIGH);
	SpiTransfer(data);
}

/**
 *  @brief: Wait until the busy_pin goes HIGH
 */
void Epd::WaitUntilIdle(void)
{
    static uint8_t waitCount = 0;
    waitCount++;

    EPD_DEBUG_PRINT("[EPD] WaitUntilIdle #");
    EPD_DEBUG_PRINT(waitCount);
    EPD_DEBUG_PRINT(" initial BUSY=");
    EPD_DEBUG_PRINTLN(DigitalRead(busy_pin));

    const unsigned long startTime = millis();
    unsigned long lastPrintTime = startTime;

    while (DigitalRead(busy_pin) == HIGH) {
        const unsigned long now = millis();

        if (now - lastPrintTime >= 500) {
            lastPrintTime = now;

            EPD_DEBUG_PRINT("[EPD] BUSY still HIGH, elapsed=");
            EPD_DEBUG_PRINT(now - startTime);
            EPD_DEBUG_PRINTLN(" ms");
        }

        if (now - startTime >= 5000) {
            EPD_DEBUG_PRINT("[EPD] ERROR: BUSY timeout at wait #");
            EPD_DEBUG_PRINTLN(waitCount);
            return;
        }

        DelayMs(10);
    }

    EPD_DEBUG_PRINT("[EPD] BUSY became LOW at wait #");
    EPD_DEBUG_PRINTLN(waitCount);

    DelayMs(200);
}

void Epd::Lut(unsigned char* lut)
{
	SendCommand(0x32);
	for(unsigned char i=0; i<153; i++)
		SendData(lut[i]);
	WaitUntilIdle();
}

void Epd::SetLut(unsigned char* lut)
{
	Lut(lut);
	
    SendCommand(0x3f);
    SendData(lut[153]);
	
    SendCommand(0x03);
    SendData(lut[154]);
	
    SendCommand(0x04);
    SendData(lut[155]);
	SendData(lut[156]);
	SendData(lut[157]);
	
	SendCommand(0x2c);
    SendData(lut[158]);
}

// High Direction
int Epd::HDirInit(void)
{
    EPD_DEBUG_PRINTLN("[EPD] HDirInit start");

	/* this calls the peripheral hardware interface, see epdif */
	if (IfInit() != 0) {
		return -1;
	}
	/* EPD hardware init start */
	Reset();

    EPD_DEBUG_PRINTLN("[EPD] Hardware reset complete");
	WaitUntilIdle();

    EPD_DEBUG_PRINTLN("[EPD] Send SWRESET 0x12");
	SendCommand(0x12);  //SWRESET
	WaitUntilIdle();
    EPD_DEBUG_PRINTLN("[EPD] SWRESET complete");

	SendCommand(0x01); //Driver output control
	SendData(0xC7);
	SendData(0x00);
	SendData(0x01);

	SendCommand(0x11); //data entry mode
	SendData(0x01);

	SendCommand(0x44); //set Ram-X address start/end position
	SendData(0x00);
	SendData(0x18);    //0x0C-->(18+1)*8=200

	SendCommand(0x45); //set Ram-Y address start/end position
	SendData(0xC7);   //0xC7-->(199+1)=200
	SendData(0x00);
	SendData(0x00);
	SendData(0x00);

	SendCommand(0x3C); //BorderWavefrom
	SendData(0x01);

	SendCommand(0x18);
	SendData(0x80);

	SendCommand(0x22); // //Load Temperature and waveform setting.
	SendData(0XB1);
	SendCommand(0x20);

	SendCommand(0x4E);   // set RAM x address count to 0;
	SendData(0x00);
	SendCommand(0x4F);   // set RAM y address count to 0X199;
	SendData(0xC7);
	SendData(0x00);
	WaitUntilIdle();

	SetLut(WF_Full_1IN54);
	/* EPD hardware init end */

    EPD_DEBUG_PRINTLN("[EPD] HDirInit complete");
	return 0;
}

// Low Direction
int Epd::LDirInit(void)
{
    EPD_DEBUG_PRINTLN("[EPD] LDirInit start");

	/* this calls the peripheral hardware interface, see epdif */
	if (IfInit() != 0) {
		return -1;
	}
	/* EPD hardware init start */
	Reset();

    EPD_DEBUG_PRINTLN("[EPD] Hardware reset complete");
	WaitUntilIdle();

    EPD_DEBUG_PRINTLN("[EPD] Send SWRESET 0x12");
	SendCommand(0x12);  //SWRESET
	WaitUntilIdle();
    EPD_DEBUG_PRINTLN("[EPD] SWRESET complete");

	SendCommand(0x01); //Driver output control
	SendData(0xC7);
	SendData(0x00);
	SendData(0x00);

	SendCommand(0x11); //data entry mode
	SendData(0x03);

  SendCommand(0x44);
  /* x point must be the multiple of 8 or the last 3 bits will be ignored */
  SendData((0 >> 3) & 0xFF);
  SendData((199 >> 3) & 0xFF);
  SendCommand(0x45);
  SendData(0 & 0xFF);
  SendData((0 >> 8) & 0xFF);
  SendData(199 & 0xFF);
  SendData((199 >> 8) & 0xFF);

	SendCommand(0x3C); //BorderWavefrom
	SendData(0x01);

	SendCommand(0x18);
	SendData(0x80);

	SendCommand(0x22); // //Load Temperature and waveform setting.
	SendData(0XB1);
	SendCommand(0x20);

	SendCommand(0x4E);   // set RAM x address count to 0;
	SendData(0x00);
	SendCommand(0x4F);   // set RAM y address count to 0X199;
	SendData(0xC7);
	SendData(0x00);
	WaitUntilIdle();

	SetLut(WF_Full_1IN54);
	/* EPD hardware init end */

    EPD_DEBUG_PRINTLN("[EPD] LDirInit complete");
	return 0;
}


/**
 *  @brief: module reset.
 *          often used to awaken the module in deep sleep,
 *          see Epd::Sleep();
 */
void Epd::Reset(void)
{
	DigitalWrite(reset_pin, HIGH);
	DelayMs(20);
	DigitalWrite(reset_pin, LOW);                //module reset
	DelayMs(5);
	DigitalWrite(reset_pin, HIGH);
	DelayMs(20);
}

void Epd::Clear(void)
{
	int w, h;
	w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	h = EPD_HEIGHT;
 
	SendCommand(0x24);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			SendData(0xff);
		}
	}
	SendCommand(0x26);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			SendData(0xff);
		}
	}
	//DISPLAY REFRESH
	DisplayFrame();
}

void Epd::Display(const unsigned char* frame_buffer)
{
	int w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	int h = EPD_HEIGHT;

	if (frame_buffer != NULL) {
		SendCommand(0x24);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}
	}

	//DISPLAY REFRESH
	DisplayFrame();
}

void Epd::DisplayPartBaseImage(const unsigned char* frame_buffer)
{
	int w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	int h = EPD_HEIGHT;

	if (frame_buffer != NULL) {
		SendCommand(0x24);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}

		SendCommand(0x26);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}
	}

	//DISPLAY REFRESH
	DisplayFrame();
}
void Epd::DisplayPartBaseWhiteImage(void)
{
	int w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	int h = EPD_HEIGHT;

	SendCommand(0x24);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			SendData(0xff);
		}
	}

	SendCommand(0x26);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			SendData(0xff);
		}
	}


	//DISPLAY REFRESH
	DisplayFrame();
}


void Epd::DisplayPart(const unsigned char* frame_buffer)
{
	int w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	int h = EPD_HEIGHT;

	if (frame_buffer != NULL) {
		SendCommand(0x24);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}
	}

	//DISPLAY REFRESH
	DisplayPartFrame();
}


/**
 *  @brief: private function to specify the memory area for data R/W
 */
void Epd::SetMemoryArea(int x_start, int y_start, int x_end, int y_end)
{
	SendCommand(0x44);
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	SendData((x_start >> 3) & 0xFF);
	SendData((x_end >> 3) & 0xFF);
	SendCommand(0x45);
	SendData(y_start & 0xFF);
	SendData((y_start >> 8) & 0xFF);
	SendData(y_end & 0xFF);
	SendData((y_end >> 8) & 0xFF);
}

/**
 *  @brief: private function to specify the start point for data R/W
 */
void Epd::SetMemoryPointer(int x, int y)
{
	SendCommand(0x4e);
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	SendData((x >> 3) & 0xFF);
	SendCommand(0x4F);
	SendData(y & 0xFF);
	SendData((y >> 8) & 0xFF);
	WaitUntilIdle();
}


/**
 *  @brief: update the display
 *          there are 2 memory areas embedded in the e-paper display
 *          but once this function is called,
 *          the the next action of SetFrameMemory or ClearFrame will
 *          set the other memory area.
 */
void Epd::DisplayFrame(void)
{
    EPD_DEBUG_PRINTLN("[EPD] DisplayFrame start");

	//DISPLAY REFRESH
	SendCommand(0x22);
	SendData(0xc7);
	SendCommand(0x20);
	WaitUntilIdle();

    EPD_DEBUG_PRINTLN("[EPD] DisplayFrame complete");
}

void Epd::DisplayPartFrame(void)
{
    EPD_DEBUG_PRINTLN("[EPD] DisplayPartFrame start");

	SendCommand(0x22);
	SendData(0xcF);
	SendCommand(0x20);
	WaitUntilIdle();

    EPD_DEBUG_PRINTLN("[EPD] DisplayPartFrame complete");
}


void Epd::SetFrameMemory(
        const unsigned char* image_buffer,
        int x,
        int y,
        int image_width,
        int image_height
)
{
	int x_end;
	int y_end;
	
	DigitalWrite(reset_pin, LOW);                //module reset
	DelayMs(2);
	DigitalWrite(reset_pin, HIGH);
	DelayMs(2);
	SendCommand(0x3c);
	SendData(0x80);

	if (
	        image_buffer == NULL ||
	        x < 0 || image_width < 0 ||
	        y < 0 || image_height < 0
	) {
		return;
	}
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	x &= 0xF8;
	image_width &= 0xF8;
	if (x + image_width >= this->width) {
		x_end = this->width - 1;
	} else {
		x_end = x + image_width - 1;
	}
	if (y + image_height >= this->height) {
		y_end = this->height - 1;
	} else {
		y_end = y + image_height - 1;
	}
	SetMemoryArea(x, y, x_end, y_end);
	SetMemoryPointer(x, y);
	SendCommand(0x24);
	/* send the image data */
	for (int j = 0; j < y_end - y + 1; j++) {
		for (int i = 0; i < (x_end - x + 1) / 8; i++) {
			SendData(image_buffer[i + j * (image_width / 8)]);
		}
	}
}

void Epd::SetFrameMemoryPartial(
        const unsigned char* image_buffer,
        int x,
        int y,
        int image_width,
        int image_height
)
{
	int x_end;
	int y_end;
	
	DigitalWrite(reset_pin, LOW);                //module reset
	DelayMs(2);
	DigitalWrite(reset_pin, HIGH);
	DelayMs(2);

	SetLut(WF_PARTIAL_1IN54_0);
    SendCommand(0x37); 
    SendData(0x00);  
    SendData(0x00);  
    SendData(0x00);  
    SendData(0x00); 
    SendData(0x00);  	
    SendData(0x40);  
    SendData(0x00);  
    SendData(0x00);   
    SendData(0x00);  
    SendData(0x00);

	SendCommand(0x3c);
	SendData(0x80);

	SendCommand(0x22); 
	SendData(0xc0); 
	SendCommand(0x20); 
	WaitUntilIdle();
	
	if (
	        image_buffer == NULL ||
	        x < 0 || image_width < 0 ||
	        y < 0 || image_height < 0
	) {
		return;
	}
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	x &= 0xF8;
	image_width &= 0xF8;
	if (x + image_width >= this->width) {
		x_end = this->width - 1;
	} else {
		x_end = x + image_width - 1;
	}
	if (y + image_height >= this->height) {
		y_end = this->height - 1;
	} else {
		y_end = y + image_height - 1;
	}
	SetMemoryArea(x, y, x_end, y_end);
	SetMemoryPointer(x, y);
	SendCommand(0x24);
	/* send the image data */
	for (int j = 0; j < y_end - y + 1; j++) {
		for (int i = 0; i < (x_end - x + 1) / 8; i++) {
			SendData(image_buffer[i + j * (image_width / 8)]);
		}
	}
}

/**
 *  @brief: Enter partial-refresh mode ONCE. Mirrors the header of
 *          SetFrameMemoryPartial (reset + partial LUT + option + activation) but
 *          performs no RAM write, so it can be called a single time before a run
 *          of PartialWindowFast() updates.
 */
void Epd::PartialModeStart(void)
{
	DigitalWrite(reset_pin, LOW);                //module reset
	DelayMs(2);
	DigitalWrite(reset_pin, HIGH);
	DelayMs(2);

	SetLut(WF_PARTIAL_1IN54_0);
	SendCommand(0x37);
	SendData(0x00);
	SendData(0x00);
	SendData(0x00);
	SendData(0x00);
	SendData(0x00);
	SendData(0x40);
	SendData(0x00);
	SendData(0x00);
	SendData(0x00);
	SendData(0x00);

	SendCommand(0x3c);
	SendData(0x80);

	SendCommand(0x22);
	SendData(0xc0);
	SendCommand(0x20);
	WaitUntilIdle();
}

/**
 *  @brief: Write one window into RAM and do a partial refresh, reusing the LUT
 *          already loaded by PartialModeStart(). No reset, no LUT reload, and a
 *          short busy-poll (no 200 ms settle) -- so each call costs only the
 *          panel's real partial-refresh time.
 */
void Epd::PartialWindowFast(
        const unsigned char* image_buffer,
        int x,
        int y,
        int image_width,
        int image_height
)
{
	int x_end;
	int y_end;

	if (
	        image_buffer == NULL ||
	        x < 0 || image_width < 0 ||
	        y < 0 || image_height < 0
	) {
		return;
	}
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	x &= 0xF8;
	image_width &= 0xF8;
	if (x + image_width >= (int)this->width) {
		x_end = this->width - 1;
	} else {
		x_end = x + image_width - 1;
	}
	if (y + image_height >= (int)this->height) {
		y_end = this->height - 1;
	} else {
		y_end = y + image_height - 1;
	}
	/* HDirInit puts the panel in Y-DECREMENT data-entry mode: the full-frame path
	 * writes user row 0 into RAM-Y (height-1) and the gate scan flips it upright,
	 * so user row r maps to RAM-Y (height-1 - r). Mirror Y here too -- otherwise
	 * the window counts up from the bottom of the panel and wraps past the edge
	 * (why a top-left rect showed at the bottom / split across both edges). */
	int ry_top = (int)this->height - 1 - y;          // RAM-Y of the window's TOP row
	int ry_bot = (int)this->height - 1 - y_end;       // RAM-Y of its BOTTOM row

	SendCommand(0x44);                                // RAM-X window (byte units)
	SendData((x >> 3) & 0xFF);
	SendData((x_end >> 3) & 0xFF);
	SendCommand(0x45);                                // RAM-Y window, high->low (decrement)
	SendData(ry_top & 0xFF);
	SendData((ry_top >> 8) & 0xFF);
	SendData(ry_bot & 0xFF);
	SendData((ry_bot >> 8) & 0xFF);
	/* set the RAM address counter to the top row WITHOUT SetMemoryPointer's 200 ms wait */
	SendCommand(0x4e);
	SendData((x >> 3) & 0xFF);
	SendCommand(0x4F);
	SendData(ry_top & 0xFF);
	SendData((ry_top >> 8) & 0xFF);

	SendCommand(0x24);
	for (int j = 0; j < y_end - y + 1; j++) {
		for (int i = 0; i < (x_end - x + 1) / 8; i++) {
			SendData(image_buffer[i + j * (image_width / 8)]);
		}
	}

	/* trigger the partial refresh and busy-poll with only a tiny settle delay */
	SendCommand(0x22);
	SendData(0xcF);
	SendCommand(0x20);
	const unsigned long start = millis();
	while (DigitalRead(busy_pin) == HIGH) {
		if (millis() - start >= 4000) break;
		DelayMs(1);
	}
	DelayMs(5);
}

void Epd::PartialFullFast(const unsigned char* frame_buffer)
{
	if (frame_buffer == NULL) return;

	/* Restore the FULL-frame RAM window and the Y-decrement mapping that HDirInit
	 * uses (user row 0 -> RAM-Y height-1), so the whole framebuffer is written and
	 * displays upright. PartialWindowFast may have left a small window set. */
	SendCommand(0x44);                                // RAM-X: 0 .. width/8-1 (bytes)
	SendData(0x00);
	SendData((unsigned char)((this->width / 8 - 1) & 0xFF));
	SendCommand(0x45);                                // RAM-Y: (height-1) down to 0
	SendData((unsigned char)((this->height - 1) & 0xFF));
	SendData((unsigned char)(((this->height - 1) >> 8) & 0xFF));
	SendData(0x00);
	SendData(0x00);
	SendCommand(0x4e);                                // X counter = 0
	SendData(0x00);
	SendCommand(0x4f);                                // Y counter = height-1 (top row)
	SendData((unsigned char)((this->height - 1) & 0xFF));
	SendData((unsigned char)(((this->height - 1) >> 8) & 0xFF));

	int w = (this->width % 8 == 0) ? (this->width / 8) : (this->width / 8 + 1);
	int h = this->height;
	SendCommand(0x24);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			SendData(frame_buffer[i + j * w]);
		}
	}

	/* trigger the partial refresh and busy-poll with only a tiny settle delay */
	SendCommand(0x22);
	SendData(0xcF);
	SendCommand(0x20);
	const unsigned long start = millis();
	while (DigitalRead(busy_pin) == HIGH) {
		if (millis() - start >= 4000) break;
		DelayMs(1);
	}
	DelayMs(5);
}

/**
 *  @brief: After this command is transmitted, the chip would enter the
 *          deep-sleep mode to save power.
 *          The deep sleep mode would return to standby by hardware reset.
 *          The only one parameter is a check code, the command would be
 *          executed if check code = 0xA5.
 *          You can use Epd::Init() to awaken
 */
void Epd::Sleep()
{
	SendCommand(0x10); //enter deep sleep
	SendData(0x01);
	DelayMs(200);

	DigitalWrite(reset_pin, LOW);
}

/* END OF FILE */
