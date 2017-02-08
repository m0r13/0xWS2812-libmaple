/* 0xWS2812 16-Channel WS2812 interface library
 * 
 * Copyright (c) 2014 Elia Ritterbusch, http://eliaselectronics.com
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "ws2812.h"

#include <Arduino.h>

/* Array defining 12 color triplets to be displayed */
uint8_t colors[12][3] = 
{
	{0xFF, 0x00, 0x00},
	{0xFF, 0x80, 0x00},
	{0xFF, 0xFF, 0x00},
	{0x80, 0xFF, 0x00},
	{0x00, 0xFF, 0x00},
	{0x00, 0xFF, 0x80},
	{0x00, 0xFF, 0xFF},
	{0x00, 0x80, 0xFF},
	{0x00, 0x00, 0xFF},
	{0x80, 0x00, 0xFF},
	{0xFF, 0x00, 0xFF},
	{0xFF, 0x00, 0x80}
};

void setup()
{	
	delay(5000);

	WS2812_init(TIMER2, DMA1, GPIOA);
}

void setAll(int r, int g, int b) {
	for (size_t i = 0; i < LEDS_PER_ROW; i++) {
		WS2812_framedata_setColumn(16, i, r, g, b);
	}
}

void setTwoColors(int offset, int width, int r1, int g1, int b1, int r2, int g2, int b2) {
	for (size_t i = 0; i < LEDS_PER_ROW; i++) {
		size_t f = (i + offset) / 12;
		if (f % 2) {
			WS2812_framedata_setColumn(16, i, r1, g1, b1);
		} else {
			WS2812_framedata_setColumn(16, i, r2, g2, b2);
		}
	}
}

void loop() {
	Serial.println("loop()");
	unsigned long last = micros();
	while (1){
		// set two pixels (columns) in the defined row (channel 0) to the
		// color values defined in the colors array
		for (int i = 0; /*i < 12*/; i++)
		{
			//Serial.print("loop i=");
			//Serial.println(i);
			// wait until the last frame was transmitted
			//Serial.println("waiting");
			while(!WS2812_TC);
			Serial.println(micros() - last);
			last = micros();
			//delay(20);
			//Serial.println("done");
			// this approach sets each pixel individually
			//WS2812_framedata_setPixel(0, 0, colors[i][0], colors[i][1], colors[i][2]);
			//WS2812_framedata_setPixel(0, 1, colors[i][0], colors[i][1], colors[i][2]);
			// this funtion is a wrapper and achieved the same thing, tidies up the code
			//WS2812_framedata_setRow(0, LEDS_PER_ROW, colors[i][0], colors[i][1], colors[i][2]);
			//WS2812_framedata_setColumn(16, 0, colors[i][0], colors[i][1], colors[i][2]);
			//WS2812_framedata_setColumn(16, 1, colors[i][0], colors[i][1], colors[i][2]);
			//WS2812_framedata_setColumn(16, 2, colors[i][0], colors[i][1], colors[i][2]);
			setTwoColors(i, 12, 0, 255, 0, 0x50, 0x08, 0x54);
			// send the framebuffer out to the LEDs
			WS2812_sendbuf(BUFFER_SIZE);
			// wait some amount of time
			delay(500);
			//Delay(500000L);
		}
	}
}


