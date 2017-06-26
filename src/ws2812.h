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

#pragma once

#include <cstdint>
#include <libmaple/dma.h>
#include <libmaple/gpio.h>
#include <libmaple/timer.h>

/* this define sets the number of TIM2 overflows
 * to append to the data frame for the LEDs to 
 * load the received data into their registers */
#define WS2812_DEADPERIOD 19

/* WS2812 framebuffer
 * buffersize = (#LEDs / 16) * 24 */
#if defined(WS2812_LED_COUNT)
const int LEDS_PER_ROW = (WS2812_LED_COUNT);
#else
#  pragma error No led count defined (WS2812_LED_COUNT)
#endif
const int BUFFER_SIZE = LEDS_PER_ROW * 24;

/**
 * Width of GPIO port that we want to control:
 * - can be one or two bytes, one byte needs fewer framedata ram
 * - use one byte if you want to address <= 8 led strips
 */
#define LEDS_GPIO_BYTES 1

#if LEDS_GPIO_BYTES == 1
typedef uint8_t WS2812_IO_framedata_type;
#elif LEDS_GPIO_BYTES == 2
typedef uint16_t WS2812_IO_framedata_type;
#else
#  pragma error Invalid LEDS_GPIO_BYTES.
#endif

/**
 * Frame data looks like this:
 * - one can choose from uint8_t / uint16_t GPIO port width
 * - each uint8/16_t is a value assigned to the GPIO port,
 *   from led strip bit stream point of view that's a single bit per strip
 * - so there is 8*uint8/16_t green, then 8*uint8/16_t red, and then 8*uint8/16_t blue
 *   for the first led, then the same for all following ones
 * - to touch only pins that are selected by gpio mask the bits are inverted
 *   in the frame data and bits of pins that are disabled are 0
 */
extern WS2812_IO_framedata_type WS2812_IO_framedata[BUFFER_SIZE];

extern volatile uint8_t WS2812_TC;
extern volatile uint8_t TIM2_overflows;

extern timer_dev* WS2812_timer;
extern dma_dev* WS2812_dma;
extern gpio_dev* WS2812_gpio;
extern uint32_t WS2812_gpio_mask;

void WS2812_init(timer_dev* timer, dma_dev* dma, gpio_dev* gpio, uint16_t gpio_mask);

/* Transmit the frambuffer with buffersize number of bytes to the LEDs 
 * buffersize = (#LEDs / 16) * 24 */
void WS2812_sendbuf(uint32_t buffersize);

/* This function sets the color of a single pixel in the framebuffer 
 * 
 * Arguments:
 * row = the channel number/LED strip the pixel is in from 0 to 15
 * column = the column/LED position in the LED string from 0 to number of LEDs per strip
 * red, green, blue = the RGB color triplet that the pixel should display 
 */
void WS2812_framedata_setPixel(uint8_t row, uint16_t column, uint8_t red, uint8_t green, uint8_t blue);

/* This function is a wrapper function to set all LEDs in the complete row to the specified color
 * 
 * Arguments:
 * row = the channel number/LED strip to set the color of from 0 to 15
 * columns = the number of LEDs in the strip to set to the color from 0 to number of LEDs per strip
 * red, green, blue = the RGB color triplet that the pixels should display 
 */
void WS2812_framedata_setRow(uint8_t row, uint16_t columns, uint8_t red, uint8_t green, uint8_t blue);

/* This function is a wrapper function to set all the LEDs in the column to the specified color
 * 
 * Arguments:
 * rows = the number of channels/LED strips to set the row in from 0 to 15
 * column = the column/LED position in the LED string from 0 to number of LEDs per strip
 * red, green, blue = the RGB color triplet that the pixels should display 
 */
void WS2812_framedata_setColumn(uint8_t rows, uint16_t column, uint8_t red, uint8_t green, uint8_t blue);
