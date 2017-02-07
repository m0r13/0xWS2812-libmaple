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

//#include <stm32f10x.h>

#include <libmaple/dma.h>
#include <libmaple/gpio.h>
#include <libmaple/timer.h>

#include <Arduino.h>

#include <cstdint>

/* this define sets the number of TIM2 overflows
 * to append to the data frame for the LEDs to 
 * load the received data into their registers */
#define WS2812_DEADPERIOD 19

uint16_t WS2812_IO_High = 0xFFFF;
uint16_t WS2812_IO_Low = 0x0000;

volatile uint8_t WS2812_TC = 1;
volatile uint8_t TIM2_overflows = 0;

/* WS2812 framebuffer
 * buffersize = (#LEDs / 16) * 24 */
const int LEDS_PER_ROW = 30 * 5;
const int BUFFER_SIZE = LEDS_PER_ROW * 24;
uint16_t WS2812_IO_framedata[BUFFER_SIZE];

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

/* simple delay counter to waste time, don't rely on for accurate timing */
void Delay(uint32_t nCount) {
  while(nCount--) {
  }
}

void GPIO_init(void)
{
	/*
	GPIO_InitTypeDef GPIO_InitStructure;
	// GPIOA Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// GPIOA pins WS2812 data outputs
	GPIO_InitStructure.GPIO_Pin = 0xFFFF;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	*/

	rcc_clk_enable(RCC_GPIOA);

	gpio_init(GPIOA);
	// TODO pin
//	gpio_set_mode(GPIOA, 0, GPIO_OUTPUT_PP);
	for (int i = 0; i < 16; i++) {
		gpio_set_mode(GPIOA, i, GPIO_OUTPUT_PP);
	}
}

void TIM2_IRQHandler(void);

void TIM2_init(void)
{
	/*
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	*/

	timer_dev* timer = TIMER2;
	
	uint16_t PrescalerValue;
	
	// TIM2 Periph clock enable
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	rcc_clk_enable(RCC_TIMER2);

	uint64_t SystemCoreClock = 72000000;
	PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	/* Time base configuration */
	/*
	TIM_TimeBaseStructure.TIM_Period = 29; // 800kHz
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	*/

	//TIM_ARRPreloadConfig(TIM2, DISABLE);
	
	// autoreload = period
	// pulse = compare

	timer_init(timer);
	timer_set_prescaler(timer, PrescalerValue);
	// set clock division
	timer->regs.gen->CR1 &= ~TIMER_CR1_CKD;
	timer->regs.gen->CR1 |= TIMER_CR1_CKD_1TCKINT;
	timer_set_reload(timer, 29);

	timer_attach_interrupt(TIMER2, TIMER_UPDATE_INTERRUPT, TIM2_IRQHandler);
	// let's suppose counter mode is up by default

	/* Timing Mode configuration: Channel 1 */
	/*
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 8;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);	
	*/

	timer_set_mode(timer, TIMER_CH1, TIMER_OUTPUT_COMPARE);
	timer_oc_set_mode(timer, TIMER_CH1, TIMER_OC_MODE_FROZEN, 0);
	timer_set_compare(timer, TIMER_CH1, 8);


	/* Timing Mode configuration: Channel 2 */
	/*
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 17;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
	*/

	timer_set_mode(timer, TIMER_CH2, TIMER_OUTPUT_COMPARE);
	timer_oc_set_mode(timer, TIMER_CH2, TIMER_OC_MODE_PWM_1, 0);
	timer_set_compare(timer, TIMER_CH2, 17);
	
	/* configure TIM2 interrupt */
	/*
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	timer_enable_irq(timer, TIMER_UPDATE_INTERRUPT);
	// TODO priority?
}

void DMA1_Channel7_IRQHandler(void);

void DMA_init(void)
{
	/*
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	*/
	
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	rcc_clk_enable(RCC_DMA1);
	
	// TIM2 Update event
	/* DMA1 Channel2 configuration ----------------------------------------------*/
	/*
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->ODR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)WS2812_IO_High;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	*/

	dma_dev* mdma = DMA1;
	dma_init(mdma);
	dma_set_per_addr(mdma, DMA_CH2, (volatile void*) &GPIOA->regs->ODR);
	dma_set_mem_addr(mdma, DMA_CH2, (volatile void*) WS2812_IO_High);
	dma_set_priority(mdma, DMA_CH2, DMA_PRIORITY_VERY_HIGH);
	
	// TIM2 CC1 event
	/* DMA1 Channel5 configuration ----------------------------------------------*/
	/*
	DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->ODR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)WS2812_IO_framedata;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	*/

	dma_set_per_addr(mdma, DMA_CH5, (volatile void*) &GPIOA->regs->ODR);
	dma_set_mem_addr(mdma, DMA_CH5, (volatile void*) WS2812_IO_framedata);
	dma_set_priority(mdma, DMA_CH5, DMA_PRIORITY_VERY_HIGH);

	// TIM2 CC2 event
	/* DMA1 Channel7 configuration ----------------------------------------------*/
	/*
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->ODR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)WS2812_IO_Low;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);
	*/

	dma_set_per_addr(mdma, DMA_CH7, (volatile void*) &GPIOA->regs->ODR);
	dma_set_mem_addr(mdma, DMA_CH7, (volatile void*) WS2812_IO_Low);
	dma_set_priority(mdma, DMA_CH7, DMA_PRIORITY_VERY_HIGH);

	/* configure DMA1 Channel7 interrupt */
	/*
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	dma_attach_interrupt(mdma, DMA_CH7, DMA1_Channel7_IRQHandler);

	/* enable DMA1 Channel7 transfer complete interrupt */
	//DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
	
	mdma->regs->CCR7 |= DMA_CCR_TCIE;
}

/* Transmit the frambuffer with buffersize number of bytes to the LEDs 
 * buffersize = (#LEDs / 16) * 24 */
void WS2812_sendbuf(uint32_t buffersize)
{		
	// transmission complete flag, indicate that transmission is taking place
	WS2812_TC = 0;
	
	// clear all relevant DMA flags
	/*
	DMA_ClearFlag(DMA1_FLAG_TC2 | DMA1_FLAG_HT2 | DMA1_FLAG_GL2 | DMA1_FLAG_TE2);
	DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_HT5 | DMA1_FLAG_GL5 | DMA1_FLAG_TE5);
	DMA_ClearFlag(DMA1_FLAG_HT7 | DMA1_FLAG_GL7 | DMA1_FLAG_TE7);
	*/

	DMA1->regs->IFCR = DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_GIF2 | DMA_ISR_TEIF2
						| DMA_ISR_TCIF5 | DMA_ISR_HTIF5 | DMA_ISR_GIF5 | DMA_ISR_TEIF5
						| DMA_ISR_HTIF7 | DMA_ISR_GIF7 | DMA_ISR_TEIF7;
	
	// configure the number of bytes to be transferred by the DMA controller
	/*
	DMA_SetCurrDataCounter(DMA1_Channel2, buffersize);
	DMA_SetCurrDataCounter(DMA1_Channel5, buffersize);
	DMA_SetCurrDataCounter(DMA1_Channel7, buffersize);
	*/

	__io void* periph_address = (__io void*) &GPIOA->regs->ODR;
	dma_xfer_size periph_size = DMA_SIZE_32BITS;
	dma_xfer_size memory_size = DMA_SIZE_16BITS;
	uint32_t mode = DMA_FROM_MEM | DMA_TRNS_CMPLT;

	__io void* memory_address = (__io void*) &WS2812_IO_High;
	dma_setup_transfer(DMA1, DMA_CH2, periph_address, periph_size, memory_address, memory_size, mode);

	memory_address = (__io void*) WS2812_IO_framedata;
	dma_setup_transfer(DMA1, DMA_CH5, periph_address, periph_size, memory_address, memory_size, mode | DMA_MINC_MODE);

	memory_address = (__io void*) &WS2812_IO_Low;
	dma_setup_transfer(DMA1, DMA_CH7, periph_address, periph_size, memory_address, memory_size, mode);

	dma_set_num_transfers(DMA1, DMA_CH2, buffersize);
	dma_set_num_transfers(DMA1, DMA_CH5, buffersize);
	dma_set_num_transfers(DMA1, DMA_CH7, buffersize);
	
	// clear all TIM2 flags
	/*
	TIM2->SR = 0;
	*/

	TIMER2->regs.gen->SR = 0;
	
	// enable the corresponding DMA channels
	/*
	DMA_Cmd(DMA1_Channel2, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	DMA_Cmd(DMA1_Channel7, ENABLE);
	*/

	dma_enable(DMA1, DMA_CH2);
	dma_enable(DMA1, DMA_CH5);
	dma_enable(DMA1, DMA_CH7);

	
	// IMPORTANT: enable the TIM2 DMA requests AFTER enabling the DMA channels!
	/*
	TIM_DMACmd(TIM2, TIM_DMA_CC1, ENABLE);
	TIM_DMACmd(TIM2, TIM_DMA_CC2, ENABLE);
	TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);
	*/
	
	TIMER2->regs.gen->DIER &= ~(TIMER_DIER_CC1DE | TIMER_DIER_CC2DE | TIMER_DIER_UDE);
	TIMER2->regs.gen->DIER |= TIMER_DIER_CC1DE | TIMER_DIER_CC2DE | TIMER_DIER_UDE;

	// preload counter with 29 so TIM2 generates UEV directly to start DMA transfer
	/*
	TIM_SetCounter(TIM2, 29);
	*/

	timer_set_count(TIMER2, 29);
	
	// start TIM2
	/*
	TIM_Cmd(TIM2, ENABLE);
	*/

	timer_resume(TIMER2);
}

/* DMA1 Channel7 Interrupt Handler gets executed once the complete framebuffer has been transmitted to the LEDs */
void DMA1_Channel7_IRQHandler(void)
{
//	Serial.println("DMA1_Channel7_IRQHandler()");
	dma_irq_cause cause = dma_get_irq_cause(DMA1, DMA_CH7);
	ASSERT(cause == DMA_TRANSFER_COMPLETE);
	if (cause == DMA_TRANSFER_COMPLETE) {
		/*
		// clear DMA7 transfer complete interrupt flag
		DMA_ClearITPendingBit(DMA1_IT_TC7);	
		// enable TIM2 Update interrupt to append 50us dead period
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		// disable the DMA channels
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		DMA_Cmd(DMA1_Channel7, DISABLE);
		// IMPORTANT: disable the DMA requests, too!
		TIM_DMACmd(TIM2, TIM_DMA_CC1, DISABLE);
		TIM_DMACmd(TIM2, TIM_DMA_CC2, DISABLE);
		TIM_DMACmd(TIM2, TIM_DMA_Update, DISABLE);
		*/

		DMA1->regs->ISR &= ~DMA_ISR_TCIF7;

		timer_enable_irq(TIMER2, TIMER_UPDATE_INTERRUPT);
		
		dma_disable(DMA1, DMA_CH2);
		dma_disable(DMA1, DMA_CH5);
		dma_disable(DMA1, DMA_CH7);
		
		TIMER2->regs.gen->DIER &= ~(TIMER_DIER_CC1DE | TIMER_DIER_CC2DE | TIMER_DIER_UDE);
	}
}

/* TIM2 Interrupt Handler gets executed on every TIM2 Update if enabled */
void TIM2_IRQHandler(void)
{
	WS2812_TC = 1; 	
//	Serial.println("TIM2_IRQHandler()");
	// Clear TIM2 Interrupt Flag
	// TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	TIMER2->regs.gen->SR &= ~TIMER_SR_TIF;
	
	/* check if certain number of overflows has occured yet 
	 * this ISR is used to guarantee a 50us dead time on the data lines
	 * before another frame is transmitted */
	if (TIM2_overflows < (uint8_t)WS2812_DEADPERIOD)
	{
//		Serial.println("TIM2_IRQHandler::firstBranch");
		// count the number of occured overflows
		TIM2_overflows++;
	}
	else
	{
//		Serial.println("TIM2_IRQHandler::secondBranch");
		// clear the number of overflows
		TIM2_overflows = 0;	
		// stop TIM2 now because dead period has been reached
		// TIM_Cmd(TIM2, DISABLE);
		timer_pause(TIMER2);
		
		/* disable the TIM2 Update interrupt again 
		 * so it doesn't occur while transmitting data */
		// TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
		
		timer_disable_irq(TIMER2, TIMER_UPDATE_INTERRUPT);
		
		// finally indicate that the data frame has been transmitted
		WS2812_TC = 1; 	
	}
}

/* This function sets the color of a single pixel in the framebuffer 
 * 
 * Arguments:
 * row = the channel number/LED strip the pixel is in from 0 to 15
 * column = the column/LED position in the LED string from 0 to number of LEDs per strip
 * red, green, blue = the RGB color triplet that the pixel should display 
 */
void WS2812_framedata_setPixel(uint8_t row, uint16_t column, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		// clear the data for pixel 
		WS2812_IO_framedata[((column*24)+i)] &= ~(0x01<<row);
		WS2812_IO_framedata[((column*24)+8+i)] &= ~(0x01<<row);
		WS2812_IO_framedata[((column*24)+16+i)] &= ~(0x01<<row);
		// write new data for pixel
		WS2812_IO_framedata[((column*24)+i)] |= ((((green<<i) & 0x80)>>7)<<row);
		WS2812_IO_framedata[((column*24)+8+i)] |= ((((red<<i) & 0x80)>>7)<<row);
		WS2812_IO_framedata[((column*24)+16+i)] |= ((((blue<<i) & 0x80)>>7)<<row);
	}
}

/* This function is a wrapper function to set all LEDs in the complete row to the specified color
 * 
 * Arguments:
 * row = the channel number/LED strip to set the color of from 0 to 15
 * columns = the number of LEDs in the strip to set to the color from 0 to number of LEDs per strip
 * red, green, blue = the RGB color triplet that the pixels should display 
 */
void WS2812_framedata_setRow(uint8_t row, uint16_t columns, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t i;
	for (i = 0; i < columns; i++)
	{
		WS2812_framedata_setPixel(row, i, red, green, blue);
	}
}

/* This function is a wrapper function to set all the LEDs in the column to the specified color
 * 
 * Arguments:
 * rows = the number of channels/LED strips to set the row in from 0 to 15
 * column = the column/LED position in the LED string from 0 to number of LEDs per strip
 * red, green, blue = the RGB color triplet that the pixels should display 
 */
void WS2812_framedata_setColumn(uint8_t rows, uint16_t column, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t i;
	for (i = 0; i < rows; i++)
	{
		WS2812_framedata_setPixel(i, column, red, green, blue);
	}
}

void setup()
{	
	delay(5000);

	Serial.println("GPIO init...");
	GPIO_init();
	Serial.println("DMA init...");
	DMA_init();
	Serial.println("TIM2 init...");
	TIM2_init();
	Serial.println("Done init...");
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
			delay(10);
			//Delay(500000L);
		}
	}
}


