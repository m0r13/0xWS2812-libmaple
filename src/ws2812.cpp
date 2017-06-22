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

WS2812_IO_framedata_type WS2812_IO_framedata[BUFFER_SIZE];

volatile uint8_t WS2812_TC = 1;
volatile uint8_t TIM2_overflows = 0;

timer_dev* WS2812_timer = nullptr;
dma_dev* WS2812_dma = nullptr;
gpio_dev* WS2812_gpio = nullptr;
uint32_t WS2812_gpio_mask = 0xffff;

void GPIO_init(void)
{
	ASSERT(WS2812_gpio != NULL);

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

	// pin should be enabled with clk
	// no need to reset gpio dev and disturb other pins
	rcc_clk_enable(WS2812_gpio->clk_id);

	for (int i = 0; i < 16; i++) {
		// set only pins we want to use
		if (WS2812_gpio_mask & (1 << i)) {
			gpio_set_mode(WS2812_gpio, i, GPIO_OUTPUT_PP);
		}
	}
}

void TIM2_IRQHandler(void);

void TIM2_init(void)
{
	ASSERT(WS2812_timer != NULL);

	/*
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	*/

	uint16_t PrescalerValue;
	
	// TIM2 Periph clock enable
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	rcc_clk_enable(WS2812_timer->clk_id);

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

	timer_init(WS2812_timer);
	timer_set_prescaler(WS2812_timer, PrescalerValue);
	// set clock division
	WS2812_timer->regs.gen->CR1 &= ~TIMER_CR1_CKD;
	WS2812_timer->regs.gen->CR1 |= TIMER_CR1_CKD_1TCKINT;
	timer_set_reload(WS2812_timer, 29);

	timer_attach_interrupt(WS2812_timer, TIMER_UPDATE_INTERRUPT, TIM2_IRQHandler);
	// let's suppose counter mode is up by default

	/* Timing Mode configuration: Channel 1 */
	/*
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 8;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);	
	*/

	timer_set_mode(WS2812_timer, TIMER_CH1, TIMER_OUTPUT_COMPARE);
	timer_oc_set_mode(WS2812_timer, TIMER_CH1, TIMER_OC_MODE_FROZEN, 0);
	timer_set_compare(WS2812_timer, TIMER_CH1, 8);


	/* Timing Mode configuration: Channel 2 */
	/*
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 17;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
	*/

	timer_set_mode(WS2812_timer, TIMER_CH2, TIMER_OUTPUT_COMPARE);
	timer_oc_set_mode(WS2812_timer, TIMER_CH2, TIMER_OC_MODE_PWM_1, 0);
	timer_set_compare(WS2812_timer, TIMER_CH2, 17);
	
	/* configure TIM2 interrupt */
	/*
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	timer_enable_irq(WS2812_timer, TIMER_UPDATE_INTERRUPT);
	// TODO priority?
}

void DMA1_Channel7_IRQHandler(void);

void DMA_init(void)
{
	ASSERT(WS2812_dma != NULL);
	ASSERT(WS2812_gpio != NULL);

	/*
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	*/
	
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	rcc_clk_enable(WS2812_dma->clk_id);
	
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

	dma_init(WS2812_dma);
	dma_set_priority(WS2812_dma, DMA_CH2, DMA_PRIORITY_VERY_HIGH);
	
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

	dma_set_priority(WS2812_dma, DMA_CH5, DMA_PRIORITY_VERY_HIGH);

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

	dma_set_priority(WS2812_dma, DMA_CH7, DMA_PRIORITY_VERY_HIGH);

	/* configure DMA1 Channel7 interrupt */
	/*
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	dma_attach_interrupt(WS2812_dma, DMA_CH7, DMA1_Channel7_IRQHandler);

	/* enable DMA1 Channel7 transfer complete interrupt */
	//DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
	
	WS2812_dma->regs->CCR7 |= DMA_CCR_TCIE;
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

	WS2812_dma->regs->IFCR = DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_GIF2 | DMA_ISR_TEIF2
						| DMA_ISR_TCIF5 | DMA_ISR_HTIF5 | DMA_ISR_GIF5 | DMA_ISR_TEIF5
						| DMA_ISR_HTIF7 | DMA_ISR_GIF7 | DMA_ISR_TEIF7;
	
	// configure the number of bytes to be transferred by the DMA controller
	/*
	DMA_SetCurrDataCounter(DMA1_Channel2, buffersize);
	DMA_SetCurrDataCounter(DMA1_Channel5, buffersize);
	DMA_SetCurrDataCounter(DMA1_Channel7, buffersize);
	*/

#if LEDS_GPIO_BYTES == 1
	dma_xfer_size periph_size = DMA_SIZE_8BITS;
	dma_xfer_size memory_size = DMA_SIZE_8BITS;
#elif LEDS_GPIO_BYTES == 2
	dma_xfer_size periph_size = DMA_SIZE_16BITS;
	dma_xfer_size memory_size = DMA_SIZE_16BITS;
#else
#  pragma error Invalid LEDS_GPIO_BYTES.
#endif

	uint32_t mode = DMA_FROM_MEM | DMA_TRNS_CMPLT;

	// set all enabled GPIO pins high
	dma_setup_transfer(
		WS2812_dma, DMA_CH2,
		(__io void*) &WS2812_gpio->regs->BSRR, periph_size,
		(__io void*) &WS2812_gpio_mask, memory_size,
		mode);

	// set all enabled GPIO pins low that have a zero in the stream
	// we write to GPIO reset register:
	// - stream low pins are 1 in framedata -> those pins are reset
	// - pins that aren't enabled are 0 in framedata -> those pins aren't touched
	dma_setup_transfer(
		WS2812_dma, DMA_CH5,
		(__io void*) &WS2812_gpio->regs->BRR, periph_size,
		(__io void*) WS2812_IO_framedata, memory_size,
		mode | DMA_MINC_MODE);

	// set all enabled GPIO pins low
	dma_setup_transfer(
		WS2812_dma, DMA_CH7,
		(__io void*) &WS2812_gpio->regs->BRR, periph_size,
		(__io void*) &WS2812_gpio_mask, memory_size,
		mode);

	dma_set_num_transfers(WS2812_dma, DMA_CH2, buffersize);
	dma_set_num_transfers(WS2812_dma, DMA_CH5, buffersize);
	dma_set_num_transfers(WS2812_dma, DMA_CH7, buffersize);
	
	// clear all TIM2 flags
	/*
	TIM2->SR = 0;
	*/

	WS2812_timer->regs.gen->SR = 0;
	
	// enable the corresponding DMA channels
	/*
	DMA_Cmd(DMA1_Channel2, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	DMA_Cmd(DMA1_Channel7, ENABLE);
	*/

	dma_enable(WS2812_dma, DMA_CH2);
	dma_enable(WS2812_dma, DMA_CH5);
	dma_enable(WS2812_dma, DMA_CH7);

	
	// IMPORTANT: enable the TIM2 DMA requests AFTER enabling the DMA channels!
	/*
	TIM_DMACmd(TIM2, TIM_DMA_CC1, ENABLE);
	TIM_DMACmd(TIM2, TIM_DMA_CC2, ENABLE);
	TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);
	*/
	
	WS2812_timer->regs.gen->DIER &= ~(TIMER_DIER_CC1DE | TIMER_DIER_CC2DE | TIMER_DIER_UDE);
	WS2812_timer->regs.gen->DIER |= TIMER_DIER_CC1DE | TIMER_DIER_CC2DE | TIMER_DIER_UDE;

	// preload counter with 29 so TIM2 generates UEV directly to start DMA transfer
	/*
	TIM_SetCounter(TIM2, 29);
	*/

	timer_set_count(WS2812_timer, 29);
	
	// start TIM2
	/*
	TIM_Cmd(TIM2, ENABLE);
	*/

	timer_resume(WS2812_timer);
}

void WS2812_init(timer_dev* timer, dma_dev* dma, gpio_dev* gpio, uint16_t gpio_mask) {
	ASSERT(timer != NULL);
	ASSERT(dma != NULL);
	ASSERT(gpio != NULL);

	WS2812_timer = timer;
	WS2812_dma = dma;
	WS2812_gpio = gpio;
#if LEDS_GPIO_BYTES == 1
	WS2812_gpio_mask = gpio_mask & 0xff;
#else
	WS2812_gpio_mask = gpio_mask;
#endif
	// set all pins disabled by gpio mask to zero
	// why? see where dma_setup_request is called
	for (int i = 0; i < BUFFER_SIZE; i++) {
		WS2812_IO_framedata[i] = gpio_mask;
	}

	GPIO_init();
	TIM2_init();
	DMA_init();
}

/* DMA1 Channel7 Interrupt Handler gets executed once the complete framebuffer has been transmitted to the LEDs */
void DMA1_Channel7_IRQHandler(void)
{
//	Serial.println("DMA1_Channel7_IRQHandler()");
	dma_irq_cause cause = dma_get_irq_cause(WS2812_dma, DMA_CH7);
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

		WS2812_dma->regs->ISR &= ~DMA_ISR_TCIF7;

		timer_enable_irq(WS2812_timer, TIMER_UPDATE_INTERRUPT);
		
		dma_disable(WS2812_dma, DMA_CH2);
		dma_disable(WS2812_dma, DMA_CH5);
		dma_disable(WS2812_dma, DMA_CH7);
		
		WS2812_timer->regs.gen->DIER &= ~(TIMER_DIER_CC1DE | TIMER_DIER_CC2DE | TIMER_DIER_UDE);
	}
}

/* TIM2 Interrupt Handler gets executed on every TIM2 Update if enabled */
void TIM2_IRQHandler(void)
{
	WS2812_TC = 1; 	
//	Serial.println("TIM2_IRQHandler()");
	// Clear TIM2 Interrupt Flag
	// TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	WS2812_timer->regs.gen->SR &= ~TIMER_SR_TIF;
	
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
		timer_pause(WS2812_timer);
		
		/* disable the TIM2 Update interrupt again 
		 * so it doesn't occur while transmitting data */
		// TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
		
		timer_disable_irq(WS2812_timer, TIMER_UPDATE_INTERRUPT);
		
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
	// skip rows that are disabled by gpio mask
	if ((WS2812_gpio_mask & (1 << row)) == 0) {
		return;
	}

	// invert bits in framedata
	// why? see where dma_setup_request is called
	red = ~red;
	green = ~green;
	blue = ~blue;
	
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		uint8_t red_bit = ((red << i) & 0x80) >> 7;
		uint8_t green_bit = ((green << i) & 0x80) >> 7;
		uint8_t blue_bit = ((blue << i) & 0x80) >> 7;

		if (green_bit) {
			WS2812_IO_framedata[((column*24)+i)] |= (1<<row);
		} else {
			WS2812_IO_framedata[((column*24)+i)] &= ~(0x01<<row);
		}

		if (red_bit) {
			WS2812_IO_framedata[((column*24)+8+i)] |= (1<<row);
		} else {
			WS2812_IO_framedata[((column*24)+8+i)] &= ~(0x01<<row);
		}

		if (blue_bit) {
			WS2812_IO_framedata[((column*24)+16+i)] |= (1<<row);
		} else {
			WS2812_IO_framedata[((column*24)+16+i)] &= ~(0x01<<row);
		}

		/*
		// clear the data for pixel
		WS2812_IO_framedata[((column*24)+i)] &= ~(0x01<<row);
		WS2812_IO_framedata[((column*24)+8+i)] &= ~(0x01<<row);
		WS2812_IO_framedata[((column*24)+16+i)] &= ~(0x01<<row);

		// write new data for pixel
		WS2812_IO_framedata[((column*24)+i)] |= ((((green<<i) & 0x80)>>7)<<row);
		WS2812_IO_framedata[((column*24)+8+i)] |= ((((red<<i) & 0x80)>>7)<<row);
		WS2812_IO_framedata[((column*24)+16+i)] |= ((((blue<<i) & 0x80)>>7)<<row);
		*/
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


