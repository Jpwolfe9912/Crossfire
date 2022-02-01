/*
 * crsf.c
 *
 *  Created on: Jan 14, 2022
 *      Author: jeremywolfe
 */

#include "stm32f7xx.h"
#include "drv_crsf.h"

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

static bool usart_read(uint8_t *pData, uint8_t size);

uint16_t rxSize;

uint8_t rxBuf[RxBuf_SIZE];
lwrb_t rxRingBuf;
uint8_t rxRingBufData[RxBuf_SIZE];

uint8_t crsfPayload[PAYLOAD_SIZE];

uint8_t state, cmd, len;

void crsf_init(void){
//	lwrb_init(&rxRingBuf, rxRingBufData, sizeof(rxRingBufData));
	/* ---PIN INFO---
	 * USART3_RX
	 * 		PD9
	 * 		AF7
	 * 		DMA 1
	 * 		Stream 1
	 * 		Channel 4
	 * USART3_RX
	 * 		PD8
	 * 		AF7
	 * 		DMA 1
	 * 		Stream 3
	 * 		Channel 4
	 * */

	/////////////////GPIO INIT///////////////////
	// enable clock for GPIOD
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	// set mode, speed, type, pull, AF
	GPIOD->MODER 	&= ~GPIO_MODER_MODER8;
	GPIOD->MODER 	|= GPIO_MODER_MODER8_1;				// AF mode
	GPIOD->OSPEEDR	|= GPIO_OSPEEDR_OSPEEDR8;			// high speed
	GPIOD->OTYPER	&= ~GPIO_OTYPER_OT8;
	GPIOD->PUPDR	&= ~GPIO_PUPDR_PUPDR8;
	GPIOD->AFR[1]	&= ~GPIO_AFRH_AFRH0;
	GPIOD->AFR[1] 	|= (0x7 << (4U * 0U));				// AF 7

	GPIOD->MODER 	&= ~GPIO_MODER_MODER9;
	GPIOD->MODER 	|= GPIO_MODER_MODER9_1;
	GPIOD->OSPEEDR	|= GPIO_OSPEEDR_OSPEEDR9;
	GPIOD->OTYPER	&= ~GPIO_OTYPER_OT9;
	GPIOD->PUPDR	&= ~GPIO_PUPDR_PUPDR9;
	GPIOD->AFR[1]	&= ~GPIO_AFRH_AFRH1;
	GPIOD->AFR[1] 	|= (0x7 << (4U * 1U));

	// DMA IRQ Init
	NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(USART3_IRQn);

	/////////////////USART INIT///////////////////
	RCC->APB1ENR	|= RCC_APB1ENR_USART3EN;

	USART3->CR1		&= ~USART_CR1_UE;		// disable usart
	USART3->BRR		= 0x1D5;				// 115200 BR
	USART3->CR1		&= ~USART_CR1_M;		// 8 bit transfer
	USART3->CR2		&= ~USART_CR2_STOP;
	USART3->CR1		&= ~USART_CR1_PCE;
	USART3->CR1		|= USART_CR1_RE	|		// enable rx, tx
					   USART_CR1_TE;
	USART3->CR3		&= ~(USART_CR3_CTSE |
						 USART_CR3_RTSE);
	USART3->CR1		&= ~USART_CR1_OVER8;

//	USART3->CR1		|= USART_CR1_IDLEIE;

	/////////////////DMA INIT///////////////////
	RCC->AHB1RSTR	|= RCC_AHB1RSTR_DMA1RST;
	RCC->AHB1RSTR	&= ~RCC_AHB1RSTR_DMA1RST;

	// disable DMA 2 stream 5
	DMA1_Stream1->CR 	&= ~DMA_SxCR_EN;
	while(DMA1_Stream1->CR & DMA_SxCR_EN){}
	DMA1_Stream1->CR	= 0;
	DMA1_Stream1->NDTR	= 0;
	DMA1_Stream1->PAR	= 0;
	DMA1_Stream1->M0AR	= 0;
	DMA1_Stream1->M1AR	= 0;
	DMA1_Stream1->FCR	= 0x00000021U;
	DMA1_Stream1->CR	&= ~DMA_SxCR_CHSEL;
	DMA1->LIFCR			|= (0x3F << 6U); //0x00000F40U;

	RCC->AHB1ENR		|= RCC_AHB1ENR_DMA1EN;

	// stream 5 ch 4 DMA settings
	DMA1_Stream1->CR	|= (0x4 << 25U);			// channel 4
	DMA1_Stream1->M0AR 	= (uint32_t)rxBuf;		// set mem address
	DMA1_Stream1->CR 	&= ~DMA_SxCR_DIR;			// per to mem
	DMA1_Stream1->FCR	&= ~DMA_SxFCR_DMDIS;		// fifo dis
	DMA1_Stream1->CR 	&= ~DMA_SxCR_MBURST;
	DMA1_Stream1->CR 	&= ~DMA_SxCR_PBURST;
	DMA1_Stream1->PAR 	= (uint32_t)(&(USART3->RDR));// set per address
	DMA1_Stream1->NDTR	= CRSF_FRAME_SIZE_MAX;		// 64 bytes NEEDS TO MATCH HOW MANY ARE COMING!!!!!
	DMA1_Stream1->CR 	&= ~DMA_SxCR_PINC;			// don't inc per
	DMA1_Stream1->CR 	|= DMA_SxCR_MINC;			// increment mem
	DMA1_Stream1->CR 	&= ~DMA_SxCR_MSIZE;			// 8 bit size
	DMA1_Stream1->CR 	&= ~DMA_SxCR_PSIZE;			// 8 bit size
	DMA1_Stream1->CR 	|= DMA_SxCR_CIRC;			// circ mode en ***was changed***
	DMA1_Stream1->CR 	&= ~DMA_SxCR_PL_0;			// medium priority

	USART3->CR1			|= USART_CR1_UE;			// enable usart

	lwrb_init(&rxRingBuf, rxRingBufData, sizeof(rxRingBufData));

	usart_read(rxBuf, RxBuf_SIZE);


}

void crsf_process(void){
	uint8_t b;
	static uint8_t lenConst;

	if(lwrb_read(&rxRingBuf, &b, 1)){
		switch(state){
			case 0: {				// start byte
				if(b == 0xC8){
					++state;
				}
				break;
			}
			case 1: {				// length byte
				len = b;
				lenConst = b;
				++state;
				if(len == 0){
					++state;
				}
				break;
			}
			case 2: {				// payload bytes
				crsfPayload[lenConst - len] = b;
				--len;
				if(len == 0){

					++state;
				}
				break;
			}
			case 3: {
				if(b == 0xFF){
					state = 0;
					break;
				}
			}
		}
	}
}


static bool usart_read(uint8_t *pData, uint8_t size){
	rxSize = size;
	if(!(USART3->ISR & USART_ISR_BUSY)){		// wait for UART to be ready
		DMA1_Stream1->CR	&= ~DMA_SxCR_EN;	// disable DMA
		while(DMA1_Stream1->CR & DMA_SxCR_EN);
		DMA1_Stream1->CR	|= (0x4 << 25U);	// set DMA channel
		DMA1_Stream1->NDTR	= rxSize;				// set transfer size
		DMA1_Stream1->M0AR	= (uint32_t)pData;	// set memory address

		DMA1->LIFCR			|= (0x3F << 6U);	// clear flags

		DMA1_Stream1->CR 	|= DMA_SxCR_TCIE;	// set transfer complete interrupts
		DMA1_Stream1->CR	|= DMA_SxCR_HTIE;

		DMA1_Stream1->CR	|= DMA_SxCR_EN;		// enable DMA

		USART3->CR3			|= USART_CR3_DMAR;	// enable DMA for UART

		USART3->ICR			|= USART_ICR_IDLECF;// clear idle interrupt flag
		USART3->CR1			|= USART_CR1_IDLEIE;// enable idle line interrupts

		return true;
	}
	else return false;
}


/**
 * \brief           Check for new data received with DMA
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure
 * data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes,
 * hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
void usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(rxBuf) - (DMA1_Stream1->NDTR);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            usart_process_data(&rxBuf[old_pos], pos - old_pos);
        }
        else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            usart_process_data(&rxBuf[old_pos], ARRAY_LEN(rxBuf) - old_pos);
            if (pos > 0) {
                usart_process_data(&rxBuf[0], pos);
            }
        }
        old_pos = pos;                          /* Save current position as old for next transfers */
    }
}


/**
 * \brief           Process received data over UART
 * Data are written to RX ringbuffer for application processing at latter stage
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void usart_process_data(const void* data, size_t len) {
    lwrb_write(&rxRingBuf, data, len);  /* Write data to receive buffer */
}



/* Interrupt handlers here */

/**
 * \brief           DMA1 stream1 interrupt handler for USART3 RX
 */
void DMA1_Stream1_IRQHandler(void) {
    /* Check half-transfer complete interrupt */
	if(DMA1->LISR & DMA_LISR_TCIF1){
		DMA1->LIFCR		|= DMA_LIFCR_CTCIF1;	/* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
	if(DMA1->LISR & DMA_LISR_HTIF1){
		DMA1->LIFCR		|= DMA_LIFCR_CHTIF1;	/* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
 * \brief           USART3 global interrupt handler
 */
void USART3_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    if (USART3->ISR & USART_ISR_IDLE) {
    	USART3->ICR		|= USART_ICR_IDLECF;	/* Clear IDLE line flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Implement other events when needed */
}



















//void rx_process(uint16_t Size)
//{
//	oldPos = newPos;  // Update the last position before copying new data
//
//	/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
//	 * This is to maintain the circular buffer
//	 * The old data in the main buffer will be overlapped
//	 */
//	if (oldPos+Size > MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
//	{
//		uint16_t datatocopy = MainBuf_SIZE-oldPos;  // find out how much space is left in the main buffer
//		memcpy ((uint8_t *)MainBuf+oldPos, rxBuf, datatocopy);  // copy data in that remaining space
//
//		oldPos = 0;  // point to the start of the buffer
//		memcpy ((uint8_t *)MainBuf, (uint8_t *)rxBuf+datatocopy, (Size-datatocopy));  // copy the remaining data
//		newPos = (Size-datatocopy);  // update the position
//	}
//
//	/* if the current position + new data size is less than the main buffer
//	 * we will simply copy the data into the buffer and update the position
//	 */
//	else
//	{
//		memcpy ((uint8_t *)MainBuf+oldPos, rxBuf, Size);
//		newPos = Size+oldPos;
//	}
//
//	/* Update the position of the Head
//	 * If the current position + new size is less then the buffer size, Head will update normally
//	 * Or else the head will be at the new position from the beginning
//	 */
//	if (Head+Size < MainBuf_SIZE) Head = Head+Size;
//	else Head = Head+Size - MainBuf_SIZE;
//
//
//	/* start the DMA again */
//	usart_read(rxBuf, Size);
//}
//
///* Resets the Ring buffer */
//void Ringbuf_Reset (void)
//{
//	memset(MainBuf,'\0', MainBuf_SIZE);
//	memset(rxBuf, '\0', RxBuf_SIZE);
//	Tail = 0;
//	Head = 0;
//	oldPos = 0;
//	newPos = 0;
//}
//
//uint8_t checkString (char *str, char *buffertolookinto)
//{
//	int stringlength = strlen (str);
//	int bufferlength = strlen (buffertolookinto);
//	int so_far = 0;
//	int indx = 0;
//repeat:
//	while (str[so_far] != buffertolookinto[indx])
//	{
//		indx++;
//		if (indx>bufferlength) return 0;
//	}
//
//	if (str[so_far] == buffertolookinto[indx])
//	{
//		while (str[so_far] == buffertolookinto[indx])
//		{
//			so_far++;
//			indx++;
//		}
//	}
//
//	if (so_far != stringlength)
//	{
//		so_far =0;
//		if (indx >= bufferlength) return 0;
//		goto repeat;
//	}
//
//	if (so_far == stringlength) return 1;
//	else return 0;
//}
//
//
///* Waits for a particular string to arrive in the incoming buffer... It also increments the tail
// * returns 1, if the string is detected
// * return 0, in case of the timeout
// */
//int waitFor (char *string, uint32_t Timeout)
//{
//	int so_far =0;
//	int len = strlen (string);
//
//	TIMEOUT = Timeout;
//
//	while ((Tail==Head)&&TIMEOUT);  // let's wait for the data to show up
//	isDataAvailable = 0;
//
//again:
//
//	/* If the data doesn't show up, then return 0 */
//	if (TIMEOUT <= 0) return 0;
//
//
//	/* if the incoming data does not match with the string, we will simply increment the index
//	 * And wait for the string to arrive in the incoming data
//	 * */
//	while (MainBuf[Tail] != string[so_far])  // peek in the rx_buffer to see if we get the string
//	{
//		if (TIMEOUT <= 0) return 0;
//
//
//		if (Tail == Head) goto again;
//		Tail++;
//
//		if (Tail==MainBuf_SIZE) Tail = 0;
//	}
//
//	/* If the incoming data does match with the string, we will return 1 to indicate this */
//	while (MainBuf[Tail] == string[so_far]) // if we got the first letter of the string
//	{
//		if (TIMEOUT <= 0) return 0;
//		so_far++;
//
//		if (Tail == Head) goto again;
//		Tail++;
//		if (Tail==MainBuf_SIZE) Tail = 0;
//		if (so_far == len) return 1;
//	}
//
////	if (so_far != len)
////	{
////		so_far = 0;
////		goto again;
////	}
//
//	delay(100);
//
//	if ((so_far!=len)&&isDataAvailable)
//	{
//		isDataAvailable = 0;
////		so_far = 0;
//		goto again;
//	}
//	else
//	{
//		so_far = 0;
//		goto again;
//	}
//
//
//	return 0;
//}
//
//
///* copies the data from the incoming buffer into our buffer
// * Must be used if you are sure that the data is being received
// * it will copy irrespective of, if the end string is there or not
// * if the end string gets copied, it returns 1 or else 0
// *
// */
//int copyUpto (char *string, char *buffertocopyinto, uint32_t Timeout)
//{
//	int so_far =0;
//	int len = strlen (string);
//	int indx = 0;
//
//	TIMEOUT = Timeout;
//	while ((Tail==Head)&&TIMEOUT);
//	isDataAvailable = 0;
//again:
//
//	if (TIMEOUT<=0) return 0;
//
//	/* Keep copying data until the string is found in the incoming data */
//	while (MainBuf[Tail] != string [so_far])
//	{
//		buffertocopyinto[indx] = MainBuf[Tail];
//
//		if (Tail == Head) goto again;
//		Tail++;
//		indx++;
//		if (Tail==MainBuf_SIZE) Tail = 0;
//	}
//
///* If the string is found, copy it and return 1
// * or else goto again: and keep copying
// */
//	while (MainBuf[Tail] == string [so_far])
//	{
//		so_far++;
//		buffertocopyinto[indx++] = MainBuf[Tail++];
//		if (Tail==MainBuf_SIZE) Tail = 0;
//		if (so_far == len) return 1;
//	}
//
//	delay(100);
//
//	if ((so_far!=len)&&isDataAvailable)
//	{
//		isDataAvailable = 0;
////		so_far = 0;
//		goto again;
//	}
//	else
//	{
//		so_far = 0;
//		goto again;
//	}
//    return 0;
//}
//
//
//
///* Copies the entered number of characters, after the entered string (from the incoming buffer), into the buffer
// * returns 1, if the string is copied
// * returns 0, in case of the timeout
// */
//int getAfter (char *string, uint8_t numberofchars, char *buffertocopyinto, uint32_t Timeout)
//
//{
//	if ((waitFor(string, Timeout)) != 1) return 0;
////	TIMEOUT = Timeout/3;
////	while (TIMEOUT > 0);
//	delay(100);
//	for (int indx=0; indx<numberofchars; indx++)
//	{
//		if (Tail==MainBuf_SIZE) Tail = 0;
//		buffertocopyinto[indx] = MainBuf[Tail++];  // save the data into the buffer... increments the tail
//	}
//	return 1;
//}
//
//
///* Copies the data between the 2 strings from the source buffer into the destination buffer
// * It does not copy the start string or the end string..
// */
//
//void getDataFromBuffer (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto)
//{
//	int startStringLength = strlen (startString);
//	int endStringLength   = strlen (endString);
//	int so_far = 0;
//	int indx = 0;
//	int startposition = 0;
//	int endposition = 0;
//
//repeat1:
//
//	while (startString[so_far] != buffertocopyfrom[indx]) indx++;
//	if (startString[so_far] == buffertocopyfrom[indx])
//	{
//		while (startString[so_far] == buffertocopyfrom[indx])
//		{
//			so_far++;
//			indx++;
//		}
//	}
//
//	if (so_far == startStringLength) startposition = indx;
//	else
//	{
//		so_far =0;
//		goto repeat1;
//	}
//
//	so_far = 0;
//
//repeat2:
//
//	while (endString[so_far] != buffertocopyfrom[indx]) indx++;
//	if (endString[so_far] == buffertocopyfrom[indx])
//	{
//		while (endString[so_far] == buffertocopyfrom[indx])
//		{
//			so_far++;
//			indx++;
//		}
//	}
//
//	if (so_far == endStringLength) endposition = indx-endStringLength;
//	else
//	{
//		so_far =0;
//		goto repeat2;
//	}
//
//	so_far = 0;
//	indx=0;
//
//	for (int i=startposition; i<endposition; i++)
//	{
//		buffertocopyinto[indx] = buffertocopyfrom[i];
//		indx++;
//	}
//}
//
//
//
//
//
///* Interrupt handlers here */
//
///**
// * \brief           DMA1 stream1 interrupt handler for USART3 RX
// */
//void DMA1_Stream1_IRQHandler(void) {
//	/* Check idle line interrupt */
//	if(DMA1->LISR & DMA_LISR_TCIF1){
//		DMA1->LIFCR		|= DMA_LIFCR_CTCIF1;
//
////		DMA1_Stream1->CR	&= ~DMA_SxCR_TCIE;
//	}
//}
//
///**
// * \brief           USART3 global interrupt handler
// */
//void USART3_IRQHandler(void) {
//	/* Check for IDLE line interrupt */
//	if(USART3->ISR & USART_ISR_IDLE){
//		USART3->ICR |= USART_ICR_IDLECF;
//
////		uint16_t bytesRemaining = DMA1_Stream1->NDTR;
////		if((bytesRemaining > 0) && (bytesRemaining < rxSize)){
//			USART3->CR3		&= ~USART_CR3_DMAR;
//			USART3->CR1		&= ~USART_CR1_IDLEIE;
//
//			DMA1_Stream1->CR	&= ~DMA_SxCR_EN;
//			while(DMA1_Stream1->CR & DMA_SxCR_EN);
//
//			rx_process(rxSize);
////		}
//	}
//}
