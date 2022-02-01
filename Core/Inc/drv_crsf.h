/*
 * drv_crsf.h
 *
 *  Created on: Jan 14, 2022
 *      Author: jeremywolfe
 */

#ifndef INC_DRV_CRSF_H_
#define INC_DRV_CRSF_H_

#include "stdbool.h"
#include "string.h"
#include "dwt.h"
#include "lwrb.h"

#define CRSF_FRAME_SIZE_MAX			64

/* Define the Size Here */
#define RXBUF_SIZE 64

#define PAYLOAD_SIZE				22

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

void crsf_init(void);

void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void crsf_process(void);


extern uint16_t rxSize;

extern uint8_t rxBuf[RxBuf_SIZE];

extern lwrb_t rxRingBuf;
extern uint8_t rxRingBufData[RxBuf_SIZE];

extern uint8_t crsfPayload[PAYLOAD_SIZE];

#endif /* INC_DRV_CRSF_H_ */
