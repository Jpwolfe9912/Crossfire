/*
 * dwt.c
 *
 *  Created on: Dec 28, 2021
 *      Author: jeremywolfe
 */
#include "dwt.h"

static volatile uint32_t usTicks = 0;

void DWT_Init(void){
	usTicks = SystemCoreClock / 1000000;
    // enable DWT access
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // enable the CPU cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delayMicroseconds(uint32_t us){
    uint32_t elapsed = 0;
    uint32_t lastCount = DWT->CYCCNT;
    uint32_t usTicks = 216;

    for (;;) {
        register uint32_t current_count = DWT->CYCCNT;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}

void delay(uint32_t ms){
    while (ms--)
        delayMicroseconds(1000);
}

