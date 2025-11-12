/*
    \file  gd32g5x3_init.h
*/
/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef GD32G5X3_INIT_H
#define GD32G5X3_INIT_H

#include "gd32g5x3.h"

#ifdef __cplusplus
extern "C" {
 #endif

/* user code [global 0] begin */

/* user code [global 0] end */

/* External includes*/
/* user code [External Includes] begin */

/* user code [External Includes] end */

/* Private Type Definitions */
/* user code [Type Definitions] begin */

/* user code [Type Definitions] end */

/* Shared Macros */
/* user code [Shared Macros] begin */

/* user code [Shared Macros] end */

/* External Constants  */
/* user code [External Constants] begin */

/* user code [External Constants] end */

/* External Variables */
/* user code [External Variables] begin */

/* user code [External Variables] end */

void msd_system_init(void);
void msd_clock_init(void);
void msd_gpio_init(void);
void msd_gpio_deinit(void);
void msd_adc0_init(void);
void msd_adc0_deinit(void);
void msd_adc1_init(void);
void msd_adc1_deinit(void);
void msd_can0_init(void);
void msd_can0_deinit(void);
void msd_spi0_init(void);
void msd_spi2_deinit(void);
void msd_timer0_init(void);
void msd_timer0_deinit(void);
void msd_uart3_init(void);
void msd_uart3_deinit(void);
void msd_uart4_init(void);
void msd_uart4_deinit(void);
void msd_usart0_init(void);
void msd_usart0_deinit(void);
void msd_usart2_init(void);
void msd_usart2_deinit(void);
void timer_config(void);
void TIMER1_init(void);
void NVIC_init(void);
void WATCH_DOG_init(void);
void CAN0_init(int baudrate);
void msd_can2_init(void);
 
 

 
 
uint16_t spi_rw_half_word(uint32_t spi_periph, const uint16_t i_HalfWord);
uint16_t cdd_TLI5012_ReadReg(uint16_t i_Cmd, uint16_t * const  i_Data);
uint16_t cdd_TLI5012_WriteReg(uint16_t i_Cmd, uint16_t i_Data);
void cdd_TLI5012_Init(void);

void Test_TLI5012B(void);
void can2_txMessage(uint8_t length,uint8_t id,uint8_t *can2txdata);


/* user code [global 1] begin */

/* user code [global 1] end */

/* user code [Public Functions] begin */
void msd_timer0_multi_pwm_init(void);
/* user code [Public Functions] end */

#ifdef __cplusplus
}
 #endif

#endif /*GD32G5X3_INIT_H */
