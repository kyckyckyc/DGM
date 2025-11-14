/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example

    \version 2025-02-10, V1.1.0, firmware for GD32G5x3
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

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

#include "gd32g5x3.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
 #include "gd32g5x3_misc.h"
#include "gd32g5x3_init.h"
#include "anticogging.h"
#include "can.h"
#include "controller.h"
#include "encoder.h"
#include "foc.h"
#include "mc_task.h"
#include "pwm_curr.h"
#include "usr_config.h"
#include "rgb.h"
 volatile  uint32_t SystickCount = 0;
can_mailbox_descriptor_struct can2receive_message;
can_mailbox_descriptor_struct can2transmit_message;
// can_mailbox_descriptor_struct transmit_message;
// can_mailbox_descriptor_struct receive_message;
uint8_t can0fd_txdata[64]={0};
uint8_t can2_txdata[8]={0};
 uint32_t testid = 0;
extern uint8_t can0_rxflag;
extern uint8_t can2_rxflag;

/* 全局发送管理器 */
can2_tx_manager_t can2_tx_manager = {
    .mailboxes = {8, 9, 10},
    .current_index = 0,
    .sent_count = {0, 0, 0}
};
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/


int main(void)
{
		__disable_irq();
		msd_clock_init();
		msd_system_init();
		msd_gpio_init();
 		msd_adc0_init();
 		msd_adc1_init();

		msd_spi0_init();
		timer_config();
        trigsel_init(TRIGSEL_OUTPUT_ADC0_INSTRG, TRIGSEL_INPUT_TIMER0_CH3);
		TIMER1_init();
//		msd_uart3_init();
//		msd_uart4_init();
//		msd_usart0_init();
//		msd_usart2_init();
  		NVIC_init();
		//WATCH_DOG_init();
        msd_can0_init();
       // msd_can2_init();
       msd_can2_fifo_init();
//cdd_TLI5012_Init();
 if (USR_CONFIG_read_config()) {
       USR_CONFIG_set_default_config();
   }

   if (0 == USR_CONFIG_read_cogging_map()) {
       AnticoggingValid = true;
   } else {
       USR_CONFIG_set_default_cogging_map();
   }

    CAN_set_node_id(UsrConfig.node_id);
//      CAN0_init(UsrConfig.can_baudrate);

   MCT_init();
   FOC_init();
   PWMC_init();
   ENCODER_init();
   CONTROLLER_init();

    //fwdgt_enable();	
	
	
	
	
	    __enable_irq();
    can_struct_para_init(CAN_MDSC_STRUCT, &can2transmit_message);
    can_struct_para_init(CAN_MDSC_STRUCT, &can2receive_message);
    can_struct_para_init(CAN_MDSC_STRUCT, &transmit_message);
    can_struct_para_init(CAN_MDSC_STRUCT, &receive_message);
   // wait voltage stable
   for (uint8_t i = 0, j = 0; i < 250; i++) {
       if (Foc.v_bus_filt > 20) {
           if (++j > 20) {
               break;
           }
       }
       delay_ms(2);
   }

   if (PWMC_CurrentReadingPolarization() != 0) {
       StatuswordNew.errors.selftest = 1;
   }

   MCT_set_state(IDLE);

   uint32_t tick = 0;

   
    while(1) 
	{
  
     fwdgt_counter_reload();
    
     MCT_low_priority_task();

       if (get_ms_since(tick) > 1000) {
           tick = SystickCount;
       }




      for(int i = 0; i < 0x0a; i++){
           can0fd_txdata[i] = i;
         }
        // can0fd_txMessage(0x0a,0x08,can0fd_txdata);
  /////can2 send test
         can2_txdata[0]=0x10;
         can2_txdata[1]=0x11;
         can2_txdata[2]=0x12;
         can2_txdata[3]=0x13;
         can2_txdata[4]=0x14;

       // can2_txMessage_checked(0x05, 0x10, can2_txdata, 10u);
        //delay_ms(50);
        //can2_txMessage(0x05,0x09,can2_txdata);

		 if(SET == can0_rxflag) 
         {
            can0_rxflag = RESET;
            /* read the receive message */
            can_mailbox_receive_data_read(CAN0, 0U, &receive_message);
            // unpack_cmd(&receive_message);
         }
        if(SET == can2_rxflag) 
        {
           // can2_rxflag = RESET;
            /* read the receive message */
        //     can_mailbox_receive_data_read(CAN2, 2U, &can2receive_message);
             //can2_rxframe.id = can2receive_message.id & 0xFFFFFF;

        //     can2_rxframe.dlc = can2receive_message.dlc;
        //     for(int j=0;j<8;j++)
        //     {
        //       can2_rxframe.data[j] = (uint8_t) can2receive_message.data[j];
        //     }
            
        //    parse_frame(&can2_rxframe);
        }
			
    }
}

  void Error_Handler(void)
{
    __disable_irq();

    /* Main PWM Output Disable */
    timer_primary_output_config(TIMER0, DISABLE);

    while (1) {
    }
}

void delay_ms(const uint16_t ms)
{
    volatile uint32_t i = ms * 17200;
    while (i-- > 0) {
        __NOP();
        __NOP();
    }
}
