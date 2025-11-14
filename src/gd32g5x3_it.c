/*!
    \file    gd32g5x3_it.c
    \brief   interrupt service routines

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

#include "gd32g5x3_it.h"
#include "main.h"
#include "systick.h"
#include "can.h"
#include "mc_task.h"
uint32_t inserted_data[5];
extern can_mailbox_descriptor_struct transmit_message;
extern can_mailbox_descriptor_struct receive_message;
// uint32_t testid = 0;
uint8_t can2_rxflag = 0;
uint8_t can0_rxflag = 0;
// uint8_t can0fd_txdata[64]={0};


//#include "can.h"
//#include "mc_task.h"
 void SysTick_Handler(void)
{
//    SystickCount++;
}


void NMI_Handler(void)
{
    Error_Handler();
}

void HardFault_Handler(void)
{
    Error_Handler();
}

void MemManage_Handler(void)
{
    Error_Handler();
}

void BusFault_Handler(void)
{
    Error_Handler();
}

void UsageFault_Handler(void)
{
    Error_Handler();
}

void SVC_Handler(void)
{
    Error_Handler();
}

void DebugMon_Handler(void)
{
    Error_Handler();
}

void PendSV_Handler(void)
{
    Error_Handler();
}

 


void ADC0_1_IRQHandler(void)
{
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);

     MCT_high_frequency_task();

//    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);
//    /* read ADC inserted sequence data register */
//    inserted_data[0] = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
//    inserted_data[1] = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_1);
//    inserted_data[2] = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_2);
//    inserted_data[3] = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_3);
//    inserted_data[4] = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);
//        
}





void TIMER1_IRQHandler(void)
{
    /* clear update interrupt bit */
    TIMER_INTF(TIMER1) = (~(uint32_t) TIMER_INT_FLAG_UP);

     MCT_safety_task();

    SystickCount++;

}

//void CAN0_RX0_IRQHandler(void)
//{
//     CAN_receive_callback();
//}
        CanFrame rxframe;

void CAN0_Message_IRQHandler(void)
{
    if(RESET != can_interrupt_flag_get(CAN0, CAN_INT_FLAG_MB0)) {
        can_interrupt_flag_clear(CAN0, CAN_INT_FLAG_MB0);
        can0_rxflag = SET;

//        can_mailbox_receive_data_read(CAN0, 0U, &receive_message);
//			
//        rxframe.id =  receive_message.id;
//        rxframe.dlc = receive_message.dlc ;
//        rxframe.data[0] = (uint8_t) receive_message.data[0];
//        rxframe.data[1] = (uint8_t) receive_message.data[1];
//        rxframe.data[2] = (uint8_t) receive_message.data[2];
//        rxframe.data[3] = (uint8_t) receive_message.data[3];
//        rxframe.data[4] = (uint8_t) receive_message.data[4];
//        rxframe.data[5] = (uint8_t) receive_message.data[5];
//        rxframe.data[6] = (uint8_t) receive_message.data[6];
//        rxframe.data[7] = (uint8_t) receive_message.data[7];
//        parse_frame(&rxframe);
    


    }
}

can_rx_fifo_struct rx_frame;
void CAN2_Message_IRQHandler(void)
{
    // if(RESET != can_interrupt_flag_get(CAN2, CAN_INT_FLAG_MB2)) {
    //     can_interrupt_flag_clear(CAN2, CAN_INT_FLAG_MB2);
    //    // can2_rxflag = SET;
    //      can_mailbox_receive_data_read(CAN2, 2U, &can2receive_message);
    //         can2_rxframe.id = can2receive_message.id & 0xFFFFFF;
    //         can2_rxframe.dlc = can2receive_message.dlc;
    //         for(int j=0;j<8;j++)
    //         {
    //           can2_rxframe.data[j] = (uint8_t) can2receive_message.data[j];
    //         }
            
    //        parse_frame(&can2_rxframe);



    // }

     if(can_interrupt_flag_get(CAN2, CAN_INT_FLAG_FIFO_AVAILABLE) != RESET) {
        
        
        /* 读取FIFO数据 */
        can_rx_fifo_read(CAN2, &rx_frame);

        can2_rxframe.id = rx_frame.id & 0xFFFFFF;
        can2_rxframe.dlc = rx_frame.dlc;
        can2_rxframe.data[0] = (rx_frame.data[0] >> 0) & 0xFF;  /* 字节3 → datanew[0] */
        can2_rxframe.data[1] = (rx_frame.data[0] >> 8) & 0xFF;  /* 字节2 → datanew[1] */
        can2_rxframe.data[2] = (rx_frame.data[0] >> 16)  & 0xFF;  /* 字节1 → datanew[2] */
        can2_rxframe.data[3] = (rx_frame.data[0] >> 24)  & 0xFF;  /* 字节0 → datanew[3] */
        can2_rxframe.data[4] = (rx_frame.data[1] >> 0) & 0xFF;  /* 字节7 → datanew[4] */
        can2_rxframe.data[5] = (rx_frame.data[1] >> 8) & 0xFF;  /* 字节6 → datanew[5] */
        can2_rxframe.data[6] = (rx_frame.data[1] >> 16)  & 0xFF;  /* 字节5 → datanew[6] */
        can2_rxframe.data[7] = (rx_frame.data[1] >> 24)  & 0xFF;  /* 字节4 → datanew[7]*/

        parse_frame(&can2_rxframe);
        //can_tx(&can2_rxframe);
        /* 处理接收到的数据 */
       // process_can2_received_frame(&rx_frame);
        
        /* 清除中断标志 */
        
        can_interrupt_flag_clear(CAN2, CAN_INT_FLAG_FIFO_AVAILABLE);

    }
    
    /* 处理FIFO警告中断 */
    if(can_interrupt_flag_get(CAN2, CAN_INT_FLAG_FIFO_WARNING) != RESET) {
        printf("CAN2 FIFO Warning Interrupt\n");
        can_interrupt_flag_clear(CAN2, CAN_INT_FLAG_FIFO_WARNING);
    }
    
    /* 处理FIFO溢出中断 */
    if(can_interrupt_flag_get(CAN2, CAN_INT_FLAG_FIFO_OVERFLOW) != RESET) {
        printf("CAN2 FIFO Overflow Interrupt - Data Lost!\n");
        can_interrupt_flag_clear(CAN2, CAN_INT_FLAG_FIFO_OVERFLOW);
    }
}
