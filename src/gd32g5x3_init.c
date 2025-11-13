/*
    \file  gd32g5x3_init.c
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
#include "gd32g5x3.h"
#include "system_gd32g5x3.h"
#include "gd32g5x3_init.h"
#include "main.h"
#include "encoder.h"
/* user code [global 0] begin */
// #define SPI_TX_OFF()                                                                                \
// 	do {                                                                                          \
// 		gpio_init(SPI0_MOSI_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, SPI0_MOSI_PIN); \
// 	} while(0);
// #define SPI_TX_ON()                                                                             \
// 	do {                                                                                      \
// 		gpio_init(SPI0_MOSI_PORT, SPI0_MOSI_FUNC, GPIO_OSPEED_50MHZ, SPI0_MOSI_PIN); \
// 	} while(0);

// #define SPI_CS_ENABLE()	 gpio_bit_reset(U1_SPI0_NSS_PORT, U1_SPI0_NSS_PIN) 
// #define SPI_CS_DISABLE() gpio_bit_set(U1_SPI0_NSS_PORT, U1_SPI0_NSS_PIN)  









/* user code [global 0] end */

/* External includes*/
/* user code [External Includes] begin */

/* user code [External Includes] end */

/* Private Type Definitions */
/* user code [Private Type Definitions] begin */

/* user code [Private Type Definitions] end */

/* Private Macros */
/* user code [Private Macros] begin */

/* user code [Private Macros] end */

/* Private Constants */
/* user code [Private Constants] begin */

/* user code [Private Constants] end */

/* Private Variables */
/* user code [Private Variables] begin */

/* user code [Private Variables] end */

/* Private Function Declaration */
/* user code [Private Function Declaration] begin */
 
extern can_mailbox_descriptor_struct transmit_message;
extern can_mailbox_descriptor_struct receive_message;

/* user code [Private Function Declaration] end */

/* Extern Variables */
/* user code [Extern Variables] begin */

/* user code [Extern Variables] end */

/*!
    \brief      SYSTEM initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
// uint16_t spi_rw_half_word(uint32_t spi_periph, const uint16_t i_HalfWord)
// {
//     uint32_t to;
//     uint16_t data = 0xFFFF;
//     bool ok = true;

//     // Send data
//     to = 200;
//     while ((RESET == spi_flag_get(spi_periph, SPI_FLAG_TBE)) && --to);
//     if (to == 0) ok = false;

//     if (ok) {
//         if(RESET == spi_flag_get(spi_periph, SPI_FLAG_RBNE)) {
//             (void)spi_data_receive(spi_periph); /* throw Dummy */
//         }
        
//         spi_data_transmit(spi_periph, i_HalfWord);

//         /* wait rx succ*/
//         to = 200;
//         while ((RESET == spi_flag_get(spi_periph, SPI_FLAG_RBNE)) && --to);
//         if (to == 0) ok = false;
//         else data = spi_data_receive(spi_periph); /* throw Dummy wait bsu idle */

//         if (ok) {
//             to = 200;
//             while ((SET == spi_flag_get(spi_periph, SPI_FLAG_TRANS)) 
//                    && SET == spi_flag_get(spi_periph, SPI_FLAG_RBNE) && --to);
//             if (to == 0) ok = false;
//         }
//     }

//     return ok ? data : 0xFFFF;
// }
 






void msd_system_init(void)
{
    /* user code [system_init local 0] begin */

    /* user code [system_init local 0] end */

    rcu_periph_clock_enable(RCU_SYSCFG);
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    systick_clksource_set(SYSTICK_CLKSOURCE_HCLK);
    nvic_irq_enable(MemoryManagement_IRQn, 0, 0);
    nvic_irq_enable(BusFault_IRQn, 0, 0);
    nvic_irq_enable(UsageFault_IRQn, 0, 0);
    nvic_irq_enable(SVCall_IRQn, 0, 0);
    nvic_irq_enable(DebugMonitor_IRQn, 0, 0);
    nvic_irq_enable(PendSV_IRQn, 0, 0);

    /* user code [system_init local 1] begin */

    /* user code [system_init local 1] end */
}

/*!
    \brief      CLOCK initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
// #if defined(__SYSTEM_CLOCK_216M_PLL_HXTAL) 

// void msd_clock_init(void)
// {
//     /* user code [clock_init local 0] begin */

//     /* user code [clock_init local 0] end */

//     rcu_osci_on(RCU_HXTAL);
//     while (rcu_osci_stab_wait(RCU_HXTAL) != SUCCESS);

//     rcu_pll_source_config(RCU_PLLSRC_HXTAL);
//     rcu_pll_config((1U), (54U), (2U), (2U), (2U));
//     rcu_pll_clock_output_enable(RCU_PLLP);
//     rcu_pll_clock_output_enable(RCU_PLLR);
//     fmc_wscnt_set(FMC_WAIT_STATE_7);
//     rcu_system_clock_source_config(RCU_CKSYSSRC_PLLP);
//     rcu_osci_on(RCU_PLL_CK);
//     while (rcu_osci_stab_wait(RCU_PLL_CK) != SUCCESS);

//     rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);
//     rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV1);
//     rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV1);
//     rcu_apb3_clock_config(RCU_APB3_CKAHB_DIV1);

//     rcu_can_clock_config(IDX_CAN0, RCU_CANSRC_APB2);
//     rcu_adc_clock_config(IDX_ADC0, RCU_ADCSRC_PLLR);
//     rcu_adc_clock_config(IDX_ADC1, RCU_ADCSRC_PLLR);
//     rcu_usart_clock_config(IDX_USART0, RCU_USARTSRC_APB);
//     rcu_usart_clock_config(IDX_USART2, RCU_USARTSRC_APB);

//     rcu_osci_bypass_mode_disable(RCU_HXTAL);
//     rcu_osci_bypass_mode_disable(RCU_LXTAL);
//     rcu_hxtal_clock_monitor_disable();
//     rcu_lxtal_clock_monitor_disable();

//     /* update SystemCoreClock value */
//     SystemCoreClockUpdate();

//     /* setup systick timer for 1000Hz interrupts */
//     if (SysTick_Config(SystemCoreClock / 1000U))
//     {
//         /* capture error */
//         while (1);
//     }

//     /* user code [clock_init local 1] begin */

//     /* user code [clock_init local 1] end */
// }

//#elif defined(__SYSTEM_CLOCK_216M_PLL_IRC8M)

void msd_clock_init(void)
{


    rcu_osci_on(RCU_IRC8M);
    while (rcu_osci_stab_wait(RCU_IRC8M) != SUCCESS);
    rcu_irc8m_adjust_value_set(16U);

    rcu_pll_source_config(RCU_PLLSRC_IRC8M);
    rcu_pll_config((2U), (108U), (2U), (2U), (2U));
    rcu_pll_clock_output_enable(RCU_PLLP);
    fmc_wscnt_set(FMC_WAIT_STATE_7);
    rcu_system_clock_source_config(RCU_CKSYSSRC_PLLP);
    rcu_osci_on(RCU_PLL_CK);
    while (rcu_osci_stab_wait(RCU_PLL_CK) != SUCCESS);

    rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);
    rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV1);
    rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV1);
    rcu_apb3_clock_config(RCU_APB3_CKAHB_DIV1);
    rcu_osci_bypass_mode_disable(RCU_HXTAL);
    rcu_osci_bypass_mode_disable(RCU_LXTAL);
    rcu_hxtal_clock_monitor_disable();
    rcu_lxtal_clock_monitor_disable();
 
    SystemCoreClockUpdate();


    if (SysTick_Config(SystemCoreClock / 1000U))
    {

    while (1);
    }

}

//#endif


/*!
    \brief      GPIO initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_gpio_init(void)
{
//    /* user code [gpio_init local 0] begin */

//    /* user code [gpio_init local 0] end */

//    rcu_periph_clock_enable(RCU_GPIOD);
//    rcu_periph_clock_enable(RCU_GPIOC);
//    rcu_periph_clock_enable(RCU_GPIOB);
//    rcu_periph_clock_enable(RCU_GPIOA);
//    gpio_af_set(GPIOC, GPIO_AF_1, GPIO_PIN_6);
//    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
//    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6);
//    gpio_input_filter_set(GPIOC, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_6);

//    gpio_bit_set(GPIOB, GPIO_PIN_2);
//    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_2);

//    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_6);
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6);
//    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_6);

//    gpio_af_set(GPIOC, GPIO_AF_2, GPIO_PIN_8);
//    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
//    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_8);
//    gpio_input_filter_set(GPIOC, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_8);

//    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
//    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN7);
//    exti_init(EXTI_7, EXTI_INTERRUPT, EXTI_TRIG_RISING);
//    exti_interrupt_flag_clear(EXTI_7);

//    gpio_af_set(GPIOA, GPIO_AF_6, GPIO_PIN_8);
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_8);
//    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_8);

//    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_7);
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_7);
//    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_7);

//    gpio_af_set(GPIOC, GPIO_AF_2, GPIO_PIN_9);
//    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
//    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_9);
//    gpio_input_filter_set(GPIOC, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_9);

//    gpio_af_set(GPIOA, GPIO_AF_6, GPIO_PIN_9);
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_9);
//    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_9);

//    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_14);
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_14);
//    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_14);

//    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_13);
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_13);
//    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_13);

//    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_15);
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_15);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_15);
//    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_15);

//    gpio_af_set(GPIOA, GPIO_AF_6, GPIO_PIN_10);
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_10);
//    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_10);

//    gpio_bit_reset(GPIOC, GPIO_PIN_0);
//    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
//    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_0);

//    gpio_bit_reset(GPIOC, GPIO_PIN_2);
//    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
//    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_2);

//    gpio_bit_set(GPIOB, GPIO_PIN_1);
//    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_1);

//    gpio_bit_reset(GPIOC, GPIO_PIN_1);
//    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
//    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_1);

//    /* user code [gpio_init local 1] begin */
   /* user code [gpio_init local 0] begin */

    /* user code [gpio_init local 0] end */

    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOA);
    //CS1
    gpio_bit_reset(GPIOC, GPIO_PIN_4);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_4);
    //CS0
    gpio_bit_reset(GPIOA, GPIO_PIN_4);
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_4);
    //RGB
    gpio_bit_reset(GPIOB, GPIO_PIN_5);
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_5);
    //TEMP MOS
    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_5);
    //enc_IFB
    gpio_af_set(GPIOC, GPIO_AF_2, GPIO_PIN_8);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_8);
    gpio_input_filter_set(GPIOC, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_8);
    //SPI0_MISO
    gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_6);
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_6);
    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_6);
    //SPI0_SCK
    gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_5);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_5);
    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_5);
    //enc_IFA
    gpio_af_set(GPIOC, GPIO_AF_2, GPIO_PIN_7);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_7);
    gpio_input_filter_set(GPIOC, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_7);
    //PWMAL
    gpio_af_set(GPIOA, GPIO_AF_6, GPIO_PIN_8);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_8);
    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_8);
    //SPI0_MOSI
    gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_7);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_7);
    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_7);
    //enc_IFC
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_9);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN9);
    exti_init(EXTI_9, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_flag_clear(EXTI_9);
    //PWMBL
    gpio_af_set(GPIOA, GPIO_AF_6, GPIO_PIN_9);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_9);
    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_9);
    //PWMBH
    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_14);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_14);
    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_14);
    //PWMAH
    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_13);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_13);
    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_13);
    //PWMCH
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_15);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_15);
    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_15);
    //PWMAL
    gpio_af_set(GPIOA, GPIO_AF_6, GPIO_PIN_10);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_10);
    gpio_input_filter_set(GPIOA, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_10);
    //nfault
    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_12);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_12);
    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_12);
    //nsleep
    gpio_bit_reset(GPIOB, GPIO_PIN_11);
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_11);

    /* user code [gpio_init local 1] begin */

    /* user code [gpio_init local 1] end */
    /* user code [gpio_init local 1] end */
}

/*!
    \brief      GPIO deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_gpio_deinit(void)
{
    /* user code [gpio_deinit local 0] begin */

    /* user code [gpio_deinit local 0] end */

    rcu_periph_clock_disable(RCU_GPIOD);
    rcu_periph_clock_disable(RCU_GPIOC);
    rcu_periph_clock_disable(RCU_GPIOB);
    rcu_periph_clock_disable(RCU_GPIOA);

    exti_deinit();

    /* user code [gpio_deinit local 1] begin */

    /* user code [gpio_deinit local 1] end */
}

/*!
    \brief      ADC0 initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_adc0_init(void)
{
    /* user code [adc0_init local 0] begin */

    /* user code [adc0_init local 0] end */

    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_TRIGSEL);

    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_2);

    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_1);

    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_3);

    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0);


   /* reset ADC */
    adc_deinit(ADC0);
    /* ADC clock config */
    adc_clock_config(ADC0, ADC_CLK_SYNC_HCLK_DIV12);
    /* ADC contineous function enable */
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC scan mode enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC resolution config */
    adc_resolution_config(ADC0, ADC_RESOLUTION_12B);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 4);

    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_0, 6);//Ia
    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_1, 6);//Ib
    adc_inserted_channel_config(ADC0, 2, ADC_CHANNEL_2, 6);//Ic
    adc_inserted_channel_config(ADC0, 3, ADC_CHANNEL_3, 6);//vbus
//     trigsel_init(TRIGSEL_OUTPUT_ADC0_INSTRG, TRIGSEL_INPUT_TIMER0_CH3);
 
    /* ADC trigger config */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, EXTERNAL_TRIGGER_RISING);
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);

    /* enable ADC interface */
    adc_enable(ADC0);
    /* wait for ADC stability */
     delay_ms(1);
    /* ADC calibration mode config */
    adc_calibration_mode_config(ADC0, ADC_CALIBRATION_OFFSET_MISMATCH);
    /* ADC calibration number config */
    adc_calibration_number(ADC0, ADC_CALIBRATION_NUM1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);

}

/*!
    \brief      ADC0 deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_adc0_deinit(void)
{
    /* user code [adc0_deinit local 0] begin */

    /* user code [adc0_deinit local 0] end */

    rcu_periph_clock_disable(RCU_ADC0);

    adc_deinit(ADC0);

    /* user code [adc0_deinit local 1] begin */

    /* user code [adc0_deinit local 1] end */
}

/*!
    \brief      ADC1 initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_adc1_init(void)
{
    /* user code [adc1_init local 0] begin */

    /* user code [adc1_init local 0] end */

    rcu_periph_clock_enable(RCU_ADC1);
    rcu_periph_clock_enable(RCU_TRIGSEL);

    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_5);


   /* reset ADC */
    adc_deinit(ADC1);
    /* ADC clock config */
    adc_clock_config(ADC1, ADC_CLK_SYNC_HCLK_DIV12);
    /* ADC contineous function enable */
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC scan mode enable */
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
    /* ADC resolution config */
    adc_resolution_config(ADC1, ADC_RESOLUTION_12B);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 1);

     adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_10, 6);//mos temp
 
     trigsel_init(TRIGSEL_OUTPUT_ADC1_INSTRG, TRIGSEL_INPUT_TIMER0_CH3);
 
    /* ADC trigger config */
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, EXTERNAL_TRIGGER_RISING);
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
//     adc_interrupt_enable(ADC1, ADC_INT_EOIC);

    /* enable ADC interface */
    adc_enable(ADC1);
    /* wait for ADC stability */
     delay_ms(1);
    /* ADC calibration mode config */
    adc_calibration_mode_config(ADC1, ADC_CALIBRATION_OFFSET_MISMATCH);
    /* ADC calibration number config */
    adc_calibration_number(ADC1, ADC_CALIBRATION_NUM1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);

    /* user code [adc1_init local 1] begin */

    /* user code [adc1_init local 1] end */
}

/*!
    \brief      ADC1 deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_adc1_deinit(void)
{
    /* user code [adc1_deinit local 0] begin */

    /* user code [adc1_deinit local 0] end */

    rcu_periph_clock_disable(RCU_ADC1);

    adc_deinit(ADC1);

    /* user code [adc1_deinit local 1] begin */

    /* user code [adc1_deinit local 1] end */
}

 
/*!
    \brief      CAN0 deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_can0_deinit(void)
{
    /* user code [can0_deinit local 0] begin */

    /* user code [can0_deinit local 0] end */

    rcu_periph_clock_disable(RCU_CAN0);

    can_deinit(CAN0);

    /* user code [can0_deinit local 1] begin */

    /* user code [can0_deinit local 1] end */
}

/*!
    \brief      SPI2 initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
    spi_parameter_struct spi0_parameter;


void msd_spi0_init(void)
{
    /* user code [spi2_init local 0] begin */

    /* user code [spi2_init local 0] end */


    rcu_periph_clock_enable(RCU_SPI0);

//    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_3);
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_3);
//    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_3);

//    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_5);
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_5);
//    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_5);

//    gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_4);
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_4);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_4);
//    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_4);

    spi_struct_para_init(&spi0_parameter);

    spi0_parameter.device_mode = SPI_MASTER;
    spi0_parameter.nss = SPI_NSS_SOFT;
    spi0_parameter.endian = SPI_ENDIAN_MSB;

    #if defined(TLE5012B)
    spi0_parameter.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;//
    spi0_parameter.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi0_parameter.prescale = SPI_PSC_64;
    spi0_parameter.frame_size = SPI_FRAMESIZE_16BIT;
    
    #elif defined(KTH71)
    spi0_parameter.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;//
    spi0_parameter.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi0_parameter.prescale = SPI_PSC_8;
    spi0_parameter.frame_size = SPI_FRAMESIZE_8BIT;

    #endif
    spi_init(SPI0, &spi0_parameter);


    // spi_nss_output_enable(SPI0);
    // spi_nssp_mode_enable(SPI0);

//    spi_fifo_access_size_config(SPI0, SPI_HALFWORD_ACCESS);

    spi_enable(SPI0);


 

    /* user code [spi2_init local 1] begin */

    /* user code [spi2_init local 1] end */
}





/**
  * @brief Send or read half word through SPI
  * @param[in] spi_periph: SPI peripheral
  * @param[in] i_HalfWord: data to send
  * @return received data
  */
 


/*!
    \brief      SPI2 deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_spi2_deinit(void)
{
    /* user code [spi2_deinit local 0] begin */

    /* user code [spi2_deinit local 0] end */

    rcu_periph_clock_disable(RCU_SPI2);

    spi_deinit(SPI2);

    /* user code [spi2_deinit local 1] begin */

    /* user code [spi2_deinit local 1] end */
}

/*!
    \brief      TIMER0 initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_timer0_init(void)
{
    /* user code [timer0_init local 0] begin */

    /* user code [timer0_init local 0] end */

    timer_parameter_struct timer0_parameter;

    rcu_periph_clock_enable(RCU_TIMER0);

    timer_struct_para_init(&timer0_parameter);

    timer_adjustment_mode_config(TIMER0, DISABLE);
    timer_auto_reload_shadow_enable(TIMER0);
    timer_counter_initial_register_config(TIMER0, DISABLE);
    timer_upif_backup_config(TIMER0, DISABLE);
    timer_master_slave_mode_config(TIMER0, TIMER_MASTER_SLAVE_MODE_DISABLE);
    timer_master_output0_trigger_source_select(TIMER0, TIMER_TRI_OUT0_SRC_O3CPRE);
    timer_master_output1_trigger_source_select(TIMER0, TIMER_TRI_OUT1_SRC_UPDATE);
    timer0_parameter.prescaler = 0;
    timer0_parameter.alignedmode = TIMER_COUNTER_CENTER_BOTH;
    timer0_parameter.period = 5399;
    timer0_parameter.repetitioncounter = 0x00000000;
    timer0_parameter.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER0, &timer0_parameter);

    timer_internal_clock_config(TIMER0);

    nvic_irq_enable(TIMER0_BRK_IRQn, 0, 0);
    nvic_irq_enable(TIMER0_UP_IRQn, 0, 0);

    /* user code [timer0_init local 1] begin */

    /* user code [timer0_init local 1] end */
}

/*!
    \brief      TIMER0 deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_timer0_deinit(void)
{
    /* user code [timer0_deinit local 0] begin */

    /* user code [timer0_deinit local 0] end */

    rcu_periph_clock_disable(RCU_TIMER0);

    timer_deinit(TIMER0);

    /* user code [timer0_deinit local 1] begin */

    /* user code [timer0_deinit local 1] end */
}

/*!
    \brief      UART3 initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_uart3_init(void)
{
    /* user code [uart3_init local 0] begin */

    /* user code [uart3_init local 0] end */

    rcu_periph_clock_enable(RCU_UART3);

    gpio_af_set(GPIOC, GPIO_AF_5, GPIO_PIN_11);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_11);
    gpio_input_filter_set(GPIOC, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_11);

    gpio_af_set(GPIOC, GPIO_AF_5, GPIO_PIN_10);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_10);
    gpio_input_filter_set(GPIOC, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_10);

    usart_baudrate_set(UART3, 115200);
    usart_word_length_set(UART3, USART_WL_8BIT);
    usart_parity_config(UART3, USART_PM_NONE);
    usart_stop_bit_set(UART3, USART_STB_1BIT);
    usart_receive_config(UART3, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART3, USART_TRANSMIT_ENABLE);
    usart_oversample_config(UART3, USART_OVSMOD_16);
    usart_sample_bit_config(UART3, USART_OSB_3BIT);
    usart_fifo_disable(UART3);
    usart_overrun_disable(UART3);
    usart_reception_error_dma_disable(UART3);
    usart_data_first_config(UART3, USART_MSBF_LSB);
    usart_invert_config(UART3, USART_TXPIN_DISABLE);
    usart_invert_config(UART3, USART_RXPIN_DISABLE);
    usart_invert_config(UART3, USART_DINV_DISABLE);
    usart_invert_config(UART3, USART_SWAP_DISABLE);
    usart_enable(UART3);

    /* user code [uart3_init local 1] begin */

    /* user code [uart3_init local 1] end */
}

/*!
    \brief      UART3 deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_uart3_deinit(void)
{
    /* user code [uart3_deinit local 0] begin */

    /* user code [uart3_deinit local 0] end */

    rcu_periph_clock_disable(RCU_UART3);

    usart_deinit(UART3);

    /* user code [uart3_deinit local 1] begin */

    /* user code [uart3_deinit local 1] end */
}

/*!
    \brief      UART4 initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_uart4_init(void)
{
    /* user code [uart4_init local 0] begin */

    /* user code [uart4_init local 0] end */

    rcu_periph_clock_enable(RCU_UART4);

    gpio_af_set(GPIOC, GPIO_AF_5, GPIO_PIN_12);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_12);
    gpio_input_filter_set(GPIOC, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_12);

    gpio_af_set(GPIOD, GPIO_AF_5, GPIO_PIN_2);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_2);
    gpio_input_filter_set(GPIOD, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_2);

    usart_baudrate_set(UART4, 115200);
    usart_word_length_set(UART4, USART_WL_8BIT);
    usart_parity_config(UART4, USART_PM_NONE);
    usart_stop_bit_set(UART4, USART_STB_1BIT);
    usart_receive_config(UART4, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
    usart_oversample_config(UART4, USART_OVSMOD_16);
    usart_sample_bit_config(UART4, USART_OSB_3BIT);
    usart_fifo_disable(UART4);
    usart_overrun_disable(UART4);
    usart_reception_error_dma_disable(UART4);
    usart_data_first_config(UART4, USART_MSBF_LSB);
    usart_invert_config(UART4, USART_TXPIN_DISABLE);
    usart_invert_config(UART4, USART_RXPIN_DISABLE);
    usart_invert_config(UART4, USART_DINV_DISABLE);
    usart_invert_config(UART4, USART_SWAP_DISABLE);
    usart_enable(UART4);

    /* user code [uart4_init local 1] begin */

    /* user code [uart4_init local 1] end */
}

/*!
    \brief      UART4 deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_uart4_deinit(void)
{
    /* user code [uart4_deinit local 0] begin */

    /* user code [uart4_deinit local 0] end */

    rcu_periph_clock_disable(RCU_UART4);

    usart_deinit(UART4);

    /* user code [uart4_deinit local 1] begin */

    /* user code [uart4_deinit local 1] end */
}

/*!
    \brief      USART0 initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_usart0_init(void)
{
    /* user code [usart0_init local 0] begin */

    /* user code [usart0_init local 0] end */

    rcu_periph_clock_enable(RCU_USART0);

    gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_7);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_7);
    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_7);

    gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_6);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_6);
    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_6);

    usart_baudrate_set(USART0, 115200);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_receiver_timeout_disable(USART0);
    usart_oversample_config(USART0, USART_OVSMOD_16);
    usart_sample_bit_config(USART0, USART_OSB_3BIT);
    usart_fifo_disable(USART0);
    usart_overrun_disable(USART0);
    usart_reception_error_dma_disable(USART0);
    usart_data_first_config(USART0, USART_MSBF_LSB);
    usart_invert_config(USART0, USART_TXPIN_DISABLE);
    usart_invert_config(USART0, USART_RXPIN_DISABLE);
    usart_invert_config(USART0, USART_DINV_DISABLE);
    usart_invert_config(USART0, USART_SWAP_DISABLE);
    usart_enable(USART0);

    /* user code [usart0_init local 1] begin */

    /* user code [usart0_init local 1] end */
}

/*!
    \brief      USART0 deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_usart0_deinit(void)
{
    /* user code [usart0_deinit local 0] begin */

    /* user code [usart0_deinit local 0] end */

    rcu_periph_clock_disable(RCU_USART0);

    usart_deinit(USART0);

    /* user code [usart0_deinit local 1] begin */

    /* user code [usart0_deinit local 1] end */
}

/*!
    \brief      USART2 initialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_usart2_init(void)
{
    /* user code [usart2_init local 0] begin */

    /* user code [usart2_init local 0] end */

    rcu_periph_clock_enable(RCU_USART2);

    gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_9);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_9);
    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_9);

    gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_11);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, GPIO_PIN_11);
    gpio_input_filter_set(GPIOB, GPIO_ISPERIOD(0), GPIO_IFTYPE_ASYNC, GPIO_PIN_11);

    usart_baudrate_set(USART2, 115200);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_receiver_timeout_disable(USART2);
    usart_oversample_config(USART2, USART_OVSMOD_16);
    usart_sample_bit_config(USART2, USART_OSB_3BIT);
    usart_fifo_disable(USART2);
    usart_overrun_disable(USART2);
    usart_reception_error_dma_disable(USART2);
    usart_data_first_config(USART2, USART_MSBF_LSB);
    usart_invert_config(USART2, USART_TXPIN_DISABLE);
    usart_invert_config(USART2, USART_RXPIN_DISABLE);
    usart_invert_config(USART2, USART_DINV_DISABLE);
    usart_invert_config(USART2, USART_SWAP_DISABLE);
    usart_enable(USART2);

    /* user code [usart2_init local 1] begin */

    /* user code [usart2_init local 1] end */
}

/*!
    \brief      USART2 deinitialization function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void msd_usart2_deinit(void)
{
    /* user code [usart2_deinit local 0] begin */

    /* user code [usart2_deinit local 0] end */

    rcu_periph_clock_disable(RCU_USART2);

    usart_deinit(USART2);

    /* user code [usart2_deinit local 1] begin */

    /* user code [usart2_deinit local 1] end */
}

/* user code [global 1] begin */

/* user code [global 1] end */

/* user code [Public Functions Implementations] begin */

/* user code [Public Functions Implementations] end */

/* user code [Private Functions Implementations] begin */

/* user code [Private Functions Implementations] end */


void timer_config(void)
{
    /* -----------------------------------------------------------------------
    TIMER0 configuration to:
    generate 4 complementary PWM signals with 4 different duty cycles:
    - TIMER0 frequency is fixed to 216MHz, TIMER0 prescaler is equal to 216,
       so TIMER0 counter frequency is 1MHz, the PWM frequency = 1KHz.

       the four duty cycles are computed as the following description:
       the channel 0 duty cycle is set to 20%, so multi mode channel 0 is set to 80%.
       the channel 1 duty cycle is set to 40%, so multi mode channel 1 is set to 60%.
       the channel 2 duty cycle is set to 60%, so multi mode channel 2 is set to 40%.
       the channel 3 duty cycle is set to 80%, so multi mode channel 3 is set to 20%.
    - CH0/MCH0,CH1/MCH1, CH2/MCH2 and CH3/MCH3 are configured in PWM mode 0.
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    timer_break_parameter_struct timer_breakpara;

    rcu_periph_clock_enable(RCU_TIMER0);
    timer_deinit(TIMER0);

    /* TIMER0 configuration */
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_DOWN;
//    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 5399;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 1;
    timer_init(TIMER0, &timer_initpara);

    /* CH0~3 & MCH0~3 configuration in PWM mode */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_HIGH;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* config CH0~3 & MCH0~3 as output*/
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocinitpara);

    /* config MCH0~3 output complementary */
    timer_multi_mode_channel_mode_config(TIMER0, TIMER_CH_0, TIMER_MCH_MODE_COMPLEMENTARY);
    timer_multi_mode_channel_mode_config(TIMER0, TIMER_CH_1, TIMER_MCH_MODE_COMPLEMENTARY);
    timer_multi_mode_channel_mode_config(TIMER0, TIMER_CH_2, TIMER_MCH_MODE_COMPLEMENTARY);
    timer_multi_mode_channel_mode_config(TIMER0, TIMER_CH_3, TIMER_MCH_MODE_COMPLEMENTARY);

    /* CH0 configuration in PWM mode 0, duty cycle 20% */
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 200);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration in PWM mode 0, duty cycle 40% */
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 400);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode 0, duty cycle 60% */
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 600);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    /* CH3 configuration in PWM mode 0, duty cycle 80% */
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, 20);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    /* BREAK configuration */
    timer_break_struct_para_init(&timer_breakpara);
    

    timer_breakpara.runoffstate         = TIMER_ROS_STATE_ENABLE;
    timer_breakpara.ideloffstate        = TIMER_IOS_STATE_ENABLE;
    timer_breakpara.deadtime            = 54U;
    timer_breakpara.outputautostate     = TIMER_OUTAUTO_DISABLE;
    timer_breakpara.protectmode         = TIMER_CCHP0_PROT_OFF;
    timer_breakpara.break0state         = TIMER_BREAK0_ENABLE;
    timer_breakpara.break0filter        = 30U;
    timer_breakpara.break0polarity      = TIMER_BREAK0_POLARITY_LOW;
    timer_breakpara.break0lock          = TIMER_BREAK0_LK_DISABLE;
    timer_breakpara.break0release       = TIMER_BREAK0_UNRELEASE;
    timer_breakpara.break1state         = TIMER_BREAK1_DISABLE;
    timer_breakpara.break1filter        = 0U;
    timer_breakpara.break1polarity      = TIMER_BREAK1_POLARITY_HIGH;
    timer_breakpara.break1lock          = TIMER_BREAK1_LK_DISABLE;
    timer_breakpara.break1release       = TIMER_BREAK1_UNRELEASE;
    timer_break_config(TIMER0, &timer_breakpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);
    timer_primary_output_config(TIMER0, ENABLE);

    /* auto-reload preload enable */
    timer_enable(TIMER0);
}


  void TIMER1_init(void)
{
     rcu_periph_clock_enable(RCU_TIMER1);

    // TIMER1CLK = SystemCoreClock/2/60000 = 1KHz
    timer_parameter_struct timer_initpara;

    timer_deinit(TIMER1);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler        = 1;
    timer_initpara.alignedmode      = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period           = 108000 - 1;
    timer_initpara.clockdivision    = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* enable the TIMER interrupt */
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);

    timer_enable(TIMER1);
}


  void NVIC_init(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    // ADC01 insert convert complete interrupt NVIC
    NVIC_SetPriority(ADC0_1_IRQn, 0);
    NVIC_EnableIRQ(ADC0_1_IRQn);

    // TIM1 interrupt NVIC
    NVIC_SetPriority(TIMER1_IRQn, 1);
    NVIC_EnableIRQ(TIMER1_IRQn);


    // CAN0 Rx interrupt NVIC
    NVIC_SetPriority(CAN0_Message_IRQn, 2);
    NVIC_EnableIRQ(CAN0_Message_IRQn);

    NVIC_SetPriority(CAN2_Message_IRQn, 2);
    NVIC_EnableIRQ(CAN2_Message_IRQn);

}




void WATCH_DOG_init(void)
{


    /* enable IRC32K */
    rcu_osci_on(RCU_IRC32K);
    
    /* wait till IRC32K is ready */
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC32K)){
    }
    
    /* configure FWDGT counter clock: 32KHz(IRC32K) / 64 = 0.5 KHz */
    fwdgt_config(2*500, FWDGT_PSC_DIV64);
    
    /* after 2 seconds to generate a reset */
//    fwdgt_enable();


}

 void msd_can0_init(void)
{
     /* user code [can0_init local 0] begin */

    /* user code [can0_init local 0] end */

    /* enable CAN port clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* configure CAN0_TX GPIO */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_12);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_12);
    gpio_af_set(GPIOA, GPIO_AF_9, GPIO_PIN_12);

    /* configure CAN0_RX GPIO */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_11);
    gpio_af_set(GPIOA, GPIO_AF_9, GPIO_PIN_11);

    can_parameter_struct can_parameter;
    can_fd_parameter_struct fd_parameter;

    /* select CK_APB2 as CAN's clock source */
    rcu_can_clock_config(IDX_CAN0, RCU_CANSRC_APB2);
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    /* initialize CAN register */
    can_deinit(CAN0);
    /* initialize CAN */
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FD_INIT_STRUCT, &fd_parameter);

    /* initialize CAN parameters */
    can_parameter.internal_counter_source = CAN_TIMER_SOURCE_BIT_CLOCK;
    can_parameter.self_reception = DISABLE;
    can_parameter.mb_tx_order = CAN_TX_HIGH_PRIORITY_MB_FIRST;
    can_parameter.mb_tx_abort_enable = ENABLE;
    can_parameter.local_priority_enable = DISABLE;
    can_parameter.mb_rx_ide_rtr_type = CAN_IDE_RTR_FILTERED;
    can_parameter.mb_remote_frame = CAN_STORE_REMOTE_REQUEST_FRAME;
    can_parameter.rx_private_filter_queue_enable = DISABLE;
    can_parameter.edge_filter_enable = DISABLE;
    can_parameter.protocol_exception_enable = DISABLE;
    can_parameter.rx_filter_order = CAN_RX_FILTER_ORDER_MAILBOX_FIRST;
    can_parameter.memory_size = CAN_MEMSIZE_32_UNIT;
    /* filter configuration */
    can_parameter.mb_public_filter = 0U;
    /* baud rate 1Mbps */
    can_parameter.resync_jump_width = 1U;
    can_parameter.prop_time_segment = 2U;
    can_parameter.time_segment_1 = 4U;
    can_parameter.time_segment_2 = 1U;
    can_parameter.prescaler = 27U;

    /* initialize CAN */
    can_init(CAN0, &can_parameter);

    /* FD parameter configurations */
    fd_parameter.bitrate_switch_enable = ENABLE;
    fd_parameter.iso_can_fd_enable = ENABLE;
    fd_parameter.mailbox_data_size = CAN_MAILBOX_DATA_SIZE_64_BYTES;
    fd_parameter.tdc_enable = DISABLE;
    fd_parameter.tdc_offset = 0U;
    /* FD baud rate 1Mbps */
    fd_parameter.resync_jump_width = 1U;
    fd_parameter.prop_time_segment = 2U;
    fd_parameter.time_segment_1 = 4U;
    fd_parameter.time_segment_2 = 1U;
    fd_parameter.prescaler = 27U;

    can_fd_config(CAN0, &fd_parameter);

    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_Message_IRQn, 0U, 0U);

    /* enable CAN MB0 interrupt */
    can_interrupt_enable(CAN0, CAN_INT_MB0);

    can_operation_mode_enter(CAN0, CAN_NORMAL_MODE);

    receive_message.rtr = 0U;
    receive_message.ide = 0U;
    receive_message.code = CAN_MB_RX_STATUS_EMPTY;
    /* rx mailbox */
    receive_message.id = 0x55U;
    can_mailbox_config(CAN0, 0U, &receive_message);
 
}

void msd_can2_init(void)
{
    /* user code [can0_init local 0] begin */

    /* user code [can0_init local 0] end */

    rcu_periph_clock_enable(RCU_GPIOB);

    /* configure CAN0_TX GPIO */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_4);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_4);

    /* configure CAN0_RX GPIO */
	gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, GPIO_PIN_3);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_3);
   
	can_parameter_struct can2_parameter;
    //can_fd_parameter_struct fd2_parameter;

    /* select CK_APB2 as CAN's clock source */
    rcu_can_clock_config(IDX_CAN2, RCU_CANSRC_APB2);
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN2);
    /* initialize CAN register */
    can_deinit(CAN2);
    /* initialize CAN */
    can_struct_para_init(CAN_INIT_STRUCT, &can2_parameter);
    //can_struct_para_init(CAN_FD_INIT_STRUCT, &fd2_parameter);

    /* initialize CAN parameters */
    can2_parameter.internal_counter_source = CAN_TIMER_SOURCE_BIT_CLOCK;
    can2_parameter.self_reception = DISABLE;
    can2_parameter.mb_tx_order = CAN_TX_HIGH_PRIORITY_MB_FIRST;
    can2_parameter.mb_tx_abort_enable = ENABLE;
    can2_parameter.local_priority_enable = DISABLE;
    can2_parameter.mb_rx_ide_rtr_type = CAN_IDE_RTR_FILTERED;
    can2_parameter.mb_remote_frame = CAN_STORE_REMOTE_REQUEST_FRAME;
    can2_parameter.rx_private_filter_queue_enable = DISABLE;
    can2_parameter.edge_filter_enable = DISABLE;
    can2_parameter.protocol_exception_enable = DISABLE;
    can2_parameter.rx_filter_order = CAN_RX_FILTER_ORDER_MAILBOX_FIRST;
    can2_parameter.memory_size = CAN_MEMSIZE_32_UNIT;
    /* filter configuration */
    can2_parameter.mb_public_filter = 0U;
    /* baud rate 1Mbps */
    can2_parameter.resync_jump_width = 1U;
    can2_parameter.prop_time_segment = 2U;
    can2_parameter.time_segment_1 = 4U;
    can2_parameter.time_segment_2 = 1U;
    can2_parameter.prescaler = 54U;

    /* initialize CAN */
    can_init(CAN2, &can2_parameter);

    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN2_Message_IRQn, 0U, 0U);

    /* enable CAN MB0 interrupt */
    can_interrupt_enable(CAN2, CAN_INT_MB2);

    can_operation_mode_enter(CAN2, CAN_NORMAL_MODE);

    can2receive_message.rtr = 0U;
    can2receive_message.ide = 0U;
    can2receive_message.code = CAN_MB_RX_STATUS_EMPTY;
    /* rx mailbox */
    can2receive_message.id = 0x01U;
    can_mailbox_config(CAN2, 2U, &can2receive_message);
}
void can0fd_txMessage(uint8_t length,uint8_t id,uint8_t *can0txdata)
{
    for(int i = 0; i < length; i++){
        transmit_message.data[i] = can0txdata[i];
    }
    transmit_message.rtr = 0U;
    transmit_message.ide = 0U;
    transmit_message.code = CAN_MB_TX_STATUS_DATA;
    transmit_message.brs = 1U;
    transmit_message.fdf = 1U;
    transmit_message.esi = 0U;
    transmit_message.prio = 0U;
    transmit_message.data_bytes = length;
    transmit_message.id = id;

    can_mailbox_config(CAN0, 1U, &transmit_message);
}
void can2_txMessage(uint8_t length,uint8_t id,uint8_t *can2txdata)
{
    for(int i = 0; i < length; i++){
        can2transmit_message.data[i] = can2txdata[i];
    }
    can2transmit_message.rtr = 0U;
    can2transmit_message.ide = 0U;
    can2transmit_message.code = CAN_MB_TX_STATUS_DATA;
    can2transmit_message.brs = 0U;
    can2transmit_message.fdf = 0U;
    can2transmit_message.esi = 0U;
    can2transmit_message.prio = 0U;
    can2transmit_message.data_bytes = length;
    can2transmit_message.id = id;

    can_mailbox_config(CAN2, 3u, &can2transmit_message);
}

