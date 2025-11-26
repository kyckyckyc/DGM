/*
    Copyright 2021 codenocold codenocold@qq.com
    Address : https://github.com/codenocold/dgm
    This file is part of the dgm firmware.
    The dgm firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    The dgm firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pwm_curr.h"

uint16_t adc_buff[3];
int16_t  phase_a_adc_offset = 0;
int16_t  phase_b_adc_offset = 0;

void PWMC_init(void)
{
    /* Disable ADC interrupt */
    adc_interrupt_disable(ADC0, ADC_INT_EOIC);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);

    /* enable ADC0 */
    adc_enable(ADC0);
    /* Wait ADC0 startup */
    delay_ms(1);
    /* ADC0 calibration */
    adc_calibration_enable(ADC0);

    /* ADC software trigger enable */
//    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);

    /* ADC0 inject convert complete interrupt */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);

    /* enable ADC1 */
    adc_enable(ADC1);
    /* Wait ADC1 startup */
    delay_ms(1);
    /* ADC1 calibration */
    adc_calibration_enable(ADC1);

    /* Hold TIMER0 counter when core is halted */
    dbg_periph_enable(DBG_TIMER0_HOLD);

    /* Enable TIMER0 counter */
    timer_enable(TIMER0);

//    timer_repetition_value_config(TIMER0, 1);

    /* Set all duty to 50% */
    set_a_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));
    set_b_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));
    set_c_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);

    /* Main PWM Output Enable */
    timer_primary_output_config(TIMER0, ENABLE);
}

void PWMC_SwitchOnPWM(void)
{
    /* Set all duty to 50% */
    set_a_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));
    set_b_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));
    set_c_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));

    /* wait for a new PWM period */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    while (RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)) {
    };
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
}

void PWMC_SwitchOffPWM(void)
{
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);

    /* wait for a new PWM period */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    while (RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)) {
    };
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
}

void PWMC_TurnOnLowSides(void)
{
    /* Set all duty to 0% */
    set_a_duty(0);
    set_b_duty(0);
    set_c_duty(0);

    /* wait for a new PWM period */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    while (RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)) {
    };
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
}

int PWMC_CurrentReadingPolarization(void)
{
    int i         = 0;
    int adc_sum_a = 0;
    int adc_sum_b = 0;

    /* Clear Update Flag */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    /* Wait until next update */
    while (RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)) {
    };
    /* Clear Update Flag */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);

    while (i < 64) {
        if (timer_flag_get(TIMER0, TIMER_FLAG_UP) == SET) {
            timer_flag_clear(TIMER0, TIMER_FLAG_UP);

            i++;
            adc_sum_a += READ_IPHASE_A_ADC();
            adc_sum_b += READ_IPHASE_B_ADC();
        }
    }

    phase_a_adc_offset = adc_sum_a / i;
    phase_b_adc_offset = adc_sum_b / i;

    // offset check
    i                         = 0;
    const int Vout            = 2048;
    const int check_threshold = 200;
    if (phase_a_adc_offset > (Vout + check_threshold) || phase_a_adc_offset < (Vout - check_threshold)) {
        i = -1;
    }
    if (phase_b_adc_offset > (Vout + check_threshold) || phase_b_adc_offset < (Vout - check_threshold)) {
        i = -1;
    }

    return i;
}



// /* 可调参数：根据你的硬件/PWM 频率调整 */
// #define ARM_WAIT_PWM_CYCLES  3    /* 等待多少个 PWM 周期（一般 2~4 足够） */
// #define OFFSET_SAMPLES       64   /* 用于计算 offset 的采样点数（建议 32~128） */
// #define DISCARD_SAMPLES      8    /* 丢弃的首批不稳定采样数（1~20） */
// #define OPAMP_SAFE_DELAY_US  50   /* 额外的微秒延迟，保证运放/ADC S&H稳定（可调） */

// /* 你工程里可能已经有 delay_us、timer_flag_get/clear、adc_read_xxx 函数。
//    如果没有，请替换成你工程对应的 API。 */

// extern void PWMC_TurnOnLowSides(void);
// extern void delay_us(uint32_t us); /* 微秒延时 */
// extern int32_t adc_read_phaseA_raw(void); /* 返回原始 ADC 读数或已换算为电流计数值 */
// extern int32_t adc_read_phaseB_raw(void);
// extern int32_t adc_read_phaseC_raw(void);
// //extern void discard_adc_sample(void); /* 可选：如果你有专门的丢弃函数 */

// void measure_current_offsets(int32_t *off_a, int32_t *off_b, int32_t *off_c)
// {
//     int64_t sum_a = 0, sum_b = 0, sum_c = 0;
//     int32_t raw;

//     /* 丢弃前 DISCARD_SAMPLES 次采样（保证 S&H / ADC 稳定） */
//     for (int i = 0; i < DISCARD_SAMPLES; ++i) {
//         /* 如果 ADC 是触发型且需要在 PWM 同步下采样，确保这里的 adc_read_* 是阻塞型读取 */
//         raw = adc_read_phaseA_raw(); (void)raw;
//         raw = adc_read_phaseB_raw(); (void)raw;
//         raw = adc_read_phaseC_raw(); (void)raw;
//     }

//     /* 采样 OFFSET_SAMPLES 次并求均值 */
//     for (int i = 0; i < OFFSET_SAMPLES; ++i) {
//         sum_a += adc_read_phaseA_raw();
//         sum_b += adc_read_phaseB_raw();
//         sum_c += adc_read_phaseC_raw();
//     }

//     *off_a = (int32_t)(sum_a / OFFSET_SAMPLES);
//     *off_b = (int32_t)(sum_b / OFFSET_SAMPLES);
//     *off_c = (int32_t)(sum_c / OFFSET_SAMPLES);
// }

// /* 等待 N 个 PWM 周期的辅助（基于你原先的 timer_flag_get 用法） */
// static void wait_pwm_cycles(uint32_t cycles)
// {
//     for (uint32_t i = 0; i < cycles; ++i) {
//         timer_flag_clear(TIMER0, TIMER_FLAG_UP);
//         while (RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)) {
//             /* 等待下一个 PWM 更新 */
//         }
//     }
// }