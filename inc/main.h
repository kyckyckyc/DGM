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

#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32g5x3.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

//#define __DEBUG__

#ifdef __DEBUG__
// clang-format off
    #include "SEGGER_RTT.h"
    #define DEBUG(format, ...) SEGGER_RTT_printf(0, format, ##__VA_ARGS__);
    inline void DEBUG_PLOT(float f1, float f2) { float value[2] = {f1, f2}; SEGGER_RTT_Write(1, &value, sizeof(value)); }
// clang-format on
#else
#define DEBUG(format, ...)
#define DEBUG_PLOT(value)
#endif

typedef struct {
    uint32_t mailboxes[3];      /* 可用的发送邮箱 */
    uint8_t current_index;      /* 当前使用的邮箱索引 */
    uint32_t sent_count[3];     /* 每个邮箱的发送计数 */
} can2_tx_manager_t;
extern volatile uint32_t SystickCount;
extern can_mailbox_descriptor_struct transmit_message;
extern can_mailbox_descriptor_struct receive_message;
extern can_mailbox_descriptor_struct can2transmit_message;
extern can_mailbox_descriptor_struct can2receive_message;
extern can2_tx_manager_t can2_tx_manager;
// LED ACT
#define LED_ACT_SET()        GPIO_BOP(GPIOC) = (uint32_t) GPIO_PIN_13
#define LED_ACT_RESET()      GPIO_BC(GPIOC) = (uint32_t) GPIO_PIN_13
#define LED_ACT_GET()        (GPIO_OCTL(GPIOC) & (GPIO_PIN_13))

// SPI0 NCS0
#define ENC_NCS_SET()        GPIO_BOP(GPIOA) = (uint32_t) GPIO_PIN_4
#define ENC_NCS_RESET()      GPIO_BC(GPIOA) = (uint32_t) GPIO_PIN_4

// SPI0 NCS1
#define ENC_NCS1_SET()        GPIO_BOP(GPIOC) = (uint32_t) GPIO_PIN_4
#define ENC_NCS1_RESET()      GPIO_BC(GPIOC) = (uint32_t) GPIO_PIN_4

/* FLASH MAP ---------------------------------------------*/
#define PAGE_SIZE            ((uint32_t) 0x400U) // 1KB

#define APP_MAIN_ADDR        ((uint32_t) (0x8000000 + 0 * PAGE_SIZE))  // Page 0
#define APP_BACK_ADDR        ((uint32_t) (0x8000000 + 50 * PAGE_SIZE)) // Page 50
#define APP_MAX_SIZE         ((uint32_t) (50 * PAGE_SIZE))             // 50KB

#define BOOTLOADER_ADDR      ((uint32_t) (0x8000000 + 100 * PAGE_SIZE)) // Page 100
#define BOOTLOADER_MAX_SIZE  ((uint32_t) (10 * PAGE_SIZE))              // 10KB

#define USR_CONFIG_ADDR      ((uint32_t) (0x8000000 + 110 * PAGE_SIZE)) // Page 110
#define USR_CONFIG_MAX_SIZE  ((uint32_t) (5 * PAGE_SIZE))               // 5KB

#define COGGING_MAP_ADDR     ((uint32_t) (0x8000000 + 115 * PAGE_SIZE)) // Page 115
#define COGGING_MAP_MAX_SIZE ((uint32_t) (10 * PAGE_SIZE))              // 10KB

/* Exported functions prototypes ---------------------------------------------*/
static inline void watch_dog_feed(void)
{
    FWDGT_CTL = FWDGT_KEY_RELOAD;
}

static inline uint32_t get_ms_since(uint32_t tick)
{
    return (uint32_t) ((SystickCount - tick) / 2U);
}

void Error_Handler(void);
void delay_ms(const uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
