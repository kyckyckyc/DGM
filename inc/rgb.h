#ifndef _RGB_H
#define _RGB_H
#include <string.h>
#include "main.h"
#include <stdint.h>
// WS2812B引脚定义
#define WS2812B_PIN    GPIO_PIN_5
#define WS2812B_PORT   GPIOB
#define LED_COUNT 1
// 颜色结构体
typedef struct {
    uint8_t g;  // 绿色
    uint8_t r;  // 红色  
    uint8_t b;  // 蓝色
} rgb_color_t;


void delay_ns_216mhz(uint32_t ns);
void delay_us_216mhz(uint32_t us);


void ws2812b_delay_ns(uint32_t ns);
void ws2812b_send_bit(uint8_t bit);
void ws2812b_send_byte(uint8_t byte);
void ws2812b_send_color(rgb_color_t color);
void ws2812b_reset(void);
void ws2812b_init(void);
void ws2812b_set_led(uint8_t led_index, rgb_color_t color, rgb_color_t *led_buffer);
void ws2812b_set_all(rgb_color_t color, rgb_color_t *led_buffer, uint16_t led_count);
void ws2812b_update(rgb_color_t *led_buffer, uint16_t led_count);
void hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, rgb_color_t *color);
void rainbow_effect(rgb_color_t *led_buffer, uint16_t led_count, uint8_t hue_offset);
void breathing_effect(rgb_color_t *led_buffer, uint16_t led_count, rgb_color_t color, uint8_t brightness);
void running_light(rgb_color_t *led_buffer, uint16_t led_count, rgb_color_t color, uint8_t position);
void simple_rainbow_demo(void);
void single_led_hsv_gradient(void);
void single_led_color_cycle(void);
void debug_gradient_step_by_step(void);
void debug_gradient(void);
#endif